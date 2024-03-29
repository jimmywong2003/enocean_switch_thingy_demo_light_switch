/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>

/* HAL */
#include "boards.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_client.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "example_common.h"
#include "ble_softdevice_support.h"
#include "enocean.h"

#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)

static generic_onoff_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool                   m_device_provisioned;

/* Forward declaration */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);


#define SWITCH_DEBOUNCE_INTERVAL_US (MS_TO_US(500))

#define ENOCEAN_ADDRESS {0xD4, 0x62, 0x00, 0x00, 0x15, 0xe2};

const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

typedef struct
{
    bool     a0;
    uint32_t a0_ts;
    bool     a1;
    uint32_t a1_ts;
    bool     b0;
    uint32_t b0_ts;
    bool     b1;
    uint32_t b1_ts;
} app_switch_status_t;
static app_switch_status_t m_switch_state;

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(BSP_LED_2_MASK  | BSP_LED_3_MASK,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_blink_stop();
    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}


static bool m_client_onoff_state[4];

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    set_params.tid = tid++;
    transition_params.delay_ms = APP_CONFIG_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_CONFIG_ONOFF_TRANSITION_TIME_MS;

    

    switch(button_number)
    {
        case 0:
            m_client_onoff_state[0] = !m_client_onoff_state[0];
            set_params.on_off = m_client_onoff_state[0];
            status = generic_onoff_client_set_unack(&m_clients[0], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            hal_led_pin_set(BSP_LED_0, set_params.on_off);
            break;
        case 2:
            m_client_onoff_state[1] = !m_client_onoff_state[1];
            set_params.on_off = m_client_onoff_state[1];
            status = generic_onoff_client_set_unack(&m_clients[1], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            hal_led_pin_set(BSP_LED_1, set_params.on_off);
            break;

        case 1:
            m_client_onoff_state[2] = !m_client_onoff_state[2];
            set_params.on_off = m_client_onoff_state[2];
            status = generic_onoff_client_set_unack(&m_clients[2], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            hal_led_pin_set(BSP_LED_2, set_params.on_off);
            break;
        case 3:
            m_client_onoff_state[3] = !m_client_onoff_state[3];
            set_params.on_off = m_client_onoff_state[3];
            status = generic_onoff_client_set_unack(&m_clients[3], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            hal_led_pin_set(BSP_LED_3, set_params.on_off);
            break;
    }



#if 0
    switch (button_number)
    {
        case 0:
        case 1:
            /* Demonstrate acknowledged transaction, using 1st client model instance */
            /* In this examples, users will not be blocked if the model is busy */
            (void)access_model_reliable_cancel(m_clients[0].model_handle);
            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
            hal_led_pin_set(BSP_LED_0, set_params.on_off);
            break;

        case 2:
        case 3:
            /* Demonstrate un-acknowledged transaction, using 2nd client model instance */
            status = generic_onoff_client_set_unack(&m_clients[1], &set_params,
                                                    &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
            hal_led_pin_set(BSP_LED_1, set_params.on_off);
            break;
    }
#endif

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", button_number);
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", button_number);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void app_switch_debounce(enocean_switch_status_t * p_status)
{
    /* Toggle state on the unicast server address using 1st on/off client */
    if (p_status->a0 && timer_diff(timer_now(), m_switch_state.a0_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.a0_ts = timer_now();
        m_switch_state.a0 = !m_switch_state.a0;
        button_event_handler(0);
    }
    /* Toggle state on the second unicast server address using 2nd on/off client */
    else if (p_status->a1 && timer_diff(timer_now(), m_switch_state.a1_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.a1_ts = timer_now();
        m_switch_state.a1 = !m_switch_state.a1;
        button_event_handler(1);
    }
 
    /* Toggle state on the nodes subscribed to the Odd group address using 3rd on/off client */
    if (p_status->b0 && timer_diff(timer_now(), m_switch_state.b0_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.b0_ts = timer_now();
        m_switch_state.b0 = !m_switch_state.b0;
        button_event_handler(2);
    }
    /* Toggle state on the nodes subscribed to the Even group address using 3th on/off client */
    else if (p_status->b1 && timer_diff(timer_now(), m_switch_state.b1_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.b1_ts = timer_now();
        m_switch_state.b1 = !m_switch_state.b1;
        button_event_handler(3);
    }
}

static void rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;

        ERROR_CHECK(generic_onoff_client_init(&m_clients[i], i + 1));
    }
}

static void m_app_cb(enocean_evt_t * p_evt)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sw data A0: %d A1: %d B0: %d B1: %d Action: %d\n",
    p_evt->params.data.status.a0,p_evt->params.data.status.a1,p_evt->params.data.status.b0,
    p_evt->params.data.status.b1,p_evt->params.data.status.action);

    /*
    if(p_evt->params.data.status.a0 == 1)
    {
        button_event_handler(0);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 0 activate\n");
    }
    if(p_evt->params.data.status.a1 == 1)
    {
        button_event_handler(1);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 1 activate\n");
    }

    if(p_evt->params.data.status.b1 == 1)
    {
        button_event_handler(2);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 2 activate\n");
    }
    if(p_evt->params.data.status.b0 == 1)
    {
        button_event_handler(3);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button 3 activate\n");
    }
    */
    app_switch_debounce(&p_evt->params.data.status);
}
static void rx_cb(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{


    uint8_t enocean_mac[6] = ENOCEAN_ADDRESS;

    enocean_packet_t p_pkt;
    enocean_evt_t evt;
    if (memcmp(p_rx_data->p_metadata->params.scanner.adv_addr.addr, enocean_mac, 6) == 0){
        enocean_data_packet_t * p_data = (enocean_data_packet_t *)p_rx_data->p_payload;
        p_pkt.type = ENOCEAN_DATA_PACKET;
        p_pkt.data.data_packet.seq = p_data->seq;
        p_pkt.p_ble_gap_addr = p_rx_data->p_metadata->params.scanner.adv_addr.addr;
        p_pkt.data.data_packet.p_raw_data = (uint8_t *)p_data;
        p_pkt.data.data_packet.length = p_rx_data->length - PTM215B_DATA_PACKET_MIC_SIZE;
        p_pkt.data.data_packet.p_mic  = &p_rx_data->p_payload[p_pkt.data.data_packet.length];

        uint8_t sw_data = p_pkt.data.data_packet.p_raw_data[PTM215B_DATA_PACKET_SWITCH_STATUS_OFFSET] & ~PTM215B_SWITCH_STATUS_RFU_MASK;
        evt.params.data.status.a0 = (sw_data >> PTM215B_SWITCH_STATUS_A0_BIT) & 0x01;
        evt.params.data.status.a1 = (sw_data >> PTM215B_SWITCH_STATUS_A1_BIT) & 0x01;
        evt.params.data.status.b0 = (sw_data >> PTM215B_SWITCH_STATUS_B0_BIT) & 0x01;
        evt.params.data.status.b1 = (sw_data >> PTM215B_SWITCH_STATUS_B1_BIT) & 0x01;
        evt.params.data.status.action = ((sw_data >> PTM215B_SWITCH_STATUS_ACTION_BIT) & 0x01) ?
                                        RELEASE_ACTION : PRESS_ACTION;
        evt.params.data.p_optional_data = &p_pkt.data.data_packet.p_raw_data[PTM215B_DATA_PACKET_OPTIONAL_DATA_OFFSET];
        evt.params.data.optional_data_length = p_pkt.data.data_packet.length
                                            - PTM215B_DATA_PACKET_OPTIONAL_DATA_OFFSET;

        m_app_cb(&evt);

          

    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));
    nrf_mesh_rx_cb_set(rx_cb);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}


static void start(void)
{
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_LS_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}
uint8_t m_reset = 0;
#include "nrf_delay.h"
int main(void)
{
    nrf_gpio_cfg_input(25, NRF_GPIO_PIN_PULLUP);
    nrf_delay_ms(100);
    initialize();
    if (nrf_gpio_pin_read(25) == 0) {
        m_reset = 1;
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset\n");
    }
    start();

    for (;;)
    {
        if (m_reset == 1 && m_device_provisioned)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "prepare to run the node reset\n");
            m_reset = 0;
            mesh_stack_config_clear();
            nrf_delay_ms(100);
            node_reset();
        }
        (void)sd_app_evt_wait();
    }
}
