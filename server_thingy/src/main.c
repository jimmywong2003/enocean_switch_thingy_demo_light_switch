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
//#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_onoff.h"
#include "ble_softdevice_support.h"

/* Thingy dependencies */
#include "pca20020.h"
#include "app_scheduler.h"
#include "drv_ext_light.h"
#include "drv_ext_gpio.h"
#include "support_func.h"
#include "nrf_delay.h"
#include "twi_manager.h"
#include "m_ui_demo.h"
#include "app_button.h"
#include "nrf_drv_gpiote.h"
#include "macros_common.h"

#define SCHED_MAX_EVENT_DATA_SIZE   MAX(APP_TIMER_SCHED_EVENT_DATA_SIZE, 20) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            180//60  /**< Maximum number of events in the scheduler queue. */


#define ONOFF_SERVER_0_LED          (BSP_LED_0)
#define APP_ONOFF_ELEMENT_INDEX     (0)

static bool m_device_provisioned;

/*************************************************************************************************/
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);

/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_CONFIG_FORCE_SEGMENTATION,
                     APP_CONFIG_MIC_SIZE,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb)

uint8_t m_reset;
ble_uis_led_t led_breath_red = {.mode = BLE_UIS_LED_MODE_BREATHE, 
                                .data.mode_breathe.color_mix = DRV_EXT_LIGHT_COLOR_RED,
                                .data.mode_breathe.intensity  = 10,
                                .data.mode_breathe.delay = 100};

ble_uis_led_t led_const_off = {.mode = BLE_UIS_LED_MODE_CONST, 
                                .data.mode_const.r = 0x00,
                                .data.mode_const.g = 0x00,
                                .data.mode_const.b = 0x00};

ble_uis_led_t led_const_on = {.mode = BLE_UIS_LED_MODE_CONST, 
                                .data.mode_const.r = 0xff,
                                .data.mode_const.g = 0xff,
                                .data.mode_const.b = 0xff};

ble_uis_led_t led_breath_blue = {.mode = BLE_UIS_LED_MODE_BREATHE, 
                                .data.mode_breathe.color_mix = DRV_EXT_LIGHT_COLOR_BLUE,
                                .data.mode_breathe.intensity  = DEFAULT_LED_INTENSITY_PERCENT,
                                .data.mode_breathe.delay = DEFAULT_LED_OFF_TIME_MS};

ble_uis_led_t led_breath_quick_blue = {.mode = BLE_UIS_LED_MODE_BREATHE, 
                                .data.mode_breathe.color_mix = DRV_EXT_LIGHT_COLOR_BLUE,
                                .data.mode_breathe.intensity  = DEFAULT_LED_INTENSITY_PERCENT,
                                .data.mode_breathe.delay = 100};


ble_uis_led_t led_breath_white = {.mode = BLE_UIS_LED_MODE_BREATHE, 
                                .data.mode_breathe.color_mix = DRV_EXT_LIGHT_COLOR_WHITE,
                                .data.mode_breathe.intensity  = DEFAULT_LED_INTENSITY_PERCENT,
                                .data.mode_breathe.delay = DEFAULT_LED_OFF_TIME_MS};

ble_uis_led_t led_light_blue = {.mode = BLE_UIS_LED_MODE_CONST, 
                                .data.mode_const.r = 0,
                                .data.mode_const.g = 0,
                                .data.mode_const.b = 20,
                                };

int16_t m_dimming_counter = 0;
uint8_t m_count_direction = 0;
static bool m_led_status = false;
static const nrf_drv_twi_t     m_twi_sensors = NRF_DRV_TWI_INSTANCE(TWI_SENSOR_INSTANCE);

APP_TIMER_DEF(dim_id);

void dimming_handler()
{
  if(m_count_direction == 1){
      if(m_dimming_counter >= 250){
          app_timer_stop(dim_id);  
          m_dimming_counter = 250;
      }else{
          m_dimming_counter+=10;
      }
      drv_ext_light_intensity_set(1, m_dimming_counter);
      
  }else{
      if(m_dimming_counter <= 0){
          app_timer_stop(dim_id); 
          m_dimming_counter = 0;
           
      }else{
          m_dimming_counter-=10;
      }
      drv_ext_light_intensity_set(1, m_dimming_counter);      
  }
}

void dimmming_off()
{
//    drv_ext_light_intensity_set(1, 0);

    app_timer_stop(dim_id);
    NRF_LOG_INFO("enter dimm off\n");
    m_count_direction = 0;
    app_timer_start(dim_id, APP_TIMER_TICKS(40), NULL);

}

void dimmming_on()
{
//    drv_ext_light_intensity_set(1, 250);

    app_timer_stop(dim_id);
    NRF_LOG_INFO("enter dimm on\n");
    m_count_direction = 1;
    app_timer_start(dim_id, APP_TIMER_TICKS(40), NULL);

}

/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */
    ret_code_t err_code;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got SET command to %u\n", onoff);
    m_led_status = onoff;
    if(m_led_status == true)
    {   
        dimmming_on();
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Set LED, error = %X\n", err_code);
    }
    else
    {
        dimmming_off();
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Clear LED, error = %X\n", err_code);
    }
}

/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    *p_present_onoff = m_led_status;
}

static void app_model_init(void)
{
    /* Instantiate onoff server on element index 0 */
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, 0));
}

/*************************************************************************************************/

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    //hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

void board_init()
{
    uint32_t            err_code;
    drv_ext_gpio_init_t ext_gpio_init;
    static const nrf_drv_twi_config_t twi_config =
    {
        .scl                = TWI_SCL,
        .sda                = TWI_SDA,
        .frequency          = NRF_TWI_FREQ_400K,
        .interrupt_priority = NRF_MESH_IRQ_PRIORITY_THREAD
    };

    static const drv_sx1509_cfg_t sx1509_cfg =
    {
        .twi_addr       = SX1509_ADDR,
        .p_twi_instance = &m_twi_sensors,
        .p_twi_cfg      = &twi_config
    };

    ext_gpio_init.p_cfg = &sx1509_cfg;
    
    err_code = support_func_configure_io_startup(&ext_gpio_init);
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(100);
}


void thingy_init()
{   
    ret_code_t err_code;
    m_ui_init_t              ui_params;
    /**@brief Initialize the TWI manager. */
    err_code = twi_manager_init(NRF_MESH_IRQ_PRIORITY_THREAD);
    APP_ERROR_CHECK(err_code);

    /**@brief Initialize LED and button UI module. */
    ui_params.p_twi_instance = &m_twi_sensors;
    err_code = m_ui_init(&ui_params);
    APP_ERROR_CHECK(err_code);

}

static void button_evt_handler(uint8_t pin_no, uint8_t button_action)
{
    NRF_LOG_INFO("Button pressed, pin = %d, action = %d\n", pin_no, button_action);
    uint32_t err_code;
    static uint8_t led_state = 0;
    if(button_action == 1)
    {
        if(m_led_status == true)
        {
            m_led_status = false;
            dimmming_off();
        }
        else
        {
            m_led_status = true;
            dimmming_on();
        }
        app_onoff_status_publish(&m_onoff_server_0);
    }
}

static ret_code_t button_init(void)
{
    ret_code_t err_code;

    /* Configure gpiote for the sensors data ready interrupt. */
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        RETURN_IF_ERROR(err_code);
    }

    static const app_button_cfg_t button_cfg =
    {
        .pin_no         = BUTTON,
        .active_state   = APP_BUTTON_ACTIVE_LOW,
        .pull_cfg       = NRF_GPIO_PIN_PULLUP,
        .button_handler = button_evt_handler
    };
    app_timer_create(&dim_id, APP_TIMER_MODE_REPEATED, dimming_handler);

    err_code = app_button_init(&button_cfg, 1, APP_TIMER_TICKS(50));
    RETURN_IF_ERROR(err_code);

    return app_button_enable();
}


static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 0:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "User action \n");
            //hal_led_pin_set(ONOFF_SERVER_0_LED, !hal_led_pin_get(ONOFF_SERVER_0_LED));
            app_onoff_status_publish(&m_onoff_server_0);
            break;
        }

        /* Initiate node reset */
        case 3:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
            break;
        }

        default:
            break;
    }
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '4')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    led_set(&led_light_blue, NULL);
}

static void provisioning_aborted_cb(void)
{
    led_set(&led_breath_red, NULL);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
    led_set(&led_const_off, NULL);
#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
    led_set(&led_breath_white, NULL);
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_THREAD,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));

}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER | LOG_SRC_NETWORK, LOG_LEVEL_DBG3, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Thingy Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    board_init();
    thingy_init();
    button_init();

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();
}

static void start(void)
{
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
            .p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
        led_set(&led_breath_red, NULL);
    }else{
        NRF_LOG_INFO("This node has been provisioned\n");
        led_set(&led_breath_white, NULL);
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());
    ERROR_CHECK(mesh_stack_start());
}

int main(void) {
    nrf_gpio_cfg_input(BUTTON, NRF_GPIO_PIN_PULLUP);
    NRF_LOG_INIT(NULL);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
//    if (nrf_gpio_pin_read(BUTTON) == 0) {
//        m_reset = 1;
//        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reset\n");
//    }
    nrf_delay_ms(100);
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    initialize();
    start();
    for (;;) {
        if (m_reset == 1 && m_device_provisioned)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "prepare to run the node reset\n");
            m_reset = 0;
            proxy_stop();
            mesh_stack_config_clear();
            nrf_delay_ms(100);
            node_reset();
        }

        app_sched_execute();
        bool done;
        done = nrf_mesh_process();
        NRF_LOG_FLUSH();
        if (done) {
            (void)sd_app_evt_wait();
        }
    }
}