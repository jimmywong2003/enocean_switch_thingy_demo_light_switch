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
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "nrf_mesh_utils.h"
#include "device_state_manager.h"
#include "timer_scheduler.h"

#include "config_client.h"
#include "config_server.h"
#include "access_config.h"
#include "generic_level_server.h"
#include "generic_level_client.h"
#include "generic_onoff_server.h"
#include "generic_onoff_client.h"
#include "health_common.h"

#include "provisioner_helper.h"
#include "node_setup.h"
#include "example_common.h"
#include "example_network_config.h"
#include "light_switch_example_common.h"
#include "mesh_app_utils.h"

#include "log.h"
#include "nrf_mesh_assert.h"

/* USER_NOTE: Add more steps here is you want to customize the nodes further. */
/* Node setup steps */
typedef enum
{
    NODE_SETUP_IDLE,
    NODE_SETUP_CONFIG_COMPOSITION_GET,
    NODE_SETUP_CONFIG_APPKEY_ADD,
    NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH,
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_SERVER,
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT,
    NODE_SETUP_CONFIG_PUBLICATION_HEALTH,
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_SERVER,
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT1,
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT2,
    NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_SERVER,
    NODE_SETUP_DONE,
} config_steps_t;

/* Structure to hold the composition data */
typedef struct
{
    uint16_t    len;
    struct
    {
        uint8_t page_number;
        uint8_t data[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
    }composition;
} composition_data_t;

/* Expected status structure, used for setup state machine */
typedef struct
{
    uint8_t  num_statuses;
    uint16_t expected_opcode;
    const access_status_t  * p_statuses;
} expected_status_list_t;

typedef struct
{
    uint8_t count;
    timer_event_t timer;
} client_send_retry_t;

typedef enum
{
    STATUS_CHECK_PASS,
    STATUS_CHECK_FAIL,
    STATUS_CHECK_UNEXPECTED_OPCODE
} status_check_t;

/* USER_NOTE:
You can define one or more such configuration steps for a given node in your network. The choice
of the steps can be done in @ref setup_select_steps() function.
*/
/* Sequence of steps for the client nodes */
static const config_steps_t client_config_steps[] =
{
    NODE_SETUP_CONFIG_COMPOSITION_GET,
    NODE_SETUP_CONFIG_APPKEY_ADD,
    NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH,
    NODE_SETUP_CONFIG_PUBLICATION_HEALTH,
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT,
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT,
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT1,
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT2,
    NODE_SETUP_DONE
};

/* Sequence of steps for the server nodes */
static const config_steps_t server_config_steps[] =
{
    NODE_SETUP_CONFIG_COMPOSITION_GET,
    NODE_SETUP_CONFIG_APPKEY_ADD,
    NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH,
    NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_SERVER,
    NODE_SETUP_CONFIG_PUBLICATION_ONOFF_SERVER,
    NODE_SETUP_CONFIG_PUBLICATION_HEALTH,
    NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_SERVER,
    NODE_SETUP_DONE
};


static uint16_t m_current_node_addr;
static uint16_t m_retry_count;
static client_send_retry_t m_send_timer;
static const uint8_t * mp_appkey;
static uint16_t m_appkey_idx;
static access_model_id_t m_client_model_id;
static access_model_id_t m_server_model_id;


static const config_steps_t m_idle_step = NODE_SETUP_IDLE;
static const config_steps_t * mp_config_step = &m_idle_step;
static node_setup_successful_cb_t m_node_setup_success_cb;
static node_setup_failed_cb_t m_node_setup_failed_cb;
static expected_status_list_t m_expected_status_list;

/* Forward declaration */
static void config_step_execute(void);

/*************************************************************************************************/

static void node_setup_state_clear(void)
{
    timer_sch_abort(&m_send_timer.timer);
    mp_config_step = &m_idle_step;
}

static void node_setup_succeed(void)
{
    node_setup_state_clear();
    m_node_setup_success_cb();
}

static void node_setup_fail(void)
{
    node_setup_state_clear();
    m_node_setup_failed_cb();
}

/* Set expected status opcode and acceptable value of status codes */
static void expected_status_set(uint32_t opcode, uint32_t n, const access_status_t * p_list)
{
    if (n > 0)
    {
        NRF_MESH_ASSERT(p_list != NULL);
    }

    m_expected_status_list.expected_opcode = opcode;
    m_expected_status_list.num_statuses = n;
    m_expected_status_list.p_statuses = p_list;
}

static void config_step_retry_schedule(void)
{
    NRF_MESH_ASSERT_DEBUG(m_send_timer.count > 0);
    m_send_timer.count--;

    timestamp_t next_timeout = timer_now() + MS_TO_US(CLIENT_BUSY_SEND_RETRY_DELAY_MS);
    timer_sch_reschedule(&m_send_timer.timer, next_timeout);
}

/* Callback for the timer event */
static void client_send_timer_cb(timestamp_t timestamp, void * p_context)
{
    /* retry the last step */
    config_step_execute();
}

/**
 *  When config client status message is received, this function checks for the expected opcode, and
 *  status values. It is required by the node setup state machine.
 */
static status_check_t check_expected_status(uint16_t rx_opcode, const config_msg_t * p_msg)
{
    uint8_t status = 0xFF;
    if (rx_opcode != m_expected_status_list.expected_opcode)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Unexpected opcode: exp 0x%04x  rx 0x%04x\n",
              m_expected_status_list.expected_opcode, rx_opcode);
        return STATUS_CHECK_UNEXPECTED_OPCODE;
    }

    switch (rx_opcode)
    {
        /* COMPOSITION_DATA_STATUS does not have a STATUS field */
        case CONFIG_OPCODE_COMPOSITION_DATA_STATUS:
            break;

        case CONFIG_OPCODE_MODEL_APP_STATUS:
            status = p_msg->app_status.status;
            break;

        case CONFIG_OPCODE_MODEL_PUBLICATION_STATUS:
            status = p_msg->publication_status.status;
            break;

        case CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS:
            status = p_msg->subscription_status.status;
            break;

        case CONFIG_OPCODE_APPKEY_STATUS:
            status = p_msg->appkey_status.status;
            break;

        default:
            /** USER_TO_CONFIGURE: Resolve additional required statuses in above switch case */
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Handle additional statuses here");
            ERROR_CHECK(NRF_ERROR_NOT_FOUND);
            break;
    }

    if (m_expected_status_list.num_statuses == 0)
    {
        return STATUS_CHECK_PASS;
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "opcode status field: %d \n", status);
    for (uint32_t i = 0; i < m_expected_status_list.num_statuses; i++)
    {
        if (status == m_expected_status_list.p_statuses[i])
        {
            return STATUS_CHECK_PASS;
        }
    }
    return STATUS_CHECK_FAIL;
}
/*************************************************************************************************/
/* Application-specific functions for combining some commonly used structure assignments */

static void client_pub_state_set(config_publication_state_t *p_pubstate, uint16_t element_addr,
                                     uint16_t publish_addr)
{
    p_pubstate->element_address = element_addr;
    p_pubstate->publish_address.type = nrf_mesh_address_type_get(publish_addr);
    p_pubstate->publish_address.value = publish_addr;
    p_pubstate->appkey_index = m_appkey_idx;
    p_pubstate->frendship_credential_flag = false;
    p_pubstate->publish_ttl = (SERVER_NODE_COUNT > NRF_MESH_TTL_MAX ? NRF_MESH_TTL_MAX : SERVER_NODE_COUNT);
    p_pubstate->publish_period.step_num = 0;
    p_pubstate->publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_100MS;
    p_pubstate->retransmit_count = 1;
    p_pubstate->retransmit_interval = 0;
    p_pubstate->model_id.company_id = m_client_model_id.company_id;
    p_pubstate->model_id.model_id = m_client_model_id.model_id;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Set: on/off client: 0x%04x  pub addr: 0x%04x\n",
            p_pubstate->element_address, p_pubstate->publish_address.value);
}


/*************************************************************************************************/
/* Node setup functionality related static functions */
/* USER_NOTE:
You can code any suitable logic here to select the configuration steps for a given
node in your network.
*/
/**
 * Selects the configuration steps for the node.
 *
 * In this example, first device is always a light-switch Client. Subsequent devices are servers
 * First two servers will act as unicast devices and other servers will be subscribed to the
 * ODD or EVEN groups.
 *
 * @param[in]  addr    Address of the device being configured.
 *
 */
static void setup_select_steps(uint16_t addr)
{
    if (addr == UNPROV_START_ADDRESS)
    {
        mp_config_step = client_config_steps;
    }
    else
    {
        mp_config_step = server_config_steps;
    }
}


/** Step execution function for the configuration state machine. */
static void config_step_execute(void)
{
    uint32_t status = NRF_ERROR_INVALID_STATE;
    static uint16_t model_element_addr = 0;

    /* This example configures the provisioned nodes in the following way
     * Node 0: Client node (Switch)
     * Node 1,2,3 ...: Server nodes (Lights)
     *
     * Group Even: All nodes with even address
     * Group Odd: All nodes with odd address
     */

    /* Customize configuration steps for client vs. server nodes.
     * For client nodes: Skip, usual publication and subscription
     */

    switch (*mp_config_step)
    {
        /* Read the composition data from the node: */
        case NODE_SETUP_CONFIG_COMPOSITION_GET:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Getting composition data\n");
            status = config_client_composition_data_get(0x00);

            expected_status_set(CONFIG_OPCODE_COMPOSITION_DATA_STATUS, 0, NULL);
            break;
        }

        /* Add the application key to the node: */
        case NODE_SETUP_CONFIG_APPKEY_ADD:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding appkey\n");
            status = config_client_appkey_add(NETKEY_INDEX, m_appkey_idx, mp_appkey);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS, ACCESS_STATUS_KEY_INDEX_ALREADY_STORED};
            expected_status_set(CONFIG_OPCODE_APPKEY_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        /* Bind the health server to the application key: */
        case NODE_SETUP_CONFIG_APPKEY_BIND_HEALTH:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App key bind: Health server\n");
            access_model_id_t model_id;
            model_id.company_id = ACCESS_COMPANY_ID_NONE;
            model_id.model_id = HEALTH_SERVER_MODEL_ID;
            uint16_t element_address = m_current_node_addr;
            status = config_client_model_app_bind(element_address, m_appkey_idx, model_id);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_APP_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        /* Bind the On/Off server to the application key: */
        case NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_SERVER:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App key bind: 0x%04x server\n", m_server_model_id.model_id);
            access_model_id_t model_id;
            model_id.company_id = m_server_model_id.company_id;
            model_id.model_id = m_server_model_id.model_id;
            uint16_t element_address = m_current_node_addr;
            status = config_client_model_app_bind(element_address, m_appkey_idx, model_id);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_APP_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        /* Bind the On/Off client to the application key: */
        case NODE_SETUP_CONFIG_APPKEY_BIND_ONOFF_CLIENT:
        {
            if (model_element_addr == 0)
            {
                model_element_addr = m_current_node_addr + 1;
            }
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App key bind: 0x%04x client on element 0x%04x\n", m_client_model_id.model_id, model_element_addr);
            access_model_id_t model_id;
            model_id.company_id = m_client_model_id.company_id;
            model_id.model_id = m_client_model_id.model_id;
            uint16_t element_address = model_element_addr;
            status = config_client_model_app_bind(element_address, m_appkey_idx, model_id);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_APP_STATUS, ARRAY_SIZE(exp_status), exp_status);

            if (status == NRF_SUCCESS)
            {
                model_element_addr++;
            }
            break;
        }

        /* Configure the publication parameters for the Health server: */
        case NODE_SETUP_CONFIG_PUBLICATION_HEALTH:
        {
            config_publication_state_t pubstate = {0};
            pubstate.element_address = m_current_node_addr;
            pubstate.publish_address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            pubstate.publish_address.value = PROVISIONER_ADDRESS;
            pubstate.appkey_index = 0;
            pubstate.frendship_credential_flag = false;
            pubstate.publish_ttl = (SERVER_NODE_COUNT > NRF_MESH_TTL_MAX ? NRF_MESH_TTL_MAX : SERVER_NODE_COUNT);
            pubstate.publish_period.step_num = 1;
            pubstate.publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_10S;
            pubstate.retransmit_count = 1;
            pubstate.retransmit_interval = 0;
            pubstate.model_id.company_id = ACCESS_COMPANY_ID_NONE;
            pubstate.model_id.model_id = HEALTH_SERVER_MODEL_ID;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting publication address for the health server to 0x%04x\n", pubstate.publish_address.value);
            status = config_client_model_publication_set(&pubstate);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        /* Configure the publication parameters for the On/Off servers to publish to the
        corresponding On/Off clients (ODD servers: Client on Element 1 and EVEN servers: Client on
        element 2, of the client example). This demonstrates the state change publication due to local
        event on the server. */
        case NODE_SETUP_CONFIG_PUBLICATION_ONOFF_SERVER:
        {
            config_publication_state_t pubstate = {0};
            pubstate.element_address = m_current_node_addr;
            pubstate.publish_address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            if ((m_current_node_addr % 2) == 0)
            {
                pubstate.publish_address.value = UNPROV_START_ADDRESS + ELEMENT_IDX_ONOFF_CLIENT2;
            }
            else
            {
                pubstate.publish_address.value = UNPROV_START_ADDRESS + ELEMENT_IDX_ONOFF_CLIENT1;
            }
            pubstate.appkey_index = m_appkey_idx;
            pubstate.frendship_credential_flag = false;
            pubstate.publish_ttl = (SERVER_NODE_COUNT > NRF_MESH_TTL_MAX ? NRF_MESH_TTL_MAX : SERVER_NODE_COUNT);
            pubstate.publish_period.step_num = 0;
            pubstate.publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_100MS;
            pubstate.retransmit_count = 1;
            pubstate.retransmit_interval = 0;
            pubstate.model_id.company_id = m_server_model_id.company_id;
            pubstate.model_id.model_id = m_server_model_id.model_id;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Set: 0x%04x server pub addr: 0x%04x\n", m_server_model_id.model_id, pubstate.publish_address.value);
            status = config_client_model_publication_set(&pubstate);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        /* Configure subscription address for the On/Off server */
        case NODE_SETUP_CONFIG_SUBSCRIPTION_ONOFF_SERVER:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Adding subscription\n");
            uint16_t element_address = m_current_node_addr;
            nrf_mesh_address_t address = {NRF_MESH_ADDRESS_TYPE_INVALID, 0, NULL};
            address.type = NRF_MESH_ADDRESS_TYPE_GROUP;
            if (m_current_node_addr % 0x02)
            {
                address.value  = GROUP_ADDRESS_ODD;
            }
            else
            {
                address.value  = GROUP_ADDRESS_EVEN;
            }
            access_model_id_t model_id;
            model_id.company_id = m_server_model_id.company_id;
            model_id.model_id = m_server_model_id.model_id;
            status = config_client_model_subscription_add(element_address, address, model_id);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        case NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT1:
        {
            config_publication_state_t pubstate = {0};
            client_pub_state_set(&pubstate,
                                 m_current_node_addr + ELEMENT_IDX_ONOFF_CLIENT1,
                                 GROUP_ADDRESS_ODD);
            status = config_client_model_publication_set(&pubstate);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        case NODE_SETUP_CONFIG_PUBLICATION_ONOFF_CLIENT2:
        {
            config_publication_state_t pubstate = {0};
            client_pub_state_set(&pubstate,
                                 m_current_node_addr + ELEMENT_IDX_ONOFF_CLIENT2,
                                 GROUP_ADDRESS_EVEN);
            status = config_client_model_publication_set(&pubstate);

            static const access_status_t exp_status[] = {ACCESS_STATUS_SUCCESS};
            expected_status_set(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, ARRAY_SIZE(exp_status), exp_status);
            break;
        }

        default:
            ERROR_CHECK(NRF_ERROR_NOT_FOUND);
            break;
    }

    if (status != NRF_SUCCESS)
    {
        config_client_pending_msg_cancel();
        if (m_send_timer.count > 0)
        {
            config_step_retry_schedule();
        }
        else
        {
            node_setup_fail();
        }
    }
}

/**
 * This function retrieves the device key for the given address, and configures the tx and rx paths
 * of the config client model.
 */
static void setup_config_client(uint16_t target_addr)
{
    dsm_handle_t        addr_handle = DSM_HANDLE_INVALID;
    dsm_handle_t        devkey_handle = DSM_HANDLE_INVALID;
    nrf_mesh_address_t  addr;

    addr.type  = NRF_MESH_ADDRESS_TYPE_UNICAST;
    addr.value = target_addr;

    /* Provisioner helper has stored address and device keys already. Retrieve them. */
    ERROR_CHECK(dsm_address_handle_get(&addr, &addr_handle));
    ERROR_CHECK(dsm_devkey_handle_get(addr.value, &devkey_handle));

    /* Configure client to communicate with server at the given address */
    ERROR_CHECK(config_client_server_bind(devkey_handle));
    ERROR_CHECK(config_client_server_set(devkey_handle, addr_handle));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Config client setup: devkey_handle:%d addr_handle:%d\n", devkey_handle, addr_handle);
}

static void config_client_msg_handle(const config_client_event_t * p_event,
                                     uint16_t length)
{
    if (*mp_config_step == NODE_SETUP_IDLE || *mp_config_step == NODE_SETUP_DONE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Got unexpected config client message in state %u\n", *mp_config_step);
        return;
    }
    else
    {
        status_check_t status = check_expected_status(p_event->opcode, p_event->p_msg);
        if (status == STATUS_CHECK_PASS)
        {
            mp_config_step++;
            m_send_timer.count = CLIENT_BUSY_SEND_RETRY_LIMIT;

            if (*mp_config_step == NODE_SETUP_DONE)
            {
                node_setup_succeed();
            }
            else
            {
                config_step_execute();
            }
        }
        else if (status == STATUS_CHECK_FAIL)
        {
            node_setup_fail();
        }
    }
}

/*************************************************************************************************/
/* Public functions */

/**
 * Proccess the config client model events, and advances the node setup state machine to the next
 * state, if expected status message is received.
 */
void node_setup_config_client_event_process(config_client_event_type_t event_type,
                                        const config_client_event_t * p_event,
                                        uint16_t length)
{
    if (event_type == CONFIG_CLIENT_EVENT_TYPE_TIMEOUT)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged message status not received \n");

        if (m_retry_count > 0)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Retry ...\n");
            m_retry_count--;
            config_step_execute();
        }
        else
        {
            node_setup_fail();
        }
    }
    else if (event_type == CONFIG_CLIENT_EVENT_TYPE_MSG)
    {
        NRF_MESH_ASSERT_DEBUG(p_event != NULL);
        config_client_msg_handle(p_event, length);
    }
}

/**
 * Begins the node setup process.
 */
void node_setup_start(uint16_t address, uint8_t  retry_cnt, const uint8_t * p_appkey,
                      uint16_t appkey_idx, const char * p_client_uri)
{
    if (*mp_config_step != NODE_SETUP_IDLE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot start. Node setup procedure is in progress.\n");
        return;
    }
    m_current_node_addr = address;
    m_retry_count = retry_cnt;
    m_send_timer.timer.cb = client_send_timer_cb;
    m_send_timer.count = CLIENT_BUSY_SEND_RETRY_LIMIT;
    mp_appkey = p_appkey;
    m_appkey_idx = appkey_idx;

    /* The filter match will decide, which model pairs to pick up */
    if (strcmp(p_client_uri, EX_URI_LS_CLIENT) == 0 || strcmp(p_client_uri, EX_URI_ENOCEAN) == 0)
    {
        m_client_model_id.model_id = GENERIC_ONOFF_CLIENT_MODEL_ID;
        m_server_model_id.model_id = GENERIC_ONOFF_SERVER_MODEL_ID;
    }
    else if (strcmp(p_client_uri, EX_URI_DM_CLIENT) == 0)
    {
        m_client_model_id.model_id = GENERIC_LEVEL_CLIENT_MODEL_ID;
        m_server_model_id.model_id = GENERIC_LEVEL_SERVER_MODEL_ID;
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Invalid client URI identifier.\n");
        NRF_MESH_ASSERT(false);
    }

    m_client_model_id.company_id = ACCESS_COMPANY_ID_NONE;
    m_server_model_id.company_id = ACCESS_COMPANY_ID_NONE;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuring Node: 0x%04X\n", m_current_node_addr);

    setup_config_client(m_current_node_addr);
    setup_select_steps(m_current_node_addr);
    config_step_execute();
}

void node_setup_cb_set(node_setup_successful_cb_t config_success_cb,
                       node_setup_failed_cb_t config_failed_cb)
{
    NRF_MESH_ASSERT(config_success_cb != NULL);
    NRF_MESH_ASSERT(config_failed_cb != NULL);

    m_node_setup_success_cb = config_success_cb;
    m_node_setup_failed_cb = config_failed_cb;
}
