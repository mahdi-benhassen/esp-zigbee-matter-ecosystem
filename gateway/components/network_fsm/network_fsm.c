/**
 * @file network_fsm.c
 * @brief Network FSM Implementation - Dual-SoC (ESP32-S3 + ESP32-H2 RCP)
 *
 * Stage 1: Hardware & Network FSM Architecture
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-dualsoc
 */

#include "network_fsm.h"
#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"

#include "esp_zigbee.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo.h"

/* RCP UART for Dual-SoC */
#include "rcp_uart.h"

/*=============================================================================
 * DEFINITIONS
 *============================================================================*/

#define TAG "NET_FSM"
#define NVS_NAMESPACE       "zb_network"
#define NVS_KEY_CONFIG      "net_config"
#define NVS_KEY_NODES       "node_registry"
#define NVS_KEY_STATS       "net_stats"

#define FSM_TASK_STACK_SIZE     8192
#define FSM_TASK_PRIORITY       5
#define FSM_TASK_NAME           "net_fsm"
#define KEEPALIVE_PERIOD_MS     30000
#define NODE_TIMEOUT_MS         120000
#define RCP_CHECK_INTERVAL_MS   5000
#define RECOVERY_BACKOFF_MS     10000

/*=============================================================================
 * MODULE STATE
 *============================================================================*/

static volatile network_fsm_state_t s_current_state = NET_STATE_INIT;
static QueueHandle_t s_fsm_queue = NULL;
static EventGroupHandle_t s_fsm_event_group = NULL;
static TaskHandle_t s_fsm_task_handle = NULL;
static network_config_t s_config = {0};
static network_stats_t s_stats = {0};
static node_info_t s_node_registry[NETWORK_MAX_CHILDREN] = {0};
static uint8_t s_node_count = 0;
static SemaphoreHandle_t s_registry_mutex = NULL;
static esp_timer_handle_t s_keepalive_timer = NULL;
static uint8_t s_retry_count = 0;
static uint8_t s_recovery_attempts = 0;
static uint32_t s_net_formed_at_ms = 0;

/*=============================================================================
 * FORWARD DECLARATIONS
 *============================================================================*/

static esp_err_t action_init(const fsm_event_msg_t *event);
static esp_err_t action_hw_init(const fsm_event_msg_t *event);
static esp_err_t action_hw_init_failed(const fsm_event_msg_t *event);
static esp_err_t action_rcp_init(const fsm_event_msg_t *event);
static esp_err_t action_rcp_ready(const fsm_event_msg_t *event);
static esp_err_t action_rcp_error(const fsm_event_msg_t *event);
static esp_err_t action_rcp_recovered(const fsm_event_msg_t *event);
static esp_err_t action_zb_init(const fsm_event_msg_t *event);
static esp_err_t action_zb_init_failed(const fsm_event_msg_t *event);
static esp_err_t action_net_formation(const fsm_event_msg_t *event);
static esp_err_t action_net_form_success(const fsm_event_msg_t *event);
static esp_err_t action_net_form_failed(const fsm_event_msg_t *event);
static esp_err_t action_node_join_request(const fsm_event_msg_t *event);
static esp_err_t action_node_join_accept(const fsm_event_msg_t *event);
static esp_err_t action_node_join_reject(const fsm_event_msg_t *event);
static esp_err_t action_node_join_timeout(const fsm_event_msg_t *event);
static esp_err_t action_node_leave(const fsm_event_msg_t *event);
static esp_err_t action_node_rejoin_request(const fsm_event_msg_t *event);
static esp_err_t action_node_rejoin_success(const fsm_event_msg_t *event);
static esp_err_t action_node_rejoin_failed(const fsm_event_msg_t *event);
static esp_err_t action_net_link_fail(const fsm_event_msg_t *event);
static esp_err_t action_net_recover_start(const fsm_event_msg_t *event);
static esp_err_t action_net_recover_done(const fsm_event_msg_t *event);
static esp_err_t action_net_recover_failed(const fsm_event_msg_t *event);
static esp_err_t action_cmd_permit_join(const fsm_event_msg_t *event);
static esp_err_t action_cmd_deny_join(const fsm_event_msg_t *event);
static esp_err_t action_cmd_reset_network(const fsm_event_msg_t *event);
static esp_err_t action_cmd_shutdown(const fsm_event_msg_t *event);
static esp_err_t action_keepalive_timer(const fsm_event_msg_t *event);
static esp_err_t action_net_channel_conflict(const fsm_event_msg_t *event);

static void zb_zdo_signal_handler(esp_zb_app_signal_t *signal_struct);

/*=============================================================================
 * STATE TRANSITION TABLE - DUAL-SOC
 *============================================================================*/

static const state_transition_t s_transition_table[] = {
    /* INIT -> HW_INIT */
    {NET_STATE_INIT, EVENT_HW_INIT_DONE,
     NET_STATE_HW_INIT, action_hw_init, "INIT->HW_INIT"},

    /* HW_INIT -> RCP_INIT (Dual-SoC: init RCP UART) */
    {NET_STATE_HW_INIT, EVENT_HW_INIT_DONE,
     NET_STATE_RCP_INIT, action_rcp_init, "HW_INIT->RCP_INIT"},
    {NET_STATE_HW_INIT, EVENT_HW_INIT_FAILED,
     NET_STATE_NET_FAILED, action_hw_init_failed, "HW_INIT->FAILED"},

    /* RCP_INIT -> RCP_READY */
    {NET_STATE_RCP_INIT, EVENT_RCP_INIT_DONE,
     NET_STATE_RCP_READY, action_rcp_ready, "RCP_INIT->RCP_READY"},
    {NET_STATE_RCP_INIT, EVENT_RCP_ERROR,
     NET_STATE_NET_RECOVER, action_rcp_error, "RCP_INIT->RECOVER"},

    /* RCP_READY -> ZB_INIT */
    {NET_STATE_RCP_READY, EVENT_RCP_READY,
     NET_STATE_ZB_INIT, action_zb_init, "RCP_READY->ZB_INIT"},
    {NET_STATE_RCP_READY, EVENT_RCP_ERROR,
     NET_STATE_NET_RECOVER, action_rcp_error, "RCP_READY->RECOVER"},

    /* ZB_INIT -> NET_FORMATION */
    {NET_STATE_ZB_INIT, EVENT_ZB_INIT_DONE,
     NET_STATE_NET_FORMATION, action_net_formation, "ZB_INIT->FORMATION"},
    {NET_STATE_ZB_INIT, EVENT_ZB_INIT_FAILED,
     NET_STATE_NET_FAILED, action_zb_init_failed, "ZB_INIT->FAILED"},

    /* RCP recovery during any state */
    {NET_STATE_ZB_INIT, EVENT_RCP_RECOVERED,
     NET_STATE_RCP_READY, action_rcp_recovered, "ZB_INIT->RCP_READY(recv)"},
    {NET_STATE_NET_FORMATION, EVENT_RCP_RECOVERED,
     NET_STATE_RCP_READY, action_rcp_recovered, "FORMATION->RCP_READY(recv)"},
    {NET_STATE_NET_READY, EVENT_RCP_RECOVERED,
     NET_STATE_RCP_READY, action_rcp_recovered, "READY->RCP_READY(recv)"},
    {NET_STATE_NET_OPERATIONAL, EVENT_RCP_RECOVERED,
     NET_STATE_RCP_READY, action_rcp_recovered, "OP->RCP_READY(recv)"},

    /* NET_FORMATION -> NET_READY */
    {NET_STATE_NET_FORMATION, EVENT_NET_FORM_SUCCESS,
     NET_STATE_NET_READY, action_net_form_success, "FORMATION->READY"},
    {NET_STATE_NET_FORMATION, EVENT_NET_FORM_FAILED,
     NET_STATE_NET_FAILED, action_net_form_failed, "FORMATION->FAILED"},
    {NET_STATE_NET_FORMATION, EVENT_NET_CHANNEL_CONFLICT,
     NET_STATE_NET_FORMATION, action_net_channel_conflict, "FORMATION->FORMATION(ch)"},

    /* NET_READY transitions */
    {NET_STATE_NET_READY, EVENT_NODE_JOIN_REQUEST,
     NET_STATE_NODE_JOINING, action_node_join_request, "READY->JOINING"},
    {NET_STATE_NET_READY, EVENT_CMD_PERMIT_JOIN,
     NET_STATE_NET_READY, action_cmd_permit_join, "READY->READY(permit)"},
    {NET_STATE_NET_READY, EVENT_CMD_SHUTDOWN,
     NET_STATE_NET_SHUTDOWN, action_cmd_shutdown, "READY->SHUTDOWN"},
    {NET_STATE_NET_READY, EVENT_NET_LINK_FAIL,
     NET_STATE_NET_RECOVER, action_net_link_fail, "READY->RECOVER"},
    {NET_STATE_NET_READY, EVENT_TIMER_KEEPALIVE,
     NET_STATE_NET_READY, action_keepalive_timer, "READY->READY(kal)"},
    {NET_STATE_NET_READY, EVENT_NODE_DATA_RECEIVED,
     NET_STATE_NET_OPERATIONAL, action_keepalive_timer, "READY->OPERATIONAL"},

    /* NODE_JOINING -> NODE_OPERATIONAL / NET_READY */
    {NET_STATE_NODE_JOINING, EVENT_NODE_JOIN_ACCEPT,
     NET_STATE_NODE_OPERATIONAL, action_node_join_accept, "JOINING->OPERATIONAL"},
    {NET_STATE_NODE_JOINING, EVENT_NODE_JOIN_REJECT,
     NET_STATE_NET_READY, action_node_join_reject, "JOINING->READY(rej)"},
    {NET_STATE_NODE_JOINING, EVENT_NODE_JOIN_TIMEOUT,
     NET_STATE_NET_READY, action_node_join_timeout, "JOINING->READY(tmo)"},
    {NET_STATE_NODE_JOINING, EVENT_NODE_LEAVE_NOTIFY,
     NET_STATE_NET_READY, action_node_leave, "JOINING->READY(left)"},

    /* NODE_OPERATIONAL -> ... */
    {NET_STATE_NODE_OPERATIONAL, EVENT_NODE_DATA_RECEIVED,
     NET_STATE_NET_OPERATIONAL, action_keepalive_timer, "NODE_OP->NET_OP"},
    {NET_STATE_NODE_OPERATIONAL, EVENT_NODE_LEAVE_REQUEST,
     NET_STATE_NODE_LEAVING, action_node_leave, "NODE_OP->LEAVING"},
    {NET_STATE_NODE_OPERATIONAL, EVENT_NODE_LEAVE_NOTIFY,
     NET_STATE_NODE_LEAVING, action_node_leave, "NODE_OP->LEAVING(not)"},
    {NET_STATE_NODE_OPERATIONAL, EVENT_NODE_PING_TIMEOUT,
     NET_STATE_NODE_REJOIN, action_node_rejoin_request, "NODE_OP->REJOIN"},
    {NET_STATE_NODE_OPERATIONAL, EVENT_NODE_REJOIN_REQUEST,
     NET_STATE_NODE_REJOIN, action_node_rejoin_request, "NODE_OP->REJOIN(req)"},
    {NET_STATE_NODE_OPERATIONAL, EVENT_TIMER_KEEPALIVE,
     NET_STATE_NODE_OPERATIONAL, action_keepalive_timer, "NODE_OP->NODE_OP"},

    /* NET_OPERATIONAL -> ... */
    {NET_STATE_NET_OPERATIONAL, EVENT_NODE_JOIN_REQUEST,
     NET_STATE_NODE_JOINING, action_node_join_request, "NET_OP->JOINING"},
    {NET_STATE_NET_OPERATIONAL, EVENT_NODE_LEAVE_REQUEST,
     NET_STATE_NODE_LEAVING, action_node_leave, "NET_OP->LEAVING"},
    {NET_STATE_NET_OPERATIONAL, EVENT_NODE_LEAVE_NOTIFY,
     NET_STATE_NODE_LEAVING, action_node_leave, "NET_OP->LEAVING(not)"},
    {NET_STATE_NET_OPERATIONAL, EVENT_NODE_REJOIN_REQUEST,
     NET_STATE_NODE_REJOIN, action_node_rejoin_request, "NET_OP->REJOIN"},
    {NET_STATE_NET_OPERATIONAL, EVENT_NODE_DATA_RECEIVED,
     NET_STATE_NET_OPERATIONAL, action_keepalive_timer, "NET_OP->NET_OP"},
    {NET_STATE_NET_OPERATIONAL, EVENT_NODE_PING_TIMEOUT,
     NET_STATE_NODE_REJOIN, action_node_rejoin_request, "NET_OP->REJOIN"},
    {NET_STATE_NET_OPERATIONAL, EVENT_NET_LINK_FAIL,
     NET_STATE_NET_RECOVER, action_net_link_fail, "NET_OP->RECOVER"},
    {NET_STATE_NET_OPERATIONAL, EVENT_CMD_SHUTDOWN,
     NET_STATE_NET_SHUTDOWN, action_cmd_shutdown, "NET_OP->SHUTDOWN"},
    {NET_STATE_NET_OPERATIONAL, EVENT_CMD_PERMIT_JOIN,
     NET_STATE_NET_OPERATIONAL, action_cmd_permit_join, "NET_OP->NET_OP(pj)"},
    {NET_STATE_NET_OPERATIONAL, EVENT_CMD_DENY_JOIN,
     NET_STATE_NET_OPERATIONAL, action_cmd_deny_join, "NET_OP->NET_OP(dj)"},
    {NET_STATE_NET_OPERATIONAL, EVENT_CMD_RESET_NETWORK,
     NET_STATE_NET_FORMATION, action_cmd_reset_network, "NET_OP->FORMATION"},
    {NET_STATE_NET_OPERATIONAL, EVENT_TIMER_KEEPALIVE,
     NET_STATE_NET_OPERATIONAL, action_keepalive_timer, "NET_OP->NET_OP(kal)"},

    /* NODE_REJOIN -> NET_OPERATIONAL / NET_READY */
    {NET_STATE_NODE_REJOIN, EVENT_NODE_REJOIN_SUCCESS,
     NET_STATE_NET_OPERATIONAL, action_node_rejoin_success, "REJOIN->NET_OP"},
    {NET_STATE_NODE_REJOIN, EVENT_NODE_REJOIN_FAILED,
     NET_STATE_NET_OPERATIONAL, action_node_rejoin_failed, "REJOIN->NET_OP(fail)"},
    {NET_STATE_NODE_REJOIN, EVENT_TIMER_REJOIN_BACKOFF,
     NET_STATE_NODE_REJOIN, action_node_rejoin_request, "REJOIN->REJOIN"},
    {NET_STATE_NODE_REJOIN, EVENT_NODE_JOIN_TIMEOUT,
     NET_STATE_NET_READY, action_node_join_timeout, "REJOIN->READY"},

    /* NODE_LEAVING -> NET_OPERATIONAL */
    {NET_STATE_NODE_LEAVING, EVENT_NODE_LEAVE_NOTIFY,
     NET_STATE_NET_OPERATIONAL, action_node_leave, "LEAVING->NET_OP"},
    {NET_STATE_NODE_LEAVING, EVENT_NODE_LEAVE_FORCE,
     NET_STATE_NET_OPERATIONAL, action_node_leave, "LEAVING->NET_OP(force)"},
    {NET_STATE_NODE_LEAVING, EVENT_NODE_JOIN_TIMEOUT,
     NET_STATE_NET_OPERATIONAL, action_node_join_timeout, "LEAVING->NET_OP"},

    /* NET_RECOVER -> NET_FORMATION / NET_FAILED */
    {NET_STATE_NET_RECOVER, EVENT_NET_RECOVER_DONE,
     NET_STATE_RCP_INIT, action_net_recover_done, "RECOVER->RCP_INIT"},
    {NET_STATE_NET_RECOVER, EVENT_NET_RECOVER_FAILED,
     NET_STATE_NET_FAILED, action_net_recover_failed, "RECOVER->FAILED"},
    {NET_STATE_NET_RECOVER, EVENT_NET_RECOVER_START,
     NET_STATE_NET_RECOVER, action_net_recover_start, "RECOVER->RECOVER"},
    {NET_STATE_NET_RECOVER, EVENT_RCP_RECOVERED,
     NET_STATE_RCP_INIT, action_rcp_recovered, "RECOVER->RCP_INIT"},

    /* NET_FAILED -> NET_RECOVER / NET_SHUTDOWN */
    {NET_STATE_NET_FAILED, EVENT_NET_RECOVER_START,
     NET_STATE_NET_RECOVER, action_net_recover_start, "FAILED->RECOVER"},
    {NET_STATE_NET_FAILED, EVENT_CMD_SHUTDOWN,
     NET_STATE_NET_SHUTDOWN, action_cmd_shutdown, "FAILED->SHUTDOWN"},
    {NET_STATE_NET_FAILED, EVENT_CMD_RESET_NETWORK,
     NET_STATE_RCP_INIT, action_cmd_reset_network, "FAILED->RCP_INIT"},
};

#define TRANSITION_TABLE_SIZE (sizeof(s_transition_table) / sizeof(s_transition_table[0]))

/*=============================================================================
 * NAME MAPPINGS
 *============================================================================*/

static const char * const s_state_names[] = {
    [NET_STATE_INIT] = "INIT",
    [NET_STATE_HW_INIT] = "HW_INIT",
    [NET_STATE_RCP_INIT] = "RCP_INIT",
    [NET_STATE_RCP_READY] = "RCP_READY",
    [NET_STATE_ZB_INIT] = "ZB_INIT",
    [NET_STATE_NET_FORMATION] = "NET_FORMATION",
    [NET_STATE_NET_READY] = "NET_READY",
    [NET_STATE_NODE_JOINING] = "NODE_JOINING",
    [NET_STATE_NODE_OPERATIONAL] = "NODE_OPERATIONAL",
    [NET_STATE_NODE_REJOIN] = "NODE_REJOIN",
    [NET_STATE_NODE_LEAVING] = "NODE_LEAVING",
    [NET_STATE_NET_OPERATIONAL] = "NET_OPERATIONAL",
    [NET_STATE_NET_RECOVER] = "NET_RECOVER",
    [NET_STATE_NET_FAILED] = "NET_FAILED",
    [NET_STATE_NET_SHUTDOWN] = "NET_SHUTDOWN",
    [NET_STATE_MAX] = "UNKNOWN"
};

static const char * const s_event_names[] = {
    [EVENT_HW_INIT_DONE] = "HW_INIT_DONE",
    [EVENT_HW_INIT_FAILED] = "HW_INIT_FAILED",
    [EVENT_RCP_INIT_DONE] = "RCP_INIT_DONE",
    [EVENT_RCP_READY] = "RCP_READY",
    [EVENT_RCP_ERROR] = "RCP_ERROR",
    [EVENT_RCP_RECOVERED] = "RCP_RECOVERED",
    [EVENT_ZB_INIT_DONE] = "ZB_INIT_DONE",
    [EVENT_ZB_INIT_FAILED] = "ZB_INIT_FAILED",
    [EVENT_ZB_START_DONE] = "ZB_START_DONE",
    [EVENT_ZB_START_FAILED] = "ZB_START_FAILED",
    [EVENT_NET_FORM_SUCCESS] = "NET_FORM_SUCCESS",
    [EVENT_NET_FORM_FAILED] = "NET_FORM_FAILED",
    [EVENT_NET_CHANNEL_CONFLICT] = "NET_CHANNEL_CONFLICT",
    [EVENT_NODE_JOIN_REQUEST] = "NODE_JOIN_REQUEST",
    [EVENT_NODE_JOIN_ACCEPT] = "NODE_JOIN_ACCEPT",
    [EVENT_NODE_JOIN_REJECT] = "NODE_JOIN_REJECT",
    [EVENT_NODE_JOIN_TIMEOUT] = "NODE_JOIN_TIMEOUT",
    [EVENT_NODE_LEAVE_REQUEST] = "NODE_LEAVE_REQUEST",
    [EVENT_NODE_LEAVE_NOTIFY] = "NODE_LEAVE_NOTIFY",
    [EVENT_NODE_LEAVE_FORCE] = "NODE_LEAVE_FORCE",
    [EVENT_NODE_REJOIN_REQUEST] = "NODE_REJOIN_REQUEST",
    [EVENT_NODE_REJOIN_SUCCESS] = "NODE_REJOIN_SUCCESS",
    [EVENT_NODE_REJOIN_FAILED] = "NODE_REJOIN_FAILED",
    [EVENT_NODE_DATA_RECEIVED] = "NODE_DATA_RECEIVED",
    [EVENT_NODE_PING_TIMEOUT] = "NODE_PING_TIMEOUT",
    [EVENT_NET_LINK_FAIL] = "NET_LINK_FAIL",
    [EVENT_NET_RECOVER_START] = "NET_RECOVER_START",
    [EVENT_NET_RECOVER_DONE] = "NET_RECOVER_DONE",
    [EVENT_NET_RECOVER_FAILED] = "NET_RECOVER_FAILED",
    [EVENT_CMD_PERMIT_JOIN] = "CMD_PERMIT_JOIN",
    [EVENT_CMD_DENY_JOIN] = "CMD_DENY_JOIN",
    [EVENT_CMD_RESET_NETWORK] = "CMD_RESET_NETWORK",
    [EVENT_CMD_SHUTDOWN] = "CMD_SHUTDOWN",
    [EVENT_TIMER_KEEPALIVE] = "TIMER_KEEPALIVE",
    [EVENT_TIMER_REJOIN_BACKOFF] = "TIMER_REJOIN_BACKOFF",
    [EVENT_MAX] = "UNKNOWN"
};

/*=============================================================================
 * HELPERS
 *============================================================================*/

static const state_transition_t *find_transition(network_fsm_state_t state,
                                                   network_fsm_event_t event)
{
    for (size_t i = 0; i < TRANSITION_TABLE_SIZE; i++) {
        if (s_transition_table[i].current_state == state &&
            s_transition_table[i].event == event) {
            return &s_transition_table[i];
        }
    }
    return NULL;
}

static inline uint32_t get_timestamp_ms(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void print_transition(const state_transition_t *trans) {
    if (trans) {
        ESP_LOGI(TAG, "Transition: [%s] + %s -> [%s] (%s)",
                 s_state_names[trans->current_state],
                 s_event_names[trans->event],
                 s_state_names[trans->next_state],
                 trans->name);
    }
}

/*=============================================================================
 * ACTION IMPLEMENTATIONS
 *============================================================================*/

static esp_err_t action_init(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "=== Starting Network FSM (Dual-SoC) ===");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS needs erase");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = network_config_load(&s_config);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Using defaults");
        s_config.channel = CONFIG_ZB_CHANNEL;
        s_config.pan_id = CONFIG_ZB_PANID;
        memcpy(s_config.ext_pan_id, CONFIG_ZB_EXTPANID, 8);
        s_config.channel_mask = (1U << CONFIG_ZB_CHANNEL);
        s_config.permit_join = false;
        s_config.permit_join_duration = 0;
        s_config.require_link_key = true;
        memset(s_config.network_key, 0xAA, 16);
        memset(s_config.trust_center_address, 0, 8);
    }

    ESP_ERROR_CHECK(node_registry_init());
    ESP_LOGI(TAG, "Registry: %d nodes loaded", s_node_count);
    ESP_LOGI(TAG, "Hardware: ESP32-S3 + ESP32-H2 (UART RCP)");

    fsm_event_msg_t evt = {
        .event = EVENT_HW_INIT_DONE,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    return ESP_OK;
}

static esp_err_t action_hw_init(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "HW_INIT: ESP32-S3 peripherals ready");
    /* ESP32-S3 doesn't need special peripheral init - Wi-Fi done separately */
    fsm_event_msg_t evt = {
        .event = EVENT_HW_INIT_DONE,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    return ESP_OK;
}

static esp_err_t action_hw_init_failed(const fsm_event_msg_t *event) {
    ESP_LOGE(TAG, "HW init failed! error=%lu", event->payload.hw.error_code);
    s_recovery_attempts++;
    return ESP_OK;
}

/**
 * @brief Initialize RCP UART - ESP32-H2 radio co-processor
 */
static esp_err_t action_rcp_init(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "RCP_INIT: Initializing ESP32-H2 RCP UART...");

    esp_err_t err = rcp_uart_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "RCP UART init failed: %s", esp_err_to_name(err));
        fsm_event_msg_t evt = {
            .event = EVENT_RCP_ERROR,
            .payload.rcp.rcp_error = (uint32_t)err,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
        return err;
    }

    ESP_LOGI(TAG, "RCP UART initialized, checking RCP status...");
    rcp_uart_print_diagnostics();

    if (rcp_uart_is_operational()) {
        fsm_event_msg_t evt = {
            .event = EVENT_RCP_READY,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
    } else {
        ESP_LOGW(TAG, "RCP not ready after init, attempting recovery...");
        fsm_event_msg_t evt = {
            .event = EVENT_RCP_ERROR,
            .payload.rcp.rcp_error = 1,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
    }

    return ESP_OK;
}

static esp_err_t action_rcp_ready(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "RCP_READY: ESP32-H2 RCP is operational");
    fsm_event_msg_t evt = {
        .event = EVENT_RCP_READY,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    return ESP_OK;
}

static esp_err_t action_rcp_error(const fsm_event_msg_t *event) {
    ESP_LOGE(TAG, "RCP_ERROR: Radio Co-Processor error! rcp_error=%lu",
             event->payload.rcp.rcp_error);
    s_recovery_attempts++;
    ESP_LOGI(TAG, "Will attempt RCP recovery in recovery state");
    return ESP_OK;
}

static esp_err_t action_rcp_recovered(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "RCP_RECOVERED: RCP is back online");
    s_recovery_attempts = 0;

    /* Now proceed to Zigbee init */
    fsm_event_msg_t evt = {
        .event = EVENT_RCP_READY,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    return ESP_OK;
}

static esp_err_t action_zb_init(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "ZB_INIT: Initializing Zigbee Coordinator (via RCP)...");

    /* Platform config for RCP mode */
    esp_zb_platform_config_t platform_config = {
        .radio_config = {
            .radio_mode = RADIO_MODE_UART_RCP,
        },
        .host_config = {
            .host_connection_mode = HOST_CONNECTION_MODE_RCP,
        },
    };

    ESP_ERROR_CHECK(esp_zb_platform_config(&platform_config));

    esp_zb_cfg_t zb_config = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_COORDINATOR,
        .install_code_policy = s_config.require_link_key,
        .nwk_cfg = {
            .zczr_cfg = {
                .max_children = NETWORK_MAX_CHILDREN,
            }
        }
    };

    esp_zb_init(&zb_config);

    esp_zb_extended_pan_id_t ext_pan_id;
    memcpy(ext_pan_id.pan_id, s_config.ext_pan_id, sizeof(esp_zb_pan_id_t));
    esp_zb_set_extended_pan_id(ext_pan_id);

    esp_zb_pan_id_t pan_id = {.pan_id = s_config.pan_id};
    esp_zb_set_pan_id(pan_id);

    uint8_t channel_list[] = {s_config.channel};
    esp_zb_set_channel_config((esp_zb_channel_config_t){
        .channel_list = channel_list,
        .channel_num = 1
    });

    if (s_config.require_link_key) {
        esp_zb_secur_network_key_set(s_config.network_key);
    }

    esp_zb_app_signal_handler_register(zb_zdo_signal_handler);

    ESP_LOGI(TAG, "Coordinator config: Ch=%d, PAN=0x%04X, MaxChildren=%d",
             s_config.channel, s_config.pan_id, NETWORK_MAX_CHILDREN);

    fsm_event_msg_t evt = {
        .event = EVENT_ZB_START_DONE,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    return ESP_OK;
}

static esp_err_t action_zb_init_failed(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGE(TAG, "Zigbee init failed!");
    return ESP_OK;
}

static esp_err_t action_net_formation(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "NET_FORMATION: Starting Zigbee network formation...");
    ESP_ERROR_CHECK(esp_zb_start());
    ESP_LOGI(TAG, "Zigbee stack started via RCP");
    s_retry_count = 0;
    return ESP_OK;
}

static esp_err_t action_net_form_success(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  Zigbee NETWORK FORMED (via ESP32-H2 RCP)");
    ESP_LOGI(TAG, "============================================");
    s_net_formed_at_ms = get_timestamp_ms();
    s_retry_count = 0;
    s_recovery_attempts = 0;
    xEventGroupSetBits(s_fsm_event_group, NETWORK_EVENT_NET_READY);
    return ESP_OK;
}

static esp_err_t action_net_form_failed(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGE(TAG, "Network formation failed!");
    s_retry_count++;
    if (s_retry_count < NETWORK_MAX_RETRY_ATTEMPTS) {
        uint8_t new_ch = s_config.channel + s_retry_count;
        if (new_ch > 26) new_ch = 11 + (new_ch - 26);
        s_config.channel = new_ch;
        ESP_LOGI(TAG, "Retry %d/%d on channel %d", s_retry_count + 1,
                 NETWORK_MAX_RETRY_ATTEMPTS, new_ch);
        fsm_event_msg_t evt = {
            .event = EVENT_ZB_INIT_DONE,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
    } else {
        ESP_LOGE(TAG, "Failed after %d attempts", NETWORK_MAX_RETRY_ATTEMPTS);
        xEventGroupSetBits(s_fsm_event_group, NETWORK_EVENT_NET_FAILED);
    }
    return ESP_OK;
}

static esp_err_t action_net_channel_conflict(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGW(TAG, "Channel conflict, scanning...");
    uint8_t new_ch = s_config.channel + 1;
    if (new_ch > 26) new_ch = 11;
    s_config.channel = new_ch;
    ESP_LOGI(TAG, "Switching to channel %d", new_ch);
    return action_net_formation(event);
}

static esp_err_t action_node_join_request(const fsm_event_msg_t *event) {
    ESP_LOGI(TAG, "Join request: addr=0x%04X, RSSI=%d, LQI=%u",
             event->payload.node.short_addr, event->payload.node.rssi,
             event->payload.node.lqi);
    if (!s_config.permit_join) {
        ESP_LOGW(TAG, "Join rejected: permit disabled");
        fsm_event_msg_t evt = {
            .event = EVENT_NODE_JOIN_REJECT, .payload = event->payload,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
        return ESP_OK;
    }
    if (s_node_count >= NETWORK_MAX_CHILDREN) {
        ESP_LOGW(TAG, "Join rejected: registry full (%d/%d)",
                 s_node_count, NETWORK_MAX_CHILDREN);
        fsm_event_msg_t evt = {
            .event = EVENT_NODE_JOIN_REJECT, .payload = event->payload,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
        return ESP_OK;
    }
    ESP_LOGI(TAG, "Join accepted, awaiting device announce...");
    return ESP_OK;
}

static esp_err_t action_node_join_accept(const fsm_event_msg_t *event) {
    node_info_t new_node = {0};
    memcpy(new_node.eui64, event->payload.node.eui64, sizeof(zb_eui64_t));
    new_node.short_addr = event->payload.node.short_addr;
    new_node.is_online = true;
    new_node.is_sleepy = false;
    new_node.last_rssi = event->payload.node.rssi;
    new_node.last_lqi = event->payload.node.lqi;
    new_node.joined_at = get_timestamp_ms();
    new_node.last_seen = get_timestamp_ms();
    new_node.rejoin_count = 0;
    new_node.rejoin_backoff_ms = NODE_REJOIN_BACKOFF_BASE_MS;

    esp_err_t err = node_registry_add(&new_node);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Registry add failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Node 0x%04X added (%d/%d)",
             event->payload.node.short_addr, s_node_count, NETWORK_MAX_CHILDREN);
    s_stats.nodes_joined_total++;
    xEventGroupSetBits(s_fsm_event_group, NETWORK_EVENT_NODE_JOINED);

    if (s_node_count == 1) {
        fsm_event_msg_t evt = {
            .event = EVENT_NODE_DATA_RECEIVED,
            .payload = event->payload,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
    }
    return ESP_OK;
}

static esp_err_t action_node_join_reject(const fsm_event_msg_t *event) {
    ESP_LOGW(TAG, "Node 0x%04X join rejected", event->payload.node.short_addr);
    s_stats.join_failures++;
    esp_zb_zdo_mgmt_leave_req_t leave_req = {0};
    memcpy(leave_req.device_address, event->payload.node.eui64, sizeof(esp_zb_ieee_addr_t));
    leave_req.rejoin = 0;
    leave_req.remove_children = 0;
    esp_zb_zdo_device_leave_req(&leave_req, NULL, NULL);
    return ESP_OK;
}

static esp_err_t action_node_join_timeout(const fsm_event_msg_t *event) {
    ESP_LOGW(TAG, "Node join timeout");
    (void)event;
    return ESP_OK;
}

static esp_err_t action_node_leave(const fsm_event_msg_t *event) {
    ESP_LOGI(TAG, "Node 0x%04X leaving", event->payload.node.short_addr);
    esp_err_t err = node_registry_remove(event->payload.node.eui64);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Node removed (%d remaining)", s_node_count);
        s_stats.nodes_left_total++;
    }
    xEventGroupSetBits(s_fsm_event_group, NETWORK_EVENT_NODE_LEFT);
    return ESP_OK;
}

static esp_err_t action_node_rejoin_request(const fsm_event_msg_t *event) {
    ESP_LOGI(TAG, "Node 0x%04X rejoining", event->payload.node.short_addr);
    node_info_t node;
    esp_err_t err = node_registry_find_by_short_addr(event->payload.node.short_addr, &node);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unknown node rejoining, treat as new");
        fsm_event_msg_t evt = {
            .event = EVENT_NODE_JOIN_REQUEST,
            .payload = event->payload,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
        return ESP_OK;
    }
    node.rejoin_count++;
    node.last_seen = get_timestamp_ms();
    node.last_rssi = event->payload.node.rssi;
    node.last_lqi = event->payload.node.lqi;
    node.rejoin_backoff_ms *= 2;
    if (node.rejoin_backoff_ms > NODE_REJOIN_MAX_BACKOFF_MS)
        node.rejoin_backoff_ms = NODE_REJOIN_MAX_BACKOFF_MS;
    node_registry_add(&node);
    ESP_LOGI(TAG, "Rejoin attempt %d, backoff=%lums",
             node.rejoin_count, node.rejoin_backoff_ms);
    if (node.rejoin_count > NETWORK_MAX_RETRY_ATTEMPTS) {
        ESP_LOGW(TAG, "Max rejoin attempts exceeded, forcing leave");
        fsm_event_msg_t evt = {
            .event = EVENT_NODE_LEAVE_FORCE,
            .payload = event->payload,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
        return ESP_OK;
    }
    fsm_event_msg_t evt = {
        .event = EVENT_NODE_REJOIN_SUCCESS,
        .payload = event->payload,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    s_stats.rejoins_total++;
    return ESP_OK;
}

static esp_err_t action_node_rejoin_success(const fsm_event_msg_t *event) {
    ESP_LOGI(TAG, "Node 0x%04X rejoined", event->payload.node.short_addr);
    node_registry_set_online(event->payload.node.eui64, true);
    node_registry_update_status(event->payload.node.eui64,
                                 event->payload.node.rssi, event->payload.node.lqi);
    return ESP_OK;
}

static esp_err_t action_node_rejoin_failed(const fsm_event_msg_t *event) {
    ESP_LOGW(TAG, "Node 0x%04X rejoin failed", event->payload.node.short_addr);
    node_registry_set_online(event->payload.node.eui64, false);
    return ESP_OK;
}

static esp_err_t action_net_link_fail(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGW(TAG, "Network link failure!");
    fsm_event_msg_t evt = {
        .event = EVENT_NET_RECOVER_START,
        .timestamp_ms = get_timestamp_ms()
    };
    xQueueSend(s_fsm_queue, &evt, 0);
    return ESP_OK;
}

static esp_err_t action_net_recover_start(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "Recovery attempt %d...", s_recovery_attempts + 1);
    s_recovery_attempts++;
    for (int i = 0; i < s_node_count; i++) s_node_registry[i].is_online = false;
    esp_zb_stop();
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (s_recovery_attempts < NETWORK_MAX_RETRY_ATTEMPTS) {
        /* Try RCP reset first */
        ESP_LOGI(TAG, "Resetting RCP...");
        rcp_uart_reset();
        fsm_event_msg_t evt = {
            .event = EVENT_NET_RECOVER_DONE,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
    } else {
        ESP_LOGE(TAG, "Recovery failed after %d attempts", NETWORK_MAX_RETRY_ATTEMPTS);
        fsm_event_msg_t evt = {
            .event = EVENT_NET_RECOVER_FAILED,
            .timestamp_ms = get_timestamp_ms()
        };
        xQueueSend(s_fsm_queue, &evt, 0);
    }
    s_stats.last_recovery_at = get_timestamp_ms();
    return ESP_OK;
}

static esp_err_t action_net_recover_done(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "Recovery done, restarting...");
    return ESP_OK;
}

static esp_err_t action_net_recover_failed(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGE(TAG, "Recovery failed!");
    xEventGroupSetBits(s_fsm_event_group, NETWORK_EVENT_NET_FAILED);
    return ESP_OK;
}

static esp_err_t action_cmd_permit_join(const fsm_event_msg_t *event) {
    uint16_t duration_sec = event->payload.cmd.param & 0xFFFF;
    if (duration_sec == 0xFF) {
        ESP_LOGI(TAG, "Permit join: permanent");
        s_config.permit_join = true;
        s_config.permit_join_duration = 0xFFFF;
    } else if (duration_sec == 0) {
        ESP_LOGI(TAG, "Permit join: disabled");
        s_config.permit_join = false;
        s_config.permit_join_duration = 0;
    } else {
        ESP_LOGI(TAG, "Permit join: %us", duration_sec);
        s_config.permit_join = true;
        s_config.permit_join_duration = duration_sec;
    }
    esp_zb_secur_network_min_join_lqi_set(0);
    esp_zb_bdb_open_network(duration_sec);
    network_config_save(&s_config);
    return ESP_OK;
}

static esp_err_t action_cmd_deny_join(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "Deny join");
    s_config.permit_join = false;
    s_config.permit_join_duration = 0;
    esp_zb_bdb_close_network();
    network_config_save(&s_config);
    return ESP_OK;
}

static esp_err_t action_cmd_reset_network(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGW(TAG, "NETWORK RESET!");
    esp_zb_stop();
    memset(s_node_registry, 0, sizeof(s_node_registry));
    s_node_count = 0;
    memset(&s_stats, 0, sizeof(s_stats));
    network_config_reset();
    node_registry_persist();
    network_config_load(&s_config);
    return ESP_OK;
}

static esp_err_t action_cmd_shutdown(const fsm_event_msg_t *event) {
    (void)event;
    ESP_LOGI(TAG, "Shutting down...");
    if (s_keepalive_timer) esp_timer_stop(s_keepalive_timer);
    esp_zb_stop();
    node_registry_persist();
    network_config_save(&s_config);
    rcp_uart_deinit();
    ESP_LOGI(TAG, "Shutdown complete");
    return ESP_OK;
}

static esp_err_t action_keepalive_timer(const fsm_event_msg_t *event) {
    (void)event;
    uint32_t now = get_timestamp_ms();
    if (s_net_formed_at_ms > 0)
        s_stats.network_uptime_sec = (now - s_net_formed_at_ms) / 1000;

    /* Check RCP health periodically */
    static uint32_t last_rcp_check = 0;
    if (now - last_rcp_check > RCP_CHECK_INTERVAL_MS) {
        last_rcp_check = now;
        if (!rcp_uart_is_operational()) {
            ESP_LOGW(TAG, "RCP not operational! Triggering recovery...");
            fsm_event_msg_t evt = {
                .event = EVENT_NET_LINK_FAIL,
                .timestamp_ms = now
            };
            xQueueSend(s_fsm_queue, &evt, 0);
            return ESP_OK;
        }
    }

    /* Check node timeouts */
    for (int i = 0; i < s_node_count; i++) {
        node_info_t *node = &s_node_registry[i];
        if (!node->is_sleepy && node->is_online) {
            uint32_t idle = now - node->last_seen;
            if (idle > NODE_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Node 0x%04X timeout (idle %lums)",
                         node->short_addr, idle);
                node->is_online = false;
                fsm_event_msg_t evt = {
                    .event = EVENT_NODE_PING_TIMEOUT,
                    .payload.node.short_addr = node->short_addr,
                    .timestamp_ms = now
                };
                memcpy(evt.payload.node.eui64, node->eui64, sizeof(zb_eui64_t));
                xQueueSend(s_fsm_queue, &evt, 0);
            }
        }
        if (node->is_sleepy && node->is_online) {
            uint32_t idle = now - node->last_poll;
            if (idle > (NODE_TIMEOUT_MS * 2)) {
                ESP_LOGW(TAG, "Sleepy node 0x%04X timeout", node->short_addr);
                node->is_online = false;
            }
        }
    }
    return ESP_OK;
}

/*=============================================================================
 * ZIGBEE CALLBACKS
 *============================================================================*/

static void zb_zdo_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP: {
            fsm_event_msg_t evt = {
                .event = EVENT_ZB_START_DONE,
                .timestamp_ms = get_timestamp_ms()
            };
            xQueueSend(s_fsm_queue, &evt, 0);
            break;
        }

        case ESP_ZB_BDB_SIGNAL_FORMATION: {
            fsm_event_msg_t evt = {
                .event = (err_status == ESP_OK) ? EVENT_NET_FORM_SUCCESS : EVENT_NET_FORM_FAILED,
                .timestamp_ms = get_timestamp_ms()
            };
            xQueueSend(s_fsm_queue, &evt, 0);
            break;
        }

        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
            esp_zb_zdo_signal_device_annce_params_t *params =
                (esp_zb_zdo_signal_device_annce_params_t *)
                esp_zb_app_signal_get_params(p_sg_p);
            fsm_event_msg_t evt = {
                .event = EVENT_NODE_JOIN_ACCEPT,
                .payload.node.short_addr = params->device_short_addr,
                .payload.node.rssi = 0,
                .payload.node.lqi = 0,
                .timestamp_ms = get_timestamp_ms()
            };
            memcpy(evt.payload.node.eui64, params->ieee_addr, sizeof(zb_eui64_t));
            xQueueSend(s_fsm_queue, &evt, 0);
            break;
        }

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            esp_zb_zdo_signal_leave_params_t *params =
                (esp_zb_zdo_signal_leave_params_t *)
                esp_zb_app_signal_get_params(p_sg_p);
            fsm_event_msg_t evt = {
                .event = EVENT_NODE_LEAVE_NOTIFY,
                .payload.node.short_addr = params->short_addr,
                .timestamp_ms = get_timestamp_ms()
            };
            xQueueSend(s_fsm_queue, &evt, 0);
            break;
        }

        default:
            break;
    }
}

/*=============================================================================
 * FSM TASK
 *============================================================================*/

static void fsm_task(void *pvParameters) {
    (void)pvParameters;
    ESP_LOGI(TAG, "FSM task started on Core 0");

    s_current_state = NET_STATE_INIT;

    fsm_event_msg_t init_evt = {
        .event = EVENT_HW_INIT_DONE,
        .timestamp_ms = get_timestamp_ms()
    };
    action_init(&init_evt);

    fsm_event_msg_t event;
    while (1) {
        if (xQueueReceive(s_fsm_queue, &event, pdMS_TO_TICKS(KEEPALIVE_PERIOD_MS)) == pdTRUE) {
            const state_transition_t *trans = find_transition(s_current_state, event.event);
            if (trans != NULL) {
                print_transition(trans);
                if (trans->action != NULL) {
                    esp_err_t err = trans->action(&event);
                    if (err != ESP_OK)
                        ESP_LOGW(TAG, "Action err: %s", esp_err_to_name(err));
                }
                s_current_state = trans->next_state;
                if (s_current_state == NET_STATE_NET_SHUTDOWN) break;
            } else {
                ESP_LOGW(TAG, "No transition: [%s] + %s",
                         network_fsm_state_to_name(s_current_state),
                         network_fsm_event_to_name(event.event));
            }
        } else {
            /* Keepalive timeout */
            if (s_current_state == NET_STATE_NET_OPERATIONAL ||
                s_current_state == NET_STATE_NET_READY ||
                s_current_state == NET_STATE_NODE_OPERATIONAL) {
                fsm_event_msg_t keepalive = {
                    .event = EVENT_TIMER_KEEPALIVE,
                    .timestamp_ms = get_timestamp_ms()
                };
                action_keepalive_timer(&keepalive);
            }
        }
    }

    ESP_LOGI(TAG, "FSM task exiting");
    vTaskDelete(NULL);
}

/*=============================================================================
 * NODE REGISTRY
 *============================================================================*/

esp_err_t node_registry_init(void) {
    if (s_registry_mutex == NULL)
        s_registry_mutex = xSemaphoreCreateMutex();
    if (s_registry_mutex == NULL) return ESP_ERR_NO_MEM;

    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t size = sizeof(s_node_registry);
        err = nvs_get_blob(handle, NVS_KEY_NODES, s_node_registry, &size);
        if (err == ESP_OK) {
            s_node_count = 0;
            for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
                if (s_node_registry[i].short_addr != 0x0000 &&
                    s_node_registry[i].short_addr != 0xFFFF)
                    s_node_count++;
            }
        }
        nvs_close(handle);
    } else {
        memset(s_node_registry, 0, sizeof(s_node_registry));
        s_node_count = 0;
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

esp_err_t node_registry_add(const node_info_t *node) {
    if (node == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (memcmp(s_node_registry[i].eui64, node->eui64, sizeof(zb_eui64_t)) == 0) {
            uint16_t old_short = s_node_registry[i].short_addr;
            s_node_registry[i] = *node;
            s_node_registry[i].short_addr = node->short_addr ? node->short_addr : old_short;
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (s_node_registry[i].short_addr == 0x0000 ||
            s_node_registry[i].short_addr == 0xFFFF) {
            s_node_registry[i] = *node;
            s_node_count++;
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_ERR_NO_MEM;
}

esp_err_t node_registry_remove(const zb_eui64_t eui64) {
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (memcmp(s_node_registry[i].eui64, eui64, sizeof(zb_eui64_t)) == 0) {
            memset(&s_node_registry[i], 0, sizeof(node_info_t));
            if (s_node_count > 0) s_node_count--;
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t node_registry_find_by_eui64(const zb_eui64_t eui64, node_info_t *node) {
    if (node == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (memcmp(s_node_registry[i].eui64, eui64, sizeof(zb_eui64_t)) == 0) {
            *node = s_node_registry[i];
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t node_registry_find_by_short_addr(uint16_t short_addr, node_info_t *node) {
    if (node == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (s_node_registry[i].short_addr == short_addr) {
            *node = s_node_registry[i];
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t node_registry_update_status(const zb_eui64_t eui64, int8_t rssi, uint8_t lqi) {
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (memcmp(s_node_registry[i].eui64, eui64, sizeof(zb_eui64_t)) == 0) {
            s_node_registry[i].last_rssi = rssi;
            s_node_registry[i].last_lqi = lqi;
            s_node_registry[i].last_seen = get_timestamp_ms();
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

esp_err_t node_registry_set_online(const zb_eui64_t eui64, bool is_online) {
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (memcmp(s_node_registry[i].eui64, eui64, sizeof(zb_eui64_t)) == 0) {
            s_node_registry[i].is_online = is_online;
            if (is_online) s_node_registry[i].last_seen = get_timestamp_ms();
            xSemaphoreGive(s_registry_mutex);
            return ESP_OK;
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_ERR_NOT_FOUND;
}

uint8_t node_registry_get_count(void) { return s_node_count; }

uint8_t node_registry_get_online_count(void) {
    uint8_t count = 0;
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++)
        if (s_node_registry[i].is_online) count++;
    xSemaphoreGive(s_registry_mutex);
    return count;
}

esp_err_t node_registry_foreach(esp_err_t (*cb)(node_info_t *node, void *ud), void *ud) {
    if (cb == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < NETWORK_MAX_CHILDREN; i++) {
        if (s_node_registry[i].short_addr != 0x0000 &&
            s_node_registry[i].short_addr != 0xFFFF) {
            esp_err_t err = cb(&s_node_registry[i], ud);
            if (err != ESP_OK) { xSemaphoreGive(s_registry_mutex); return err; }
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

esp_err_t node_registry_persist(void) {
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(handle, NVS_KEY_NODES, s_node_registry, sizeof(s_node_registry));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
    }
    xSemaphoreGive(s_registry_mutex);
    return err;
}

/*=============================================================================
 * NETWORK CONFIG
 *============================================================================*/

esp_err_t network_config_load(network_config_t *config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t size = sizeof(network_config_t);
        err = nvs_get_blob(handle, NVS_KEY_CONFIG, config, &size);
        nvs_close(handle);
    }
    return err;
}

esp_err_t network_config_save(const network_config_t *config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    s_config = *config;
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        err = nvs_set_blob(handle, NVS_KEY_CONFIG, config, sizeof(network_config_t));
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);
    }
    return err;
}

esp_err_t network_config_reset(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err == ESP_OK) {
        nvs_erase_key(handle, NVS_KEY_CONFIG);
        nvs_erase_key(handle, NVS_KEY_NODES);
        nvs_erase_key(handle, NVS_KEY_STATS);
        nvs_commit(handle);
        nvs_close(handle);
    }
    memset(&s_config, 0, sizeof(s_config));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_node_registry, 0, sizeof(s_node_registry));
    s_node_count = 0;
    return ESP_OK;
}

/*=============================================================================
 * STATISTICS
 *============================================================================*/

esp_err_t network_stats_get(network_stats_t *stats) {
    if (stats == NULL) return ESP_ERR_INVALID_ARG;
    *stats = s_stats;
    return ESP_OK;
}

esp_err_t network_stats_reset(void) {
    memset(&s_stats, 0, sizeof(s_stats));
    return ESP_OK;
}

void network_stats_increment(const char *counter_name) {
    if (strcmp(counter_name, "nodes_joined") == 0) s_stats.nodes_joined_total++;
    else if (strcmp(counter_name, "nodes_left") == 0) s_stats.nodes_left_total++;
    else if (strcmp(counter_name, "rejoins") == 0) s_stats.rejoins_total++;
    else if (strcmp(counter_name, "frames_tx") == 0) s_stats.frames_tx++;
    else if (strcmp(counter_name, "frames_rx") == 0) s_stats.frames_rx++;
    else if (strcmp(counter_name, "frames_dropped") == 0) s_stats.frames_dropped++;
}

/*=============================================================================
 * FSM PUBLIC API
 *============================================================================*/

esp_err_t network_fsm_init(const network_config_t *config) {
    ESP_LOGI(TAG, "Initializing Network FSM (Dual-SoC)...");

    s_fsm_queue = xQueueCreate(NETWORK_FSM_QUEUE_SIZE, sizeof(fsm_event_msg_t));
    if (s_fsm_queue == NULL) return ESP_ERR_NO_MEM;

    s_fsm_event_group = xEventGroupCreate();
    if (s_fsm_event_group == NULL) {
        vQueueDelete(s_fsm_queue);
        return ESP_ERR_NO_MEM;
    }

    if (s_registry_mutex == NULL) {
        s_registry_mutex = xSemaphoreCreateMutex();
        if (s_registry_mutex == NULL) {
            vEventGroupDelete(s_fsm_event_group);
            vQueueDelete(s_fsm_queue);
            return ESP_ERR_NO_MEM;
        }
    }

    if (config != NULL) s_config = *config;

    BaseType_t ret = xTaskCreatePinnedToCore(
        fsm_task, FSM_TASK_NAME, FSM_TASK_STACK_SIZE,
        NULL, FSM_TASK_PRIORITY, &s_fsm_task_handle, 0);

    if (ret != pdPASS) {
        vSemaphoreDelete(s_registry_mutex);
        vEventGroupDelete(s_fsm_event_group);
        vQueueDelete(s_fsm_queue);
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "FSM initialized on Core 0");
    return ESP_OK;
}

esp_err_t network_fsm_send_event(const fsm_event_msg_t *event, TickType_t timeout_ms) {
    if (s_fsm_queue == NULL || event == NULL) return ESP_ERR_INVALID_STATE;
    if (xQueueSend(s_fsm_queue, event, timeout_ms) != pdTRUE) return ESP_ERR_TIMEOUT;
    return ESP_OK;
}

network_fsm_state_t network_fsm_get_current_state(void) { return s_current_state; }

const char *network_fsm_state_to_name(network_fsm_state_t state) {
    if (state >= NET_STATE_MAX) state = NET_STATE_MAX;
    return s_state_names[state];
}

const char *network_fsm_event_to_name(network_fsm_event_t event) {
    if (event >= EVENT_MAX) event = EVENT_MAX;
    return s_event_names[event];
}

EventBits_t network_fsm_wait_for_event(EventBits_t bits, uint32_t timeout_ms) {
    if (s_fsm_event_group == NULL) return 0;
    return xEventGroupWaitBits(s_fsm_event_group, bits, pdTRUE, pdFALSE,
                                pdMS_TO_TICKS(timeout_ms));
}

esp_err_t network_fsm_shutdown(void) {
    fsm_event_msg_t evt = {
        .event = EVENT_CMD_SHUTDOWN,
        .timestamp_ms = get_timestamp_ms()
    };
    return network_fsm_send_event(&evt, portMAX_DELAY);
}

/*=============================================================================
 * COMMANDS
 *============================================================================*/

esp_err_t network_cmd_permit_join(uint8_t duration_sec) {
    fsm_event_msg_t evt = {
        .event = EVENT_CMD_PERMIT_JOIN,
        .payload.cmd.param = duration_sec,
        .timestamp_ms = get_timestamp_ms()
    };
    return network_fsm_send_event(&evt, portMAX_DELAY);
}

esp_err_t network_cmd_deny_join(void) {
    fsm_event_msg_t evt = {
        .event = EVENT_CMD_DENY_JOIN,
        .timestamp_ms = get_timestamp_ms()
    };
    return network_fsm_send_event(&evt, portMAX_DELAY);
}

esp_err_t network_cmd_remove_node(const zb_eui64_t eui64, bool force) {
    fsm_event_msg_t evt = {
        .event = force ? EVENT_NODE_LEAVE_FORCE : EVENT_NODE_LEAVE_REQUEST,
        .timestamp_ms = get_timestamp_ms()
    };
    memcpy(evt.payload.node.eui64, eui64, sizeof(zb_eui64_t));
    return network_fsm_send_event(&evt, portMAX_DELAY);
}

esp_err_t network_cmd_reset_network(void) {
    fsm_event_msg_t evt = {
        .event = EVENT_CMD_RESET_NETWORK,
        .timestamp_ms = get_timestamp_ms()
    };
    return network_fsm_send_event(&evt, portMAX_DELAY);
}

esp_err_t network_cmd_get_config(network_config_t *config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    *config = s_config;
    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

esp_err_t network_cmd_set_config(const network_config_t *config) {
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    return network_config_save(config);
}
    if (config == NULL) return ESP_ERR_INVALID_ARG;
    return network_config_save(config);
}
