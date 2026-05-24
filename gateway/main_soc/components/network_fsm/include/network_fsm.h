/**
 * @file network_fsm.h
 * @brief Network Finite State Machine for Zigbee Gateway (Dual-SoC)
 *
 * Stage 1: Hardware & Network FSM Architecture
 * Hardware: ESP32-S3 (Wi-Fi Host) + ESP32-H2 (802.15.4 RCP via UART)
 *
 * Manages the complete lifecycle of the Zigbee network including
 * RCP (Radio Co-Processor) initialization and health monitoring.
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-dualsoc
 */

#ifndef NETWORK_FSM_H
#define NETWORK_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_zigbee.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * CONSTANTS
 *============================================================================*/

#define NETWORK_MAX_CHILDREN        CONFIG_ZB_MAX_CHILDREN
#define NETWORK_MAX_RETRY_ATTEMPTS  3
#define NETWORK_OPERATION_TIMEOUT_MS 30000
#define NODE_JOIN_TIMEOUT_MS        60000
#define NODE_REJOIN_BACKOFF_BASE_MS 5000
#define NODE_REJOIN_MAX_BACKOFF_MS  300000
#define NETWORK_FSM_QUEUE_SIZE      16

#define NETWORK_EVENT_NET_READY     BIT0
#define NETWORK_EVENT_NET_FAILED    BIT1
#define NETWORK_EVENT_NODE_JOINED   BIT2
#define NETWORK_EVENT_NODE_LEFT     BIT3
#define NETWORK_EVENT_NET_RECOVER   BIT4

/*=============================================================================
 * FSM STATES
 *============================================================================*/

typedef enum {
    /* Initialization */
    NET_STATE_INIT = 0,
    NET_STATE_HW_INIT,              /**< ESP32-S3 peripherals + ESP32-H2 RCP */
    NET_STATE_RCP_INIT,             /**< RCP UART communication */
    NET_STATE_RCP_READY,            /**< RCP confirmed operational */
    NET_STATE_ZB_INIT,              /**< Zigbee stack initialization */

    /* Network formation */
    NET_STATE_NET_FORMATION,        /**< Forming Zigbee network */
    NET_STATE_NET_READY,            /**< Network formed, accepting joins */

    /* Operational */
    NET_STATE_NODE_JOINING,
    NET_STATE_NODE_OPERATIONAL,
    NET_STATE_NODE_REJOIN,
    NET_STATE_NODE_LEAVING,
    NET_STATE_NET_OPERATIONAL,

    /* Recovery and failure */
    NET_STATE_NET_RECOVER,
    NET_STATE_NET_FAILED,
    NET_STATE_NET_SHUTDOWN,

    NET_STATE_MAX
} network_fsm_state_t;

/*=============================================================================
 * FSM EVENTS
 *============================================================================*/

typedef enum {
    /* Hardware events */
    EVENT_HW_INIT_DONE = 0,
    EVENT_HW_INIT_FAILED,

    /* RCP events (Dual-SoC specific) */
    EVENT_RCP_INIT_DONE,            /**< RCP UART initialized */
    EVENT_RCP_READY,                /**< RCP ready signal received */
    EVENT_RCP_ERROR,                /**< RCP communication error */
    EVENT_RCP_RECOVERED,            /**< RCP recovered from error */

    /* Zigbee stack events */
    EVENT_ZB_INIT_DONE,
    EVENT_ZB_INIT_FAILED,
    EVENT_ZB_START_DONE,
    EVENT_ZB_START_FAILED,

    /* Network events */
    EVENT_NET_FORM_SUCCESS,
    EVENT_NET_FORM_FAILED,
    EVENT_NET_CHANNEL_CONFLICT,

    /* Node management events */
    EVENT_NODE_JOIN_REQUEST,
    EVENT_NODE_JOIN_ACCEPT,
    EVENT_NODE_JOIN_REJECT,
    EVENT_NODE_JOIN_TIMEOUT,
    EVENT_NODE_LEAVE_REQUEST,
    EVENT_NODE_LEAVE_NOTIFY,
    EVENT_NODE_LEAVE_FORCE,
    EVENT_NODE_REJOIN_REQUEST,
    EVENT_NODE_REJOIN_SUCCESS,
    EVENT_NODE_REJOIN_FAILED,
    EVENT_NODE_DATA_RECEIVED,
    EVENT_NODE_PING_TIMEOUT,

    /* Recovery events */
    EVENT_NET_LINK_FAIL,
    EVENT_NET_RECOVER_START,
    EVENT_NET_RECOVER_DONE,
    EVENT_NET_RECOVER_FAILED,

    /* Command events */
    EVENT_CMD_PERMIT_JOIN,
    EVENT_CMD_DENY_JOIN,
    EVENT_CMD_RESET_NETWORK,
    EVENT_CMD_SHUTDOWN,

    /* Timer events */
    EVENT_TIMER_KEEPALIVE,
    EVENT_TIMER_REJOIN_BACKOFF,

    EVENT_MAX
} network_fsm_event_t;

/*=============================================================================
 * DATA STRUCTURES
 *============================================================================*/

typedef uint8_t zb_eui64_t[8];

/** @brief Node info tracked per joined device */
typedef struct {
    zb_eui64_t eui64;
    uint16_t   short_addr;
    uint8_t    endpoint;
    uint16_t   profile_id;
    uint16_t   device_id;
    uint8_t    device_version;
    bool       is_online;
    bool       is_sleepy;
    int8_t     last_rssi;
    uint8_t    last_lqi;
    uint32_t   joined_at;
    uint32_t   last_seen;
    uint32_t   last_poll;
    uint8_t    rejoin_count;
    uint32_t   rejoin_backoff_ms;
    union {
        uint16_t raw;
        struct {
            uint16_t mains_powered : 1;
            uint16_t rx_on_when_idle : 1;
            uint16_t has_onoff : 1;
            uint16_t has_level : 1;
            uint16_t has_temp : 1;
            uint16_t has_humidity : 1;
            uint16_t has_occupancy : 1;
            uint16_t has_ias : 1;
            uint16_t has_pressure : 1;
            uint16_t has_illuminance : 1;
            uint16_t reserved : 6;
        } caps;
    } device_caps;
} node_info_t;

/** @brief Network configuration persisted in NVS */
typedef struct {
    uint8_t  channel;
    uint16_t pan_id;
    uint8_t  ext_pan_id[8];
    uint8_t  network_key[16];
    uint32_t channel_mask;
    bool     permit_join;
    uint16_t permit_join_duration;
    bool     require_link_key;
    uint8_t  trust_center_address[8];
} network_config_t;

/** @brief Network statistics */
typedef struct {
    uint32_t nodes_joined_total;
    uint32_t nodes_left_total;
    uint32_t rejoins_total;
    uint32_t join_failures;
    uint32_t frames_tx;
    uint32_t frames_rx;
    uint32_t frames_dropped;
    uint32_t network_uptime_sec;
    uint32_t last_recovery_at;
} network_stats_t;

/** @brief FSM event message */
typedef struct {
    network_fsm_event_t event;
    union {
        struct { zb_eui64_t eui64; uint16_t short_addr; int8_t rssi; uint8_t lqi; } node;
        struct { uint8_t  channel; uint16_t pan_id; uint8_t  reason; } net;
        struct { uint32_t param; void    *data; } cmd;
        struct { uint32_t error_code; } hw;
        struct { uint32_t rcp_error; } rcp;
    } payload;
    uint32_t timestamp_ms;
} fsm_event_msg_t;

/** @brief State transition table entry */
typedef struct {
    network_fsm_state_t  current_state;
    network_fsm_event_t  event;
    network_fsm_state_t  next_state;
    esp_err_t (*action)(const fsm_event_msg_t *event);
    const char *name;
} state_transition_t;

/*=============================================================================
 * FUNCTION DECLARATIONS - NODE REGISTRY
 *============================================================================*/

esp_err_t node_registry_init(void);
esp_err_t node_registry_add(const node_info_t *node);
esp_err_t node_registry_remove(const zb_eui64_t eui64);
esp_err_t node_registry_find_by_eui64(const zb_eui64_t eui64, node_info_t *node);
esp_err_t node_registry_find_by_short_addr(uint16_t short_addr, node_info_t *node);
esp_err_t node_registry_update_status(const zb_eui64_t eui64, int8_t rssi, uint8_t lqi);
esp_err_t node_registry_set_online(const zb_eui64_t eui64, bool is_online);
uint8_t   node_registry_get_count(void);
uint8_t   node_registry_get_online_count(void);
esp_err_t node_registry_foreach(esp_err_t (*cb)(node_info_t *node, void *ud), void *ud);
esp_err_t node_registry_persist(void);

/*=============================================================================
 * FUNCTION DECLARATIONS - NETWORK CONFIG & STATS
 *============================================================================*/

esp_err_t network_config_load(network_config_t *config);
esp_err_t network_config_save(const network_config_t *config);
esp_err_t network_config_reset(void);
esp_err_t network_stats_get(network_stats_t *stats);
esp_err_t network_stats_reset(void);
void      network_stats_increment(const char *counter_name);

/*=============================================================================
 * FUNCTION DECLARATIONS - FSM CORE
 *============================================================================*/

esp_err_t network_fsm_init(const network_config_t *config);
esp_err_t network_fsm_send_event(const fsm_event_msg_t *event, TickType_t timeout_ms);
network_fsm_state_t network_fsm_get_current_state(void);
const char *network_fsm_state_to_name(network_fsm_state_t state);
const char *network_fsm_event_to_name(network_fsm_event_t event);
EventBits_t network_fsm_wait_for_event(EventBits_t bits, uint32_t timeout_ms);
esp_err_t network_fsm_shutdown(void);

/*=============================================================================
 * FUNCTION DECLARATIONS - COMMANDS
 *============================================================================*/

esp_err_t network_cmd_permit_join(uint8_t duration_sec);
esp_err_t network_cmd_deny_join(void);
esp_err_t network_cmd_remove_node(const zb_eui64_t eui64, bool force);
esp_err_t network_cmd_reset_network(void);
esp_err_t network_cmd_get_config(network_config_t *config);
esp_err_t network_cmd_set_config(const network_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* NETWORK_FSM_H */
