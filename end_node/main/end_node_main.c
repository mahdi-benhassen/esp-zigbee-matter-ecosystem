/**
 * @file end_node_main.c
 * @brief Universal End Node - Sensor-Agnostic Firmware
 *
 * ONE codebase for ALL sensor types. Build configuration (Kconfig) selects
 * which sensor driver(s) to compile. ZCL clusters are auto-registered based
 * on sensor capabilities.
 *
 * Supported sensors (select via menuconfig):
 *   - SHT30, SHT4x, AHT20, DHT22, Internal, Stub/Simulation
 *
 * Build:
 *   idf.py menuconfig  -> Component config -> End Node Sensor Configuration
 *   idf.py build
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "driver/gpio.h"
#include "esp_chip_info.h"
#include "esp_flash.h"

#include "esp_zigbee.h"
#include "esp_zigbee_type.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zdo/esp_zigbee_zdo_common.h"
#include "zdo/esp_zigbee_zdo_command.h"

static void do_sleep_cycle(void);
/* Universal sensor architecture */
#include "sensor_registry.h"
#include "zcl_cluster_config.h"

/*=============================================================================
 * DEFINITIONS
 *============================================================================*/

#define TAG "END_NODE"
#define LED_GPIO                    GPIO_NUM_8

/* Intervals from Kconfig */
#ifndef CONFIG_SENSOR_READ_INTERVAL_SEC
#define CONFIG_SENSOR_READ_INTERVAL_SEC     30
#endif
#ifndef CONFIG_SENSOR_REPORT_INTERVAL_SEC
#define CONFIG_SENSOR_REPORT_INTERVAL_SEC   60
#endif
#ifndef CONFIG_SLEEP_DURATION_SEC
#define CONFIG_SLEEP_DURATION_SEC           30
#endif

#define SENSOR_READ_INTERVAL_MS     (CONFIG_SENSOR_READ_INTERVAL_SEC * 1000)
#define REPORT_INTERVAL_MS          (CONFIG_SENSOR_REPORT_INTERVAL_SEC * 1000)
#define SLEEP_DURATION_US           ((uint64_t)CONFIG_SLEEP_DURATION_SEC * 1000000ULL)
#define MAX_SLEEP_US                (30 * 60 * 1000000ULL)  /* 30 min */
#define NETWORK_JOIN_TIMEOUT_MS     30000
#define NETWORK_MAX_RETRY           5

/* Event Group Bits */
#define EVENT_NET_JOINED            BIT0
#define EVENT_NET_FAILED            BIT1
#define EVENT_SENSOR_READ           BIT2
#define EVENT_REPORT_DUE            BIT3

/*=============================================================================
 * FSM
 *============================================================================*/

typedef enum {
    EN_STATE_INIT = 0,
    EN_STATE_HW_INIT,
    EN_STATE_SENSORS_INIT,
    EN_STATE_ZB_INIT,
    EN_STATE_NETWORK_SCAN,
    EN_STATE_JOINING,
    EN_STATE_REJOINING,
    EN_STATE_JOINED,
    EN_STATE_OPERATIONAL,
    EN_STATE_LEAVING,
    EN_STATE_SHUTDOWN,
    EN_STATE_MAX
} en_state_t;

typedef enum {
    EN_EVENT_HW_DONE = 0,
    EN_EVENT_SENSORS_DONE,
    EN_EVENT_SENSORS_FAILED,
    EN_EVENT_ZB_DONE,
    EN_EVENT_ZB_FAILED,
    EN_EVENT_SCAN_DONE,
    EN_EVENT_SCAN_FAIL,
    EN_EVENT_JOIN_OK,
    EN_EVENT_JOIN_FAIL,
    EN_EVENT_REJOIN_OK,
    EN_EVENT_REJOIN_FAIL,
    EN_EVENT_LEAVE_DONE,
    EN_EVENT_LEAVE_REQ,
    EN_EVENT_SENSOR_TRIG,
    EN_EVENT_REPORT_TRIG,
    EN_EVENT_SHUTDOWN,
    EN_EVENT_MAX
} en_event_t;

static volatile en_state_t s_state = EN_STATE_INIT;
static EventGroupHandle_t s_evt_group = NULL;
static uint32_t s_join_attempts = 0;
static uint32_t s_sensor_reads = 0;
static uint32_t s_reports = 0;

/* Network info persistence */
typedef struct {
    uint8_t  channel;
    uint16_t pan_id;
    uint8_t  has_info;
} net_info_t;

static net_info_t s_net_info = {0};

/* Sensor data */
static sensor_data_t s_sensor_data = {0};

/*=============================================================================
 * NVS HELPERS
 *============================================================================*/

static esp_err_t net_info_save(uint8_t channel, uint16_t pan_id)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("zb_net", NVS_READWRITE, &h);
    if (err != ESP_OK) return err;
    s_net_info.channel = channel;
    s_net_info.pan_id = pan_id;
    s_net_info.has_info = 0x01;
    nvs_set_blob(h, "info", &s_net_info, sizeof(s_net_info));
    nvs_commit(h);
    nvs_close(h);
    return ESP_OK;
}

static esp_err_t net_info_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("zb_net", NVS_READONLY, &h);
    if (err != ESP_OK) return err;
    size_t sz = sizeof(s_net_info);
    err = nvs_get_blob(h, "info", &s_net_info, &sz);
    nvs_close(h);
    return err;
}

static void net_info_clear(void)
{
    memset(&s_net_info, 0, sizeof(s_net_info));
    nvs_handle_t h;
    if (nvs_open("zb_net", NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, "info");
        nvs_commit(h);
        nvs_close(h);
    }
}

/*=============================================================================
 * ZIGBEE CALLBACKS
 *============================================================================*/

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *sig = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *sig;

    switch (sig_type) {
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
            ESP_LOGI(TAG, "ZDO: DEVICE_FIRST_START");
            s_state = EN_STATE_JOINING;
            break;

        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            ESP_LOGI(TAG, "ZDO: DEVICE_REBOOT");
            if (s_net_info.has_info == 0x01) {
                ESP_LOGI(TAG, "Using stored network info for rejoin");
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "ZDO: JOIN SUCCESS");
                xEventGroupSetBits(s_evt_group, EVENT_NET_JOINED);
            } else {
                ESP_LOGW(TAG, "ZDO: JOIN FAILED (status=%d)", status);
                xEventGroupSetBits(s_evt_group, EVENT_NET_FAILED);
            }
            break;

        case ESP_ZB_ZDO_SIGNAL_LEAVE: {
            ESP_LOGI(TAG, "ZDO: LEAVE");
            xEventGroupSetBits(s_evt_group, EVENT_NET_FAILED);
            break;
        }

        default:
            break;
    }
}

static esp_err_t en_attr_handler(const esp_zb_zcl_set_attr_value_message_t *msg)
{
    if (!msg) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Attr change: cluster=0x%04X, attr=0x%04X",
             msg->info.cluster, msg->attribute.id);

    if (msg->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY &&
        msg->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID) {
        uint16_t time = *(uint16_t *)msg->attribute.data.value;
        ESP_LOGI(TAG, "Identify: %us", time);
        for (int i = 0; i < (time > 0 ? 6 : 0); i++) {
            gpio_set_level(LED_GPIO, 1); vTaskDelay(pdMS_TO_TICKS(250));
            gpio_set_level(LED_GPIO, 0); vTaskDelay(pdMS_TO_TICKS(250));
        }
    }
    return ESP_OK;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        ret = en_attr_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    }
    return ret;
}

/*=============================================================================
 * STATE HANDLERS
 *============================================================================*/

static const char *state_name(en_state_t st) {
    const char *names[] = {"INIT","HW_INIT","SENSORS_INIT","ZB_INIT",
        "NET_SCAN","JOINING","REJOINING","JOINED","OPERATIONAL",
        "LEAVING","SHUTDOWN","UNKNOWN"};
    if (st >= EN_STATE_MAX) st = EN_STATE_MAX;
    return names[st];
}

static void do_init(void)
{
    ESP_LOGI(TAG, "=== Universal End Node Starting ===");

    /* LED */
    gpio_config_t led = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&led);
    gpio_set_level(LED_GPIO, 1);

    /* NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Load persisted network info */
    err = net_info_load();
    if (err == ESP_OK && s_net_info.has_info == 0x01) {
        ESP_LOGI(TAG, "Stored net: PAN=0x%04X, Ch=%d", s_net_info.pan_id, s_net_info.channel);
    } else {
        ESP_LOGI(TAG, "No stored network info");
        memset(&s_net_info, 0, sizeof(s_net_info));
    }

    s_state = EN_STATE_HW_INIT;
}

static void do_hw_init(void)
{
    ESP_LOGI(TAG, "HW_INIT: ESP32 ready");
    s_state = EN_STATE_SENSORS_INIT;
}

static void do_sensors_init(void)
{
    ESP_LOGI(TAG, "SENSORS_INIT: Initializing sensor registry...");

    esp_err_t err = sensor_registry_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Sensor init failed: %s", esp_err_to_name(err));
        s_state = EN_STATE_SHUTDOWN;
        return;
    }

    /* Print what we got */
    sensor_registry_print();
    zcl_clusters_print_config();

    ESP_LOGI(TAG, "Sensors ready: %d sensor(s), caps=0x%08X",
             sensor_registry_get_count(), sensor_registry_get_capabilities());

    s_state = EN_STATE_ZB_INIT;
}

static void do_zb_init(void)
{
    ESP_LOGI(TAG, "ZB_INIT: Initializing Zigbee stack...");

    /* Platform config */
    esp_zb_platform_config_t plat = {
        .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&plat));

    /* Device config - SED or Router */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
                .keep_alive = 3000,
            }
        }
    };
    esp_zb_init(&zb_cfg);

    /* Dynamic ZCL cluster registration based on sensor capabilities */
    ESP_ERROR_CHECK(zcl_clusters_init());

    /* Callbacks */
    esp_zb_core_action_handler_register(zb_action_handler);

    ESP_LOGI(TAG, "Zigbee init complete, device: %s",
             zcl_clusters_get_device_name());

    s_state = EN_STATE_NETWORK_SCAN;
}

static void do_network_scan(void)
{
    ESP_LOGI(TAG, "NET_SCAN: Starting network discovery...");

    if (s_net_info.has_info == 0x01) {
        ESP_LOGI(TAG, "Attempting rejoin with stored credentials...");
        s_state = EN_STATE_REJOINING;
        s_join_attempts = 0;
        return;
    }

    /* Start Zigbee - will trigger steering on first start */
    ESP_ERROR_CHECK(esp_zb_start(false));
    s_state = EN_STATE_JOINING;
    s_join_attempts = 0;
}

static void do_joining(void)
{
    ESP_LOGI(TAG, "JOINING: attempt %lu/%d", s_join_attempts + 1, NETWORK_MAX_RETRY);
    s_join_attempts++;

    esp_err_t err = esp_zb_bdb_start_top_level_commissioning(
        ESP_ZB_BDB_MODE_NETWORK_STEERING);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Steering failed: %s", esp_err_to_name(err));
    }

    /* Wait for result */
    EventBits_t bits = xEventGroupWaitBits(s_evt_group,
        EVENT_NET_JOINED | EVENT_NET_FAILED, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(NETWORK_JOIN_TIMEOUT_MS));

    if (bits & EVENT_NET_JOINED) {
        ESP_LOGI(TAG, "JOINED! Network access confirmed");
        s_state = EN_STATE_JOINED;
    } else if (bits & EVENT_NET_FAILED) {
        if (s_join_attempts < NETWORK_MAX_RETRY) {
            ESP_LOGW(TAG, "Join failed, retrying...");
            vTaskDelay(pdMS_TO_TICKS(3000 * s_join_attempts));
            /* Retry */
        } else {
            ESP_LOGE(TAG, "Join failed after %d attempts", NETWORK_MAX_RETRY);
            s_state = EN_STATE_SHUTDOWN;
        }
    } else {
        ESP_LOGW(TAG, "Join timeout");
        if (s_join_attempts >= NETWORK_MAX_RETRY) {
            s_state = EN_STATE_SHUTDOWN;
        }
    }
}

static void do_rejoining(void)
{
    ESP_LOGI(TAG, "REJOINING: attempt %lu/%d", s_join_attempts + 1, NETWORK_MAX_RETRY);
    s_join_attempts++;

    /* Start Zigbee - on reboot it will auto-rejoin if network info exists */
    ESP_ERROR_CHECK(esp_zb_start(false));
    s_state = EN_STATE_JOINING;
}

static void do_joined(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "  End Node JOINED - %s", zcl_clusters_get_device_name());
    ESP_LOGI(TAG, "  Sensors: %d active", sensor_registry_get_count());
    ESP_LOGI(TAG, "  Caps: 0x%08X", sensor_registry_get_capabilities());
    ESP_LOGI(TAG, "============================================");

    s_join_attempts = 0;
    gpio_set_level(LED_GPIO, 0);

    /* Persist network info */
    uint8_t ch = esp_zb_get_current_channel();
    uint16_t pan = esp_zb_get_pan_id();
    net_info_save(ch, pan);

    xEventGroupSetBits(s_evt_group, EVENT_NET_JOINED);
    s_state = EN_STATE_OPERATIONAL;
}

static void do_operational(void)
{
    /* Read sensors */
    ESP_LOGI(TAG, "OPERATIONAL: Reading sensors...");
    esp_err_t err = sensor_registry_read_all(&s_sensor_data);
    if (err == ESP_OK) {
        s_sensor_reads++;
        ESP_LOGI(TAG, "  Reads: %lu", s_sensor_reads);

        /* Update ZCL attributes (triggers auto-reporting) */
        zcl_clusters_update_from_sensors(&s_sensor_data);
        s_reports++;
    } else {
        ESP_LOGW(TAG, "Sensor read failed: %s", esp_err_to_name(err));
    }

#ifdef CONFIG_POWER_SLEEPY
    /* Enter deep sleep cycle for SED */
    do_sleep_cycle();
#else
    /* Stay awake, wait for next read interval */
    vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
#endif
}

#ifdef CONFIG_POWER_SLEEPY
static void do_sleep_cycle(void)
{
    ESP_LOGI(TAG, "Sleep cycle: %d seconds...", CONFIG_SLEEP_DURATION_SEC);

    /* Put sensors to sleep */
    sensor_registry_sleep_all();

    /* Configure wake timer */
    uint64_t sleep_us = SLEEP_DURATION_US;
    if (sleep_us > MAX_SLEEP_US) sleep_us = MAX_SLEEP_US;

    ESP_LOGI(TAG, "Entering deep sleep for %llu seconds...", sleep_us / 1000000ULL);
    esp_sleep_enable_timer_wakeup(sleep_us);
    esp_deep_sleep_start();
    /* Never returns - device resets on wake */
}
#endif

static void do_leaving(void)
{
    ESP_LOGI(TAG, "LEAVING: Removing network info...");
    net_info_clear();
    esp_zb_zdo_mgmt_leave_req_param_t req = {0};
    req.rejoin = 0;
    req.remove_children = 0;
    esp_zb_zdo_device_leave_req(&req, NULL, NULL);
    s_state = EN_STATE_SHUTDOWN;
}

static void do_shutdown(void)
{
    ESP_LOGI(TAG, "SHUTDOWN: Cleaning up...");
    sensor_registry_deinit();
    gpio_set_level(LED_GPIO, 0);

    /* Sleep for extended period, then restart */
    ESP_LOGI(TAG, "Sleeping 1 hour before restart...");
    esp_sleep_enable_timer_wakeup(3600000000ULL);
    esp_deep_sleep_start();
}

/*=============================================================================
 * MAIN FSM LOOP
 *============================================================================*/

static void fsm_loop(void)
{
    /* Check wake cause */
    esp_sleep_wakeup_cause_t wake = esp_sleep_get_wakeup_cause();
    if (wake == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Wake: timer (deep sleep wake)");
    } else if (wake == ESP_SLEEP_WAKEUP_EXT0 || wake == ESP_SLEEP_WAKEUP_EXT1) {
        ESP_LOGI(TAG, "Wake: GPIO");
    } else {
        ESP_LOGI(TAG, "Wake: power-on/reset");
    }

    /* Initialize event group */
    s_evt_group = xEventGroupCreate();
    if (s_evt_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

    /* Run FSM */
    s_state = EN_STATE_INIT;

    while (s_state != EN_STATE_SHUTDOWN) {
        ESP_LOGD(TAG, "FSM state: %s", state_name(s_state));

        switch (s_state) {
            case EN_STATE_INIT:         do_init(); break;
            case EN_STATE_HW_INIT:      do_hw_init(); break;
            case EN_STATE_SENSORS_INIT: do_sensors_init(); break;
            case EN_STATE_ZB_INIT:      do_zb_init(); break;
            case EN_STATE_NETWORK_SCAN: do_network_scan(); break;
            case EN_STATE_JOINING:      do_joining(); break;
            case EN_STATE_REJOINING:    do_rejoining(); break;
            case EN_STATE_JOINED:       do_joined(); break;
            case EN_STATE_OPERATIONAL:  do_operational(); break;
            case EN_STATE_LEAVING:      do_leaving(); break;
            default: break;
        }

        /* Short yield */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Shutdown */
    do_shutdown();
}

/*=============================================================================
 * ENTRY POINT
 *============================================================================*/

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Universal IoT End Node");
    ESP_LOGI(TAG, "  HW: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "  Stage 1: Network FSM + Universal Sensors");
    ESP_LOGI(TAG, "  ====================================");

    esp_chip_info_t ci;
    esp_chip_info(&ci);
    ESP_LOGI(TAG, "Chip: %s, cores:%d, rev:%d",
             CONFIG_IDF_TARGET, ci.cores, ci.revision);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash: %lu MB", (unsigned long)(flash_size / (1024 * 1024)));
    ESP_LOGI(TAG, "Heap: %lu bytes", (unsigned long)esp_get_free_heap_size());

    /* Start FSM */
    fsm_loop();
}
