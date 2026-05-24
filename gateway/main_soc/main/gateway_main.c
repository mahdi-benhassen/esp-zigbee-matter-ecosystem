/**
 * @file gateway_main.c
 * @brief Gateway Main Application - Dual-SoC ESP32-S3 + ESP32-H2
 *
 * Entry point for the Zigbee Coordinator / Gateway firmware.
 *
 * Hardware: ESP32-S3 (Wi-Fi Host + Application Core)
 *           ESP32-H2 (802.15.4 Radio Co-Processor via UART)
 *
 * Wiring (ESP32-S3 <-> ESP32-H2):
 *   S3 GPIO5 (UART1_TX) -> H2 GPIO8 (UART_RX)
 *   S3 GPIO4 (UART1_RX) <- H2 GPIO9 (UART_TX)
 *   S3 3.3V             -> H2 3.3V
 *   S3 GND              -> H2 GND
 *
 * Flash ESP32-H2 with RCP firmware first:
 *   cd $IDF_PATH/examples/openthread/ot_rcp
 *   idf.py set-target esp32h2 flash
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-dualsoc
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "network_fsm.h"
#include "rcp_uart.h"

/*=============================================================================
 * DEFINITIONS
 *============================================================================*/

#define TAG "GW_MAIN"
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1
#define WIFI_MAX_RETRY      5

/*=============================================================================
 * WIFI
 *============================================================================*/

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_count = 0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_count < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_wifi_retry_count++;
            ESP_LOGW(TAG, "Wi-Fi retry %d/%d", s_wifi_retry_count, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE(TAG, "Wi-Fi failed after %d retries", WIFI_MAX_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Wi-Fi IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_wifi_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static esp_err_t wifi_init_sta(void)
{
    ESP_LOGI(TAG, "Initializing Wi-Fi (ESP32-S3)...");
    s_wifi_event_group = xEventGroupCreate();
    if (s_wifi_event_group == NULL) return ESP_ERR_NO_MEM;

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {.capable = true, .required = false},
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE, pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi connected");
        return ESP_OK;
    }
    return (bits & WIFI_FAIL_BIT) ? ESP_FAIL : ESP_ERR_TIMEOUT;
}

/*=============================================================================
 * MAIN
 *============================================================================*/

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Smart Home IoT Gateway");
    ESP_LOGI(TAG, "  HW: ESP32-S3 + ESP32-H2 (Dual-SoC)");
    ESP_LOGI(TAG, "  Stage 1: Network FSM Architecture");
    ESP_LOGI(TAG, "========================================");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Host: %s (cores:%d, rev:%d)",
             CONFIG_IDF_TARGET, chip_info.cores, chip_info.revision);
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);
    ESP_LOGI(TAG, "Flash: %lu MB", (unsigned long)(flash_size / (1024 * 1024)));
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());

    /* [1] NVS */
    ESP_LOGI(TAG, "[1/5] NVS init...");
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* [2] Wi-Fi */
    ESP_LOGI(TAG, "[2/5] Wi-Fi init...");
    err = wifi_init_sta();
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Wi-Fi init: %s (continuing)", esp_err_to_name(err));

    /* [3] RCP UART check (pre-FSM diagnostic) */
    ESP_LOGI(TAG, "[3/5] RCP UART diagnostic...");
    rcp_uart_print_diagnostics();

    /* [4] Network FSM (handles RCP init internally) */
    ESP_LOGI(TAG, "[4/5] Network FSM init...");
    err = network_fsm_init(NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FSM init failed: %s", esp_err_to_name(err));
        return;
    }

    /* [5] Wait for network formation */
    ESP_LOGI(TAG, "[5/5] Waiting for Zigbee network...");
    EventBits_t bits = network_fsm_wait_for_event(
        NETWORK_EVENT_NET_READY | NETWORK_EVENT_NET_FAILED, 120000);

    if (bits & NETWORK_EVENT_NET_READY) {
        ESP_LOGI(TAG, "========================================");
        ESP_LOGI(TAG, "  Gateway ONLINE");
        ESP_LOGI(TAG, "  Zigbee via ESP32-H2 RCP operational");
        ESP_LOGI(TAG, "========================================");
    } else if (bits & NETWORK_EVENT_NET_FAILED) {
        ESP_LOGE(TAG, "Network formation failed!");
        network_cmd_reset_network();
    } else {
        ESP_LOGW(TAG, "Network formation timeout");
    }

    ESP_LOGI(TAG, "State: %s", network_fsm_state_to_name(network_fsm_get_current_state()));
    ESP_LOGI(TAG, "Free heap: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Stage 1 complete. Ready for Stage 2.");

    /* Periodic status */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(30000));
        network_stats_t stats;
        if (network_stats_get(&stats) == ESP_OK) {
            ESP_LOGI(TAG, "Status: state=%s, nodes=%d/%d, uptime=%lus, heap=%lu",
                     network_fsm_state_to_name(network_fsm_get_current_state()),
                     node_registry_get_online_count(), node_registry_get_count(),
                     stats.network_uptime_sec, esp_get_free_heap_size());
        }
        /* Periodic RCP diagnostic */
        if (!rcp_uart_is_operational()) {
            ESP_LOGW(TAG, "RCP not operational!");
            rcp_uart_print_diagnostics();
        }
    }
}
