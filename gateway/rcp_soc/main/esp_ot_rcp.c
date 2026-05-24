/**
 * @file esp_ot_rcp.c
 * @brief OpenThread Radio Co-Processor (RCP) firmware for ESP32-H2
 *
 * This firmware turns the ESP32-H2 into an 802.15.4 Radio Co-Processor
 * that communicates with the ESP32-S3 host over UART using the Spinel
 * protocol with HDLC-lite framing.
 *
 * Hardware connections (ESP32-H2 side):
 *   H2 GPIO9 (UART_TX)  -> S3 GPIO4 (UART1_RX)
 *   H2 GPIO8 (UART_RX)  <- S3 GPIO5 (UART1_TX)
 *   H2 3.3V             <- S3 3.3V
 *   H2 GND              <- S3 GND
 */

#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_openthread.h"
#include "esp_openthread_types.h"
#include "esp_vfs_eventfd.h"
#include "driver/uart.h"
#include "nvs_flash.h"

#define TAG "OT_RCP"

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  ESP32-H2 Radio Co-Processor (RCP)");
    ESP_LOGI(TAG, "  Protocol: Spinel over HDLC-lite");
    ESP_LOGI(TAG, "  UART: 115200 8N1 (GPIO8/GPIO9)");
    ESP_LOGI(TAG, "========================================");

    /* Initialize NVS — required by OpenThread */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* Initialize the event-fd virtual filesystem (required by OpenThread) */
    esp_vfs_eventfd_config_t eventfd_config = {
        .max_fds = 3,
    };
    ESP_ERROR_CHECK(esp_vfs_eventfd_register(&eventfd_config));
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Initialize OpenThread config in RCP mode */
    static esp_openthread_config_t config = {
        .platform_config = {
            .radio_config = {
                .radio_mode = RADIO_MODE_NATIVE,
            },
            .host_config = {
                .host_connection_mode = HOST_CONNECTION_MODE_RCP_UART,
                .host_uart_config = {
                    .port = UART_NUM_0,
                    .uart_config = {
                        .baud_rate = 115200,
                        .data_bits = UART_DATA_8_BITS,
                        .parity = UART_PARITY_DISABLE,
                        .stop_bits = UART_STOP_BITS_1,
                        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                        .rx_flow_ctrl_thresh = 0,
                        .source_clk = UART_SCLK_DEFAULT,
                    },
                    .rx_pin = 8,  /* ESP32-H2 RX pin connected to S3 TX (GPIO5) */
                    .tx_pin = 9,  /* ESP32-H2 TX pin connected to S3 RX (GPIO4) */
                },
            },
            .port_config = {
                .storage_partition_name = "nvs",
                .netif_queue_size = 10,
                .task_queue_size = 10,
            },
        },
    };

    /* Start OpenThread stack */
    ESP_ERROR_CHECK(esp_openthread_start(&config));

    ESP_LOGI(TAG, "RCP initialized and running");
}
