#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "esp_zigbee.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "sensor_hub.h"
#include "zigbee_config.h"

static const char *TAG = "MAIN_APP";

/* Event Group Bits */
#define EVENT_NET_JOINED    BIT0
#define EVENT_NET_FAILED    BIT1
#define EVENT_CAN_SLEEP     BIT2

static EventGroupHandle_t s_evt_group = NULL;
static SemaphoreHandle_t s_nvs_mutex = NULL;



/*=============================================================================
 * ZIGBEE CALLBACKS & SIGNAL HANDLER
 *============================================================================*/
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *sig = signal_struct->p_app_signal;
    esp_err_t status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *sig;

    switch (sig_type) {
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Zigbee stack started successfully. Starting steering/joining...");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGE(TAG, "Zigbee device start failed, status: %s", esp_err_to_name(status));
                xEventGroupSetBits(s_evt_group, EVENT_NET_FAILED);
            }
            break;

        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (status == ESP_OK) {
                ESP_LOGI(TAG, "Joined network successfully!");
                xEventGroupSetBits(s_evt_group, EVENT_NET_JOINED);
            } else {
                ESP_LOGW(TAG, "Network steering failed, status: %s", esp_err_to_name(status));
                xEventGroupSetBits(s_evt_group, EVENT_NET_FAILED);
            }
            break;

        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            ESP_LOGI(TAG, "ZBOSS Stack signals: CAN_SLEEP. Tx queues and buffers flushed.");
            xEventGroupSetBits(s_evt_group, EVENT_CAN_SLEEP);
            break;

        default:
            ESP_LOGI(TAG, "Zigbee stack signal received: 0x%02X, status: %s", sig_type, esp_err_to_name(status));
            break;
    }
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    ESP_LOGD(TAG, "ZCL Action callback: 0x%02X", callback_id);
    return ESP_OK;
}

/*=============================================================================
 * BACKGROUND ZIGBEE TASK (Stack loop)
 *============================================================================*/
static void zigbee_main_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Launching Zigbee Main Loop...");
    esp_zigbee_launch_mainloop();
    vTaskDelete(NULL);
}

/*=============================================================================
 * ISOLATED SENSOR RUNTIME AND CONTROL TASK
 *============================================================================*/
static void sensor_execution_task(void *pvParameters)
{
    sensor_hub_data_t data;
    esp_err_t err;

    /* 1. Wake cause analysis */
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_TIMER) {
        ESP_LOGI(TAG, "Woken up from Deep Sleep by RTC Timer. Bypassing unnecessary peripheral re-inits.");
    } else {
        ESP_LOGI(TAG, "Cold boot / POR reset. Initializing hardware interfaces from scratch.");
    }

    /* 2. Energize sensor rails (Power-Gate ON) */
    sensor_hub_power_on();

    /* 3. Non-blocking delay for voltage rail and transducers stabilization */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 4. Trigger conversions for slow sensors (e.g. SCD41 CO2, DS18B20 1-Wire) */
    sensor_hub_trigger_conversions();

    /* 5. Allow sensors to perform measurements (CO2 needs ~5 seconds in single-shot mode) */
    ESP_LOGI(TAG, "Waiting 5 seconds for sensor conversions to complete...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    /* 6. Collect values from all active sensor components */
    ESP_LOGI(TAG, "Collecting data from sensor matrix...");
    err = sensor_hub_collect(&data);

    /* 7. Drop sensor rails (Power-Gate OFF) immediately to prevent leakage current */
    sensor_hub_power_off();

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to collect any valid sensor data. Shutting down.");
    } else {
        ESP_LOGI(TAG, "Sensor collection completed successfully.");
    }

    /* 8. Activate Zigbee Stack and join/rejoin network */
    ESP_LOGI(TAG, "Activating Zigbee Stack...");
    zigbee_sed_stack_init();
    ESP_ERROR_CHECK(zigbee_data_model_init());
    esp_zb_core_action_handler_register(zb_action_handler);

    /* Create isolated Zigbee stack task */
    xTaskCreate(zigbee_main_task, "zigbee_task", 8192, NULL, 5, NULL);

    /* Start commissioning/join FSM */
    ESP_ERROR_CHECK(esp_zb_start(false));

    /* 9. Wait for network association confirmation */
    ESP_LOGI(TAG, "Waiting for network association...");
    EventBits_t bits = xEventGroupWaitBits(s_evt_group, EVENT_NET_JOINED | EVENT_NET_FAILED, pdTRUE, pdFALSE, pdMS_TO_TICKS(15000));

    if (bits & EVENT_NET_JOINED) {
        ESP_LOGI(TAG, "Network association confirmed. Reporting metrics...");
        /* 10. Map values to ZCL clusters and trigger reports */
        zigbee_report_sensor_data(&data);

        /* 11. Wait for ZBOSS queue to flush and stack to indicate CAN_SLEEP */
        ESP_LOGI(TAG, "Waiting for stack buffers to clear...");
        xEventGroupWaitBits(s_evt_group, EVENT_CAN_SLEEP, pdTRUE, pdFALSE, pdMS_TO_TICKS(5000));
    } else {
        ESP_LOGE(TAG, "Network association failed or timed out. Bypassing report.");
    }

    /* 12. Deep Sleep entry configuration */
    const uint64_t sleep_interval_sec = 15 * 60; /* 15 minutes sleep interval */
    ESP_LOGI(TAG, "Configuring deep sleep. Wake-up interval: %llu minutes.", sleep_interval_sec / 60);
    
    esp_sleep_enable_timer_wakeup(sleep_interval_sec * 1000000ULL);
    
    ESP_LOGI(TAG, "Entering Deep Sleep... ~7uA target current.");
    esp_deep_sleep_start();

    /* Never reached */
    vTaskDelete(NULL);
}

/*=============================================================================
 * APPLICATION MAIN ENTRY
 *============================================================================*/
void app_main(void)
{
    ESP_LOGI(TAG, "=============================================");
    ESP_LOGI(TAG, "Smart Agricultural Sleepy End Device Starting");
    ESP_LOGI(TAG, "Microcontroller: ESP32-H2 (RISC-V 96MHz)");
    ESP_LOGI(TAG, "=============================================");

    /* Initialize synchronization primitives */
    s_evt_group = xEventGroupCreate();
    s_nvs_mutex = xSemaphoreCreateMutex();

    if (!s_evt_group || !s_nvs_mutex) {
        ESP_LOGE(TAG, "Failed to create FreeRTOS sync primitives");
        return;
    }

    /* Guard NVS initialization with Mutex */
    xSemaphoreTake(s_nvs_mutex, portMAX_DELAY);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    xSemaphoreGive(s_nvs_mutex);

    /* Initialize basic hardware sensor interfaces (I2C, ADC, UART, GPIO) */
    ESP_ERROR_CHECK(sensor_hub_init());

    /* Launch the sensor control task with isolated stack memory */
    xTaskCreate(sensor_execution_task, "sensor_exec_task", 4096, NULL, 6, NULL);
}
