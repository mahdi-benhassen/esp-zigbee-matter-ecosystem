/**
 * @file internal_sensor.c
 * @brief ESP32 Internal Temperature Sensor Driver
 *
 * Uses the ESP32 on-die temperature sensor. Only provides temperature,
 * no humidity. Useful as a basic fallback or for thermal monitoring.
 *
 * Capabilities: TEMPERATURE only
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "driver/temperature_sensor.h"
#include "esp_timer.h"

#define TAG "INT_SENSOR"

static temperature_sensor_handle_t s_temp_handle = NULL;
static bool s_initialized = false;

static esp_err_t internal_init(void)
{
    temperature_sensor_config_t config = {
        .range_min = -40,
        .range_max = 85,
    };
    esp_err_t err = temperature_sensor_install(&config, &s_temp_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Temp sensor install failed: %s", esp_err_to_name(err));
        return err;
    }
    err = temperature_sensor_enable(s_temp_handle);
    if (err != ESP_OK) {
        temperature_sensor_uninstall(s_temp_handle);
        s_temp_handle = NULL;
        return err;
    }
    s_initialized = true;
    ESP_LOGI(TAG, "Internal temperature sensor initialized");
    return ESP_OK;
}

static esp_err_t internal_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL || s_temp_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    memset(data, 0, sizeof(sensor_data_t));

    float temp_c = 0;
    esp_err_t err = temperature_sensor_get_celsius(s_temp_handle, &temp_c);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Internal sensor reads die temp (often higher than ambient) */
    temp_c -= 5.0f; /* Rough offset for die-to-ambient */

    data->temperature.value.f32 = temp_c;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGD(TAG, "Internal temp: %.2fC", temp_c);
    return ESP_OK;
}

static esp_err_t internal_sleep(void)
{
    if (s_temp_handle) temperature_sensor_disable(s_temp_handle);
    return ESP_OK;
}

static esp_err_t internal_wakeup(void)
{
    if (s_temp_handle) temperature_sensor_enable(s_temp_handle);
    return ESP_OK;
}

static esp_err_t internal_deinit(void)
{
    if (s_temp_handle) {
        temperature_sensor_disable(s_temp_handle);
        temperature_sensor_uninstall(s_temp_handle);
        s_temp_handle = NULL;
    }
    s_initialized = false;
    return ESP_OK;
}

static const sensor_info_t s_info = {
    .name = "ESP Internal Temp",
    .model = "ESP-INT-TEMP",
    .capabilities = CAP_TEMPERATURE,
    .min_interval_ms = 5000,
    .default_interval_ms = 30000,
    .supports_sleep = true,
};

static const sensor_info_t *internal_get_info(void) { return &s_info; }

static const sensor_ops_t internal_ops = {
    .init = internal_init, .read = internal_read,
    .sleep = internal_sleep, .wakeup = internal_wakeup,
    .get_info = internal_get_info, .deinit = internal_deinit,
};

const sensor_ops_t internal_sensor_ops = internal_ops;
