/**
 * @file stub_sensor.c
 * @brief Stub/Simulation Sensor Driver
 *
 * Generates realistic simulated sensor data for development and testing.
 * No hardware required. This is the fallback when no real sensor is enabled.
 *
 * Capabilities: TEMPERATURE + HUMIDITY (configurable)
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include <math.h>

#define TAG "STUB_SENSOR"

/*=============================================================================
 * SIMULATION STATE (random walk for realistic data)
 *============================================================================*/

static float s_last_temp = 22.0f;
static float s_last_humidity = 50.0f;
static bool s_initialized = false;

/*=============================================================================
 * OPERATIONS
 *============================================================================*/

static esp_err_t stub_init(void)
{
    ESP_LOGI(TAG, "Stub sensor initialized (simulation mode)");
    s_initialized = true;
    s_last_temp = 22.0f;
    s_last_humidity = 50.0f;
    return ESP_OK;
}

static esp_err_t stub_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Random walk temperature: -0.3 to +0.3 C per reading */
    float temp_change = ((float)(esp_random() % 60) - 30.0f) / 100.0f;
    s_last_temp += temp_change;
    /* Clamp to realistic indoor range */
    if (s_last_temp < 10.0f) s_last_temp = 10.0f;
    if (s_last_temp > 40.0f) s_last_temp = 40.0f;

    /* Random walk humidity: -0.5 to +0.5 % per reading */
    float hum_change = ((float)(esp_random() % 100) - 50.0f) / 100.0f;
    s_last_humidity += hum_change;
    if (s_last_humidity < 10.0f) s_last_humidity = 10.0f;
    if (s_last_humidity > 95.0f) s_last_humidity = 95.0f;

    /* Fill sensor data structure */
    memset(data, 0, sizeof(sensor_data_t));

    data->temperature.value.f32 = s_last_temp;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    data->humidity.value.f32 = s_last_humidity;
    data->humidity.valid = true;
    data->humidity.timestamp_ms = data->temperature.timestamp_ms;

    ESP_LOGD(TAG, "Simulated: T=%.2fC, H=%.1f%%", s_last_temp, s_last_humidity);
    return ESP_OK;
}

static esp_err_t stub_sleep(void)
{
    ESP_LOGD(TAG, "Sleep (no-op for stub)");
    return ESP_OK;
}

static esp_err_t stub_wakeup(void)
{
    ESP_LOGD(TAG, "Wakeup (no-op for stub)");
    return ESP_OK;
}

static esp_err_t stub_deinit(void)
{
    s_initialized = false;
    return ESP_OK;
}

/*============================================================================
 * INFO & REGISTRATION
 *============================================================================*/

static const sensor_info_t s_stub_info = {
    .name = "Stub/Simulation",
    .model = "SIM-TH-v1",
    .capabilities = CAP_TEMPERATURE | CAP_HUMIDITY,
    .min_interval_ms = 5000,
    .default_interval_ms = 30000,
    .supports_sleep = true,
};

static const sensor_info_t *stub_get_info(void)
{
    return &s_stub_info;
}

static const sensor_ops_t stub_ops = {
    .init = stub_init,
    .read = stub_read,
    .sleep = stub_sleep,
    .wakeup = stub_wakeup,
    .get_info = stub_get_info,
    .deinit = stub_deinit,
};

/* Register the sensor */
const sensor_ops_t stub_sensor_ops = stub_ops;
