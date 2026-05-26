/**
 * @file sensor_registry.c
 * @brief Universal Sensor Registry Implementation
 *
 * Collects all enabled sensor drivers (selected via menuconfig/Kconfig)
 * and provides a unified interface. ZCL cluster auto-registration uses
 * the combined capability mask.
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>
#include <math.h>

#define TAG "SENSOR_REG"

/*=============================================================================
 * MODULE STATE
 *============================================================================*/

/** Max number of sensor slots */
#define MAX_SENSORS 12

static struct {
    const sensor_ops_t *ops;
    const sensor_info_t *info;
    bool initialized;
} s_sensors[MAX_SENSORS] = {0};

static uint8_t s_sensor_count = 0;
static uint32_t s_combined_caps = 0;
static bool s_registry_initialized = false;
static SemaphoreHandle_t s_registry_mutex = NULL;

static i2c_master_bus_handle_t s_shared_i2c_bus = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;

/*=============================================================================
 * STATIC: Register a sensor at init time
 *============================================================================*/

static esp_err_t register_sensor(const sensor_ops_t *ops)
{
    if (ops == NULL || s_sensor_count >= MAX_SENSORS) {
        return ESP_ERR_NO_MEM;
    }

    const sensor_info_t *info = ops->get_info();
    if (info == NULL) {
        ESP_LOGW(TAG, "Sensor has no info, skipping");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Registering sensor: %s (%s) caps=0x%08X",
             info->name, info->model, info->capabilities);

    /* Initialize the sensor */
    esp_err_t err = ops->init();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Sensor %s init failed: %s", info->name, esp_err_to_name(err));
        return err;
    }

    s_sensors[s_sensor_count].ops = ops;
    s_sensors[s_sensor_count].info = info;
    s_sensors[s_sensor_count].initialized = true;
    s_combined_caps |= info->capabilities;
    s_sensor_count++;

    ESP_LOGI(TAG, "  -> Registered (total: %d)", s_sensor_count);
    return ESP_OK;
}

/*=============================================================================
 * PUBLIC API
 *============================================================================*/

esp_err_t sensor_registry_init(void)
{
    if (s_registry_mutex == NULL) {
        s_registry_mutex = xSemaphoreCreateMutex();
        if (s_registry_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        if (s_i2c_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);

    if (s_registry_initialized) {
        xSemaphoreGive(s_registry_mutex);
        return ESP_OK;
    }

    ESP_LOGI(TAG, "=== Initializing Sensor Registry ===");
    s_sensor_count = 0;
    s_combined_caps = 0;
    memset(s_sensors, 0, sizeof(s_sensors));

    /*
     * Register each enabled sensor (controlled by Kconfig).
     * The order determines read priority if multiple sensors
     * provide the same capability.
     */

#ifdef CONFIG_SENSOR_SHT30
    register_sensor(&sht30_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_SHT4X
    register_sensor(&sht4x_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_AHT20
    register_sensor(&aht20_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_DHT22
    register_sensor(&dht22_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_INTERNAL
    register_sensor(&internal_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_BME280
    register_sensor(&bme280_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_BH1750
    register_sensor(&bh1750_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_VEML7700
    register_sensor(&veml7700_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_SCD41
    register_sensor(&scd41_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_DS18B20
    register_sensor(&ds18b20_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_ZE03_NH3
    register_sensor(&ze03_nh3_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_SOIL_MOISTURE
    register_sensor(&soil_moisture_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_JSN_SR04T
    register_sensor(&jsn_sr04t_sensor_ops);
#endif

#ifdef CONFIG_SENSOR_HX711
    register_sensor(&hx711_sensor_ops);
#endif

    /* Fallback: if nothing registered, use stub */
#ifdef CONFIG_SENSOR_STUB
    if (s_sensor_count == 0) {
        ESP_LOGW(TAG, "No sensors registered, using stub");
        register_sensor(&stub_sensor_ops);
    }
#endif

    s_registry_initialized = true;

    ESP_LOGI(TAG, "=== Registry Summary ===");
    ESP_LOGI(TAG, "  Active sensors: %d", s_sensor_count);
    ESP_LOGI(TAG, "  Capabilities: 0x%08X", s_combined_caps);
    sensor_registry_print();

    xSemaphoreGive(s_registry_mutex);
    return (s_sensor_count > 0) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t sensor_registry_deinit(void)
{
    if (s_registry_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);

    for (int i = 0; i < s_sensor_count; i++) {
        if (s_sensors[i].initialized && s_sensors[i].ops->deinit) {
            s_sensors[i].ops->deinit();
        }
    }
    s_sensor_count = 0;
    s_combined_caps = 0;
    s_registry_initialized = false;
    memset(s_sensors, 0, sizeof(s_sensors));

    if (s_i2c_mutex != NULL) {
        xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);
    }
    if (s_shared_i2c_bus != NULL) {
        i2c_del_master_bus(s_shared_i2c_bus);
        s_shared_i2c_bus = NULL;
    }
    if (s_i2c_mutex != NULL) {
        xSemaphoreGive(s_i2c_mutex);
    }

    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

esp_err_t sensor_registry_read_all(sensor_data_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_registry_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);

    if (!s_registry_initialized) {
        xSemaphoreGive(s_registry_mutex);
        return ESP_ERR_INVALID_STATE;
    }

    /* Clear output */
    memset(data, 0, sizeof(sensor_data_t));

    for (int i = 0; i < s_sensor_count; i++) {
        if (!s_sensors[i].initialized) continue;

        sensor_data_t sensor_reading = {0};
        esp_err_t err = s_sensors[i].ops->read(&sensor_reading);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Sensor %d read failed: %s", i, esp_err_to_name(err));
            continue;
        }

        /* Merge readings - later sensors overwrite earlier ones for same cap */
        if (sensor_reading.temperature.valid) data->temperature = sensor_reading.temperature;
        if (sensor_reading.humidity.valid)     data->humidity     = sensor_reading.humidity;
        if (sensor_reading.pressure.valid)     data->pressure     = sensor_reading.pressure;
        if (sensor_reading.illuminance.valid)  data->illuminance  = sensor_reading.illuminance;
        if (sensor_reading.occupancy.valid)    data->occupancy    = sensor_reading.occupancy;
        if (sensor_reading.onoff.valid)        data->onoff        = sensor_reading.onoff;
        if (sensor_reading.level.valid)        data->level        = sensor_reading.level;
        if (sensor_reading.soil_moisture.valid)data->soil_moisture= sensor_reading.soil_moisture;
        if (sensor_reading.co2.valid)          data->co2          = sensor_reading.co2;
        if (sensor_reading.pm25.valid)         data->pm25         = sensor_reading.pm25;
        if (sensor_reading.nh3.valid)          data->nh3          = sensor_reading.nh3;
        if (sensor_reading.distance.valid)     data->distance     = sensor_reading.distance;
        if (sensor_reading.weight.valid)       data->weight       = sensor_reading.weight;
        if (sensor_reading.soil_temp.valid)    data->soil_temp    = sensor_reading.soil_temp;
    }

    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

esp_err_t sensor_registry_sleep_all(void)
{
    if (s_registry_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < s_sensor_count; i++) {
        if (s_sensors[i].initialized && s_sensors[i].ops->sleep) {
            s_sensors[i].ops->sleep();
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

esp_err_t sensor_registry_wakeup_all(void)
{
    if (s_registry_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    xSemaphoreTake(s_registry_mutex, portMAX_DELAY);
    for (int i = 0; i < s_sensor_count; i++) {
        if (s_sensors[i].initialized && s_sensors[i].ops->wakeup) {
            s_sensors[i].ops->wakeup();
        }
    }
    xSemaphoreGive(s_registry_mutex);
    return ESP_OK;
}

uint32_t sensor_registry_get_capabilities(void)
{
    return s_combined_caps;
}

uint8_t sensor_registry_get_count(void)
{
    return s_sensor_count;
}

esp_err_t sensor_registry_get_sensor(uint8_t idx, const sensor_info_t **info,
                                      const sensor_ops_t **ops)
{
    if (idx >= s_sensor_count || info == NULL) {
        return ESP_ERR_NOT_FOUND;
    }
    *info = s_sensors[idx].info;
    if (ops) *ops = s_sensors[idx].ops;
    return ESP_OK;
}

void sensor_registry_print(void)
{
    ESP_LOGI(TAG, "Registered sensors:");
    for (int i = 0; i < s_sensor_count; i++) {
        const sensor_info_t *info = s_sensors[i].info;
        ESP_LOGI(TAG, "  [%d] %s (%s) caps=0x%04X interval=%lums sleep=%s",
                 i, info->name, info->model, info->capabilities,
                 info->default_interval_ms,
                 info->supports_sleep ? "yes" : "no");
    }
}

esp_err_t sensor_data_to_zcl(const sensor_data_t *data, sensor_capability_t cap,
                              void *zcl_value)
{
    if (data == NULL || zcl_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch (cap) {
        case CAP_TEMPERATURE:
            if (!data->temperature.valid) return ESP_ERR_INVALID_STATE;
            *(int16_t *)zcl_value = (int16_t)(data->temperature.value.f32 * 100.0f);
            break;

        case CAP_HUMIDITY:
            if (!data->humidity.valid) return ESP_ERR_INVALID_STATE;
            *(uint16_t *)zcl_value = (uint16_t)(data->humidity.value.f32 * 100.0f);
            break;

        case CAP_PRESSURE:
            if (!data->pressure.valid) return ESP_ERR_INVALID_STATE;
            *(int16_t *)zcl_value = (int16_t)(data->pressure.value.f32 * 10.0f);
            break;

        case CAP_ILLUMINANCE: {
            if (!data->illuminance.valid) return ESP_ERR_INVALID_STATE;
            /* ZCL: 10000 * log10(lux) + 1 */
            float lux = data->illuminance.value.f32;
            if (lux <= 0) lux = 1.0f;
            *(uint16_t *)zcl_value = (uint16_t)(10000.0f * log10f(lux) + 1.0f);
            break;
        }

        case CAP_OCCUPANCY:
            if (!data->occupancy.valid) return ESP_ERR_INVALID_STATE;
            *(uint8_t *)zcl_value = data->occupancy.value.b ? 1 : 0;
            break;

        case CAP_ONOFF:
            if (!data->onoff.valid) return ESP_ERR_INVALID_STATE;
            *(uint8_t *)zcl_value = data->onoff.value.b ? 1 : 0;
            break;

        case CAP_LEVEL_CONTROL:
            if (!data->level.valid) return ESP_ERR_INVALID_STATE;
            *(uint8_t *)zcl_value = data->level.value.u8;
            break;

        case CAP_SOIL_MOISTURE:
            if (!data->soil_moisture.valid) return ESP_ERR_INVALID_STATE;
            *(uint16_t *)zcl_value = (uint16_t)(data->soil_moisture.value.f32 * 100.0f);
            break;

        case CAP_CO2:
            if (!data->co2.valid) return ESP_ERR_INVALID_STATE;
            *(float *)zcl_value = data->co2.value.f32;
            break;

        case CAP_PM25:
            if (!data->pm25.valid) return ESP_ERR_INVALID_STATE;
            *(float *)zcl_value = data->pm25.value.f32;
            break;

        case CAP_NH3:
            if (!data->nh3.valid) return ESP_ERR_INVALID_STATE;
            /* NH3 reported as volume fraction (ppm * 1e-6) */
            *(float *)zcl_value = data->nh3.value.f32 * 1e-6f;
            break;

        case CAP_DISTANCE:
            if (!data->distance.valid) return ESP_ERR_INVALID_STATE;
            *(uint16_t *)zcl_value = (uint16_t)(data->distance.value.f32);
            break;

        case CAP_WEIGHT:
            if (!data->weight.valid) return ESP_ERR_INVALID_STATE;
            /* Weight reported in grams (kg * 1000) */
            *(uint32_t *)zcl_value = (uint32_t)(data->weight.value.f32 * 1000.0f);
            break;

        case CAP_SOIL_TEMP:
            if (!data->soil_temp.valid) return ESP_ERR_INVALID_STATE;
            /* Soil temp in ZCL hundredths of degrees C */
            *(int16_t *)zcl_value = (int16_t)(data->soil_temp.value.f32 * 100.0f);
            break;

        default:
            return ESP_ERR_NOT_SUPPORTED;
    }

    return ESP_OK;
}

#ifndef CONFIG_SENSOR_I2C_SDA_PIN
#define CONFIG_SENSOR_I2C_SDA_PIN 6
#endif
#ifndef CONFIG_SENSOR_I2C_SCL_PIN
#define CONFIG_SENSOR_I2C_SCL_PIN 7
#endif



i2c_master_bus_handle_t sensor_registry_get_i2c_bus(void)
{
    if (s_i2c_mutex == NULL) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        if (s_i2c_mutex == NULL) return NULL;
    }

    xSemaphoreTake(s_i2c_mutex, portMAX_DELAY);

    if (s_shared_i2c_bus == NULL) {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = 0,
            .sda_io_num = CONFIG_SENSOR_I2C_SDA_PIN,
            .scl_io_num = CONFIG_SENSOR_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t err = i2c_new_master_bus(&bus_config, &s_shared_i2c_bus);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to create shared I2C bus: %s", esp_err_to_name(err));
            s_shared_i2c_bus = NULL;
        } else {
            ESP_LOGI(TAG, "Shared I2C bus initialized: SDA=GPIO%d, SCL=GPIO%d", 
                     CONFIG_SENSOR_I2C_SDA_PIN, CONFIG_SENSOR_I2C_SCL_PIN);
        }
    }

    xSemaphoreGive(s_i2c_mutex);
    return s_shared_i2c_bus;
}
