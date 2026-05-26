/**
 * @file sensor_registry.c
 * @brief Universal Sensor Registry Implementation
 *
 * Collects all enabled sensor drivers (selected via menuconfig/Kconfig)
 * and provides a unified interface. ZCL cluster auto-registration uses
 * the combined capability mask.
 *
            s_sensors[i].ops->sleep();
        }
    }
    return ESP_OK;
}

esp_err_t sensor_registry_wakeup_all(void)
{
    for (int i = 0; i < s_sensor_count; i++) {
        if (s_sensors[i].initialized && s_sensors[i].ops->wakeup) {
            s_sensors[i].ops->wakeup();
        }
    }
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

static i2c_master_bus_handle_t s_shared_i2c_bus = NULL;
static SemaphoreHandle_t s_i2c_mutex = NULL;

i2c_master_bus_handle_t sensor_registry_get_i2c_bus_handle(void)
{
    return s_shared_i2c_bus;
}

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
