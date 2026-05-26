/**
 * @file sht4x_sensor.c
 * @brief Sensirion SHT40/SHT41 Temperature/Humidity Sensor Driver
 *
 * I2C interface. Highest accuracy: +/- 0.2C (SHT40) / +/- 0.1C (SHT41), +/- 1.8% RH
 * Address: 0x44 (fixed)
 *
 * Capabilities: TEMPERATURE + HUMIDITY
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#define TAG "SHT4X"

#define SHT4X_I2C_ADDR          0x44
#define SHT4X_I2C_PORT          0

/* Commands */
#define SHT4X_CMD_MEAS_HIGH     0xFD    /* High precision, 8.3ms */
#define SHT4X_CMD_MEAS_MEDIUM   0xF6    /* Medium precision, 4.8ms */
#define SHT4X_CMD_MEAS_LOW      0xE0    /* Low precision, 1.7ms */
#define SHT4X_CMD_SOFT_RESET    0x94
#define SHT4X_CMD_READ_SERIAL   0x89

/* Timing */
#define SHT4X_MEASURE_DELAY_MS  10

#ifndef CONFIG_SENSOR_I2C_SDA_PIN
#define CONFIG_SENSOR_I2C_SDA_PIN 6
#endif
#ifndef CONFIG_SENSOR_I2C_SCL_PIN
#define CONFIG_SENSOR_I2C_SCL_PIN 7
#endif
#ifndef CONFIG_SENSOR_I2C_FREQ_HZ
#define CONFIG_SENSOR_I2C_FREQ_HZ 100000
#endif

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_sht4x_dev = NULL;
static bool s_initialized = false;

static uint8_t sht4x_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
        }
    }
    return crc;
}

static esp_err_t sht4x_init(void)
{
    ESP_LOGI(TAG, "SHT4x init using shared I2C bus");

    s_i2c_bus = sensor_registry_get_i2c_bus();
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus");
        return ESP_FAIL;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT4X_I2C_ADDR,
        .scl_speed_hz = CONFIG_SENSOR_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_sht4x_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SHT4x device: %s", esp_err_to_name(err));
        s_i2c_bus = NULL;
        return err;
    }

    /* Soft reset */
    uint8_t reset_cmd = SHT4X_CMD_SOFT_RESET;
    i2c_master_transmit(s_sht4x_dev, &reset_cmd, 1, 1000);
    vTaskDelay(pdMS_TO_TICKS(1));

    s_initialized = true;
    ESP_LOGI(TAG, "SHT4x initialized");
    return ESP_OK;
}

static esp_err_t sht4x_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t cmd = SHT4X_CMD_MEAS_HIGH;
    esp_err_t err = i2c_master_transmit(s_sht4x_dev, &cmd, 1, 1000);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(SHT4X_MEASURE_DELAY_MS));

    uint8_t rx_buf[6] = {0};
    err = i2c_master_receive(s_sht4x_dev, rx_buf, 6, 1000);
    if (err != ESP_OK) return err;

    /* CRC check */
    if (sht4x_crc8(rx_buf, 2) != rx_buf[2] ||
        sht4x_crc8(&rx_buf[3], 2) != rx_buf[5]) {
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_temp = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    uint16_t raw_hum  = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];

    float temp_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);
    float humidity = -6.0f + 125.0f * ((float)raw_hum / 65535.0f);
    if (humidity < 0) humidity = 0;
    if (humidity > 100) humidity = 100;

    memset(data, 0, sizeof(sensor_data_t));
    data->temperature.value.f32 = temp_c;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    data->humidity.value.f32 = humidity;
    data->humidity.valid = true;
    data->humidity.timestamp_ms = data->temperature.timestamp_ms;

    ESP_LOGD(TAG, "SHT4x: T=%.2fC, H=%.1f%%", temp_c, humidity);
    return ESP_OK;
}

static esp_err_t sht4x_sleep(void) { return ESP_OK; }
static esp_err_t sht4x_wakeup(void) { return ESP_OK; }

static esp_err_t sht4x_deinit(void)
{
    if (s_sht4x_dev) { 
        i2c_master_bus_rm_device(s_sht4x_dev); 
        s_sht4x_dev = NULL; 
    }
    s_i2c_bus = NULL;
    s_initialized = false;
    return ESP_OK;
}

static const sensor_info_t s_info = {
    .name = "Sensirion SHT4x",
    .model = "SHT40/SHT41",
    .capabilities = CAP_TEMPERATURE | CAP_HUMIDITY,
    .min_interval_ms = 2000,
    .default_interval_ms = 10000,
    .supports_sleep = true,
};

static const sensor_info_t *sht4x_get_info(void) { return &s_info; }

static const sensor_ops_t sht4x_ops = {
    .init = sht4x_init, .read = sht4x_read,
    .sleep = sht4x_sleep, .wakeup = sht4x_wakeup,
    .get_info = sht4x_get_info, .deinit = sht4x_deinit,
};

const sensor_ops_t sht4x_sensor_ops = sht4x_ops;
