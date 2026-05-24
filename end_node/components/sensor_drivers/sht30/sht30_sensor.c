/**
 * @file sht30_sensor.c
 * @brief Sensirion SHT30 Temperature/Humidity Sensor Driver
 *
 * I2C interface. High accuracy: +/- 0.2C, +/- 2% RH
 * Address: 0x44 (ADDR pin low) or 0x45 (ADDR pin high)
 *
 * Capabilities: TEMPERATURE + HUMIDITY
 *
 * Wiring:
 *   ESP GPIO6 (SDA) -> SHT30 SDA
 *   ESP GPIO7 (SCL) -> SHT30 SCL
 *   ESP 3.3V        -> SHT30 VDD
 *   ESP GND         -> SHT30 GND
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

#define TAG "SHT30"

/* I2C Configuration */
#define SHT30_I2C_ADDR          0x44
#define SHT30_I2C_PORT          0

/* Commands */
#define SHT30_CMD_MEAS_HIGH     0x2400  /* High repeatability, clock stretching */
#define SHT30_CMD_MEAS_MEDIUM   0x240B  /* Medium repeatability */
#define SHT30_CMD_MEAS_LOW      0x2416  /* Low repeatability */
#define SHT30_CMD_SOFT_RESET    0x30A2
#define SHT30_CMD_HEATER_ON     0x306D
#define SHT30_CMD_HEATER_OFF    0x3066
#define SHT30_CMD_READ_STATUS   0xF32D

/* Timing */
#define SHT30_MEASURE_DELAY_MS  15      /* High repeatability ~15ms */

/* I2C bus configuration from Kconfig */
#ifndef CONFIG_SENSOR_I2C_SDA_PIN
#define CONFIG_SENSOR_I2C_SDA_PIN 6
#endif
#ifndef CONFIG_SENSOR_I2C_SCL_PIN
#define CONFIG_SENSOR_I2C_SCL_PIN 7
#endif
#ifndef CONFIG_SENSOR_I2C_FREQ_HZ
#define CONFIG_SENSOR_I2C_FREQ_HZ 100000
#endif

/*=============================================================================
 * MODULE STATE
 *============================================================================*/

static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_sht30_dev = NULL;
static bool s_initialized = false;

/*=============================================================================
 * CRC8 calculation (SHT30 uses CRC-8 with poly 0x31) */
 *============================================================================*/

static uint8_t sht30_crc8(const uint8_t *data, uint8_t len)
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

/*=============================================================================
 * OPERATIONS
 *============================================================================*/

static esp_err_t sht30_init(void)
{
    ESP_LOGI(TAG, "SHT30 init: SDA=GPIO%d, SCL=GPIO%d, Freq=%dHz",
             CONFIG_SENSOR_I2C_SDA_PIN, CONFIG_SENSOR_I2C_SCL_PIN,
             CONFIG_SENSOR_I2C_FREQ_HZ);

    /* Create I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = SHT30_I2C_PORT,
        .sda_io_num = CONFIG_SENSOR_I2C_SDA_PIN,
        .scl_io_num = CONFIG_SENSOR_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus creation failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Add SHT30 device */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT30_I2C_ADDR,
        .scl_speed_hz = CONFIG_SENSOR_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_sht30_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SHT30 device: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    /* Soft reset */
    uint8_t reset_cmd[2] = {0x30, 0xA2};
    i2c_master_transmit(s_sht30_dev, reset_cmd, 2, -1);
    vTaskDelay(pdMS_TO_TICKS(2));

    s_initialized = true;
    ESP_LOGI(TAG, "SHT30 initialized successfully");
    return ESP_OK;
}

static esp_err_t sht30_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL || s_sht30_dev == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Send measurement command (high repeatability) */
    uint8_t cmd[2] = {0x24, 0x00};
    esp_err_t err = i2c_master_transmit(s_sht30_dev, cmd, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Command tx failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Wait for measurement */
    vTaskDelay(pdMS_TO_TICKS(SHT30_MEASURE_DELAY_MS));

    /* Read 6 bytes: T[2] + T_CRC[1] + H[2] + H_CRC[1] */
    uint8_t rx_buf[6] = {0};
    err = i2c_master_receive(s_sht30_dev, rx_buf, 6, -1);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Verify CRC */
    if (sht30_crc8(rx_buf, 2) != rx_buf[2]) {
        ESP_LOGW(TAG, "Temperature CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (sht30_crc8(&rx_buf[3], 2) != rx_buf[5]) {
        ESP_LOGW(TAG, "Humidity CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    /* Parse temperature: T = -45 + 175 * raw / 65535 */
    uint16_t raw_temp = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    float temp_c = -45.0f + 175.0f * ((float)raw_temp / 65535.0f);

    /* Parse humidity: RH = 100 * raw / 65535 */
    uint16_t raw_hum = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
    float humidity = 100.0f * ((float)raw_hum / 65535.0f);

    /* Fill data structure */
    memset(data, 0, sizeof(sensor_data_t));
    data->temperature.value.f32 = temp_c;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    data->humidity.value.f32 = humidity;
    data->humidity.valid = true;
    data->humidity.timestamp_ms = data->temperature.timestamp_ms;

    ESP_LOGD(TAG, "SHT30: T=%.2fC, H=%.1f%%", temp_c, humidity);
    return ESP_OK;
}

static esp_err_t sht30_sleep(void)
{
    /* SHT30 has no explicit sleep command - enters idle automatically */
    ESP_LOGD(TAG, "SHT30 sleep (auto-idle)");
    return ESP_OK;
}

static esp_err_t sht30_wakeup(void)
{
    ESP_LOGD(TAG, "SHT30 wakeup (ready immediately)");
    return ESP_OK;
}

static esp_err_t sht30_deinit(void)
{
    if (s_sht30_dev) {
        i2c_master_bus_rm_device(s_sht30_dev);
        s_sht30_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    s_initialized = false;
    return ESP_OK;
}

static const sensor_info_t s_info = {
    .name = "Sensirion SHT30",
    .model = "SHT30-DIS",
    .capabilities = CAP_TEMPERATURE | CAP_HUMIDITY,
    .min_interval_ms = 2000,
    .default_interval_ms = 10000,
    .supports_sleep = true,
};

static const sensor_info_t *sht30_get_info(void) { return &s_info; }

static const sensor_ops_t sht30_ops = {
    .init = sht30_init, .read = sht30_read,
    .sleep = sht30_sleep, .wakeup = sht30_wakeup,
    .get_info = sht30_get_info, .deinit = sht30_deinit,
};

const sensor_ops_t sht30_sensor_ops = sht30_ops;
