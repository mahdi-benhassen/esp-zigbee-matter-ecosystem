/**
 * @file aht20_sensor.c
 * @brief Aosong AHT20 Temperature/Humidity Sensor Driver
 *
 * I2C interface. Good accuracy: +/- 0.3C, +/- 2% RH
 * Address: 0x38 (fixed)
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

#define TAG "AHT20"

#define AHT20_I2C_ADDR          0x38
#define AHT20_I2C_PORT          0

/* Commands */
#define AHT20_CMD_INIT          0xBE1B  /* Initialization: 0xBE, 0x08, 0x00 */
#define AHT20_CMD_TRIGGER       0xAC33  /* Trigger measurement: 0xAC, 0x33, 0x00 */
#define AHT20_CMD_SOFT_RESET    0xBA
#define AHT20_CMD_STATUS        0x71

/* Status bits */
#define AHT20_STATUS_BUSY       (1 << 7)
#define AHT20_STATUS_CALIBRATED (1 << 3)

#define AHT20_MEASURE_DELAY_MS  80

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
static i2c_master_dev_handle_t s_aht20_dev = NULL;
static bool s_initialized = false;

static esp_err_t aht20_init(void)
{
    ESP_LOGI(TAG, "AHT20 init: SDA=GPIO%d, SCL=GPIO%d",
             CONFIG_SENSOR_I2C_SDA_PIN, CONFIG_SENSOR_I2C_SCL_PIN);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = AHT20_I2C_PORT,
        .sda_io_num = CONFIG_SENSOR_I2C_SDA_PIN,
        .scl_io_num = CONFIG_SENSOR_I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_config, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C bus failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AHT20_I2C_ADDR,
        .scl_speed_hz = CONFIG_SENSOR_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_aht20_dev);
    if (err != ESP_OK) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    /* Soft reset */
    uint8_t reset_cmd = AHT20_CMD_SOFT_RESET;
    i2c_master_transmit(s_aht20_dev, &reset_cmd, 1, -1);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Initialize */
    uint8_t init_cmd[3] = {0xBE, 0x08, 0x00};
    i2c_master_transmit(s_aht20_dev, init_cmd, 3, -1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Verify calibration */
    uint8_t status_cmd = AHT20_CMD_STATUS;
    uint8_t status = 0;
    i2c_master_transmit_receive(s_aht20_dev, &status_cmd, 1, &status, 1, -1);
    if (!(status & AHT20_STATUS_CALIBRATED)) {
        ESP_LOGW(TAG, "AHT20 not calibrated (status=0x%02X), retrying init...", status);
        i2c_master_transmit(s_aht20_dev, init_cmd, 3, -1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    s_initialized = true;
    ESP_LOGI(TAG, "AHT20 initialized");
    return ESP_OK;
}

static esp_err_t aht20_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) return ESP_ERR_INVALID_STATE;

    /* Trigger measurement */
    uint8_t trigger[3] = {0xAC, 0x33, 0x00};
    esp_err_t err = i2c_master_transmit(s_aht20_dev, trigger, 3, -1);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(AHT20_MEASURE_DELAY_MS));

    /* Poll for completion */
    uint8_t rx_buf[7] = {0};
    uint8_t status_cmd = AHT20_CMD_STATUS;
    for (int i = 0; i < 10; i++) {
        err = i2c_master_transmit_receive(s_aht20_dev, &status_cmd, 1, rx_buf, 7, -1);
        if (err != ESP_OK) return err;
        if (!(rx_buf[0] & AHT20_STATUS_BUSY)) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* Parse data: status[1], humidity[20 bits], temp[20 bits] */
    uint32_t raw_hum = (((uint32_t)rx_buf[1]) << 12) |
                       (((uint32_t)rx_buf[2]) << 4) |
                       (((uint32_t)rx_buf[3]) >> 4);
    uint32_t raw_temp = ((((uint32_t)rx_buf[3]) & 0x0F) << 16) |
                        (((uint32_t)rx_buf[4]) << 8) |
                        ((uint32_t)rx_buf[5]);

    float humidity = ((float)raw_hum / (1 << 20)) * 100.0f;
    float temp_c = ((float)raw_temp / (1 << 20)) * 200.0f - 50.0f;

    if (humidity < 0) humidity = 0;
    if (humidity > 100) humidity = 100;

    memset(data, 0, sizeof(sensor_data_t));
    data->temperature.value.f32 = temp_c;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    data->humidity.value.f32 = humidity;
    data->humidity.valid = true;
    data->humidity.timestamp_ms = data->temperature.timestamp_ms;

    ESP_LOGD(TAG, "AHT20: T=%.2fC, H=%.1f%%", temp_c, humidity);
    return ESP_OK;
}

static esp_err_t aht20_sleep(void) { return ESP_OK; }
static esp_err_t aht20_wakeup(void) { return ESP_OK; }

static esp_err_t aht20_deinit(void)
{
    if (s_aht20_dev) { i2c_master_bus_rm_device(s_aht20_dev); s_aht20_dev = NULL; }
    if (s_i2c_bus) { i2c_del_master_bus(s_i2c_bus); s_i2c_bus = NULL; }
    s_initialized = false;
    return ESP_OK;
}

static const sensor_info_t s_info = {
    .name = "Aosong AHT20",
    .model = "AHT20",
    .capabilities = CAP_TEMPERATURE | CAP_HUMIDITY,
    .min_interval_ms = 2000,
    .default_interval_ms = 10000,
    .supports_sleep = true,
};

static const sensor_info_t *aht20_get_info(void) { return &s_info; }

static const sensor_ops_t aht20_ops = {
    .init = aht20_init, .read = aht20_read,
    .sleep = aht20_sleep, .wakeup = aht20_wakeup,
    .get_info = aht20_get_info, .deinit = aht20_deinit,
};

const sensor_ops_t aht20_sensor_ops = aht20_ops;
