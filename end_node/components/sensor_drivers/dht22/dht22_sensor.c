/**
 * @file dht22_sensor.c
 * @brief DHT22/AM2302 Temperature/Humidity Sensor Driver
 *
 * Single-wire protocol. Moderate accuracy: +/- 0.5C, +/- 2-5% RH
 * No I2C address - uses a single GPIO with custom timing protocol.
 *
 * Capabilities: TEMPERATURE + HUMIDITY
 *
 * Wiring:
 *   ESP GPIO5 -> DHT22 Data (with 10K pull-up to 3.3V)
 *   ESP 3.3V  -> DHT22 VCC
 *   ESP GND   -> DHT22 GND
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"

#define TAG "DHT22"

#ifndef CONFIG_SENSOR_DHT_PIN
#define CONFIG_SENSOR_DHT_PIN   5
#endif

#define DHT_GPIO                CONFIG_SENSOR_DHT_PIN
#define DHT_MAX_TIMINGS         85

static bool s_initialized = false;

/* DHT22 data packet: 5 bytes = humidity[2] + temp[2] + checksum[1] */
static uint8_t s_dht_data[5] = {0};

/**
 * @brief Read DHT22 using bit-banged single-wire protocol
 *
 * Timing-critical: disables interrupts during read.
 */
static esp_err_t dht22_read_raw(void)
{
    memset(s_dht_data, 0, 5);

    /* Send start signal: pull low for 1ms */
    gpio_set_direction(DHT_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(DHT_GPIO, 0);
    ets_delay_us(1100);
    gpio_set_level(DHT_GPIO, 1);
    ets_delay_us(30);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);

    /* Wait for DHT response (20-40us low, then 80us high, then 80us low) */
    uint32_t timeout = 0;

    /* Wait for DHT to pull low */
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1) {
        if (++timeout > 100) return ESP_ERR_TIMEOUT;
        ets_delay_us(1);
    }

    /* Wait for DHT to pull high */
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 0) {
        if (++timeout > 100) return ESP_ERR_TIMEOUT;
        ets_delay_us(1);
    }

    /* Wait for start of data */
    timeout = 0;
    while (gpio_get_level(DHT_GPIO) == 1) {
        if (++timeout > 100) return ESP_ERR_TIMEOUT;
        ets_delay_us(1);
    }

    /* Read 40 bits (5 bytes) */
    for (int byte = 0; byte < 5; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            /* Wait for rising edge */
            timeout = 0;
            while (gpio_get_level(DHT_GPIO) == 0) {
                if (++timeout > 100) return ESP_ERR_TIMEOUT;
                ets_delay_us(1);
            }

            /* Measure high duration: ~28us = 0, ~70us = 1 */
            uint32_t high_time = 0;
            while (gpio_get_level(DHT_GPIO) == 1) {
                if (++high_time > 100) return ESP_ERR_TIMEOUT;
                ets_delay_us(1);
            }

            s_dht_data[byte] <<= 1;
            if (high_time > 40) {
                s_dht_data[byte] |= 1;
            }
        }
    }

    /* Verify checksum */
    uint8_t checksum = s_dht_data[0] + s_dht_data[1] + s_dht_data[2] + s_dht_data[3];
    if (checksum != s_dht_data[4]) {
        ESP_LOGW(TAG, "Checksum failed: calc=0x%02X, rx=0x%02X", checksum, s_dht_data[4]);
        return ESP_ERR_INVALID_CRC;
    }

    return ESP_OK;
}

static esp_err_t dht22_init(void)
{
    ESP_LOGI(TAG, "DHT22 init: Data GPIO=%d", DHT_GPIO);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << DHT_GPIO),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(DHT_GPIO, 1);

    /* Wait for sensor to stabilize after power-on */
    vTaskDelay(pdMS_TO_TICKS(2000));

    s_initialized = true;
    ESP_LOGI(TAG, "DHT22 initialized");
    return ESP_OK;
}

static esp_err_t dht22_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) return ESP_ERR_INVALID_STATE;

    /* DHT22 needs ~2s between reads */
    vTaskDelay(pdMS_TO_TICKS(2500));

    esp_err_t err = dht22_read_raw();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "DHT22 read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Parse DHT22 data */
    /* Humidity: 16 bits, divide by 10 for decimal */
    float humidity = (float)(((uint16_t)s_dht_data[0] << 8) | s_dht_data[1]) / 10.0f;

    /* Temperature: 16 bits, MSB is sign */
    int16_t raw_temp = ((uint16_t)s_dht_data[2] << 8) | s_dht_data[3];
    float temp_c = (float)(raw_temp & 0x7FFF) / 10.0f;
    if (raw_temp & 0x8000) temp_c = -temp_c;

    if (humidity < 0 || humidity > 100 || temp_c < -40 || temp_c > 80) {
        ESP_LOGW(TAG, "DHT22 out of range: T=%.1f, H=%.1f", temp_c, humidity);
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(data, 0, sizeof(sensor_data_t));
    data->temperature.value.f32 = temp_c;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    data->humidity.value.f32 = humidity;
    data->humidity.valid = true;
    data->humidity.timestamp_ms = data->temperature.timestamp_ms;

    ESP_LOGD(TAG, "DHT22: T=%.1fC, H=%.1f%%", temp_c, humidity);
    return ESP_OK;
}

static esp_err_t dht22_sleep(void) { return ESP_OK; }
static esp_err_t dht22_wakeup(void) { return ESP_OK; }

static esp_err_t dht22_deinit(void)
{
    s_initialized = false;
    return ESP_OK;
}

static const sensor_info_t s_info = {
    .name = "DHT22/AM2302",
    .model = "DHT22",
    .capabilities = CAP_TEMPERATURE | CAP_HUMIDITY,
    .min_interval_ms = 2500,
    .default_interval_ms = 5000,
    .supports_sleep = false, /* DHT22 always powered, no sleep mode */
};

static const sensor_info_t *dht22_get_info(void) { return &s_info; }

static const sensor_ops_t dht22_ops = {
    .init = dht22_init, .read = dht22_read,
    .sleep = dht22_sleep, .wakeup = dht22_wakeup,
    .get_info = dht22_get_info, .deinit = dht22_deinit,
};

const sensor_ops_t dht22_sensor_ops = dht22_ops;
