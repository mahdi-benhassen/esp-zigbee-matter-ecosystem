/**
 * @file jsn_sr04t_sensor.c
 * @brief JSN-SR04T Waterproof Ultrasonic Distance Sensor Driver (GPIO-based)
 *
 * Uses GPIO trigger/echo interface to measure distance via time-of-flight
 * of an ultrasonic pulse.
 *
 * Measurement sequence:
 *   1. Pull trigger pin LOW for 2 µs (clean idle state)
 *   2. Pull trigger pin HIGH for 12 µs (initiate burst)
 *   3. Pull trigger pin LOW
 *   4. Wait for echo pin to go HIGH (timeout 30 ms)
 *   5. Measure duration of echo HIGH pulse
 *   6. distance_cm = duration_us × 0.0343 / 2
 *
 * Valid measurement range: 20 cm – 600 cm.
 * Readings outside this range are rejected with ESP_ERR_INVALID_SIZE.
 *
 * Capabilities: CAP_DISTANCE
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "JSN_SR04T"

/*=============================================================================
 * KCONFIG DEFAULTS
 *============================================================================*/
#ifndef CONFIG_SENSOR_JSN_TRIG_PIN
#define CONFIG_SENSOR_JSN_TRIG_PIN 0
#endif
#ifndef CONFIG_SENSOR_JSN_ECHO_PIN
#define CONFIG_SENSOR_JSN_ECHO_PIN 1
#endif

/*=============================================================================
 * TIMING & RANGE CONSTANTS
 *============================================================================*/
#define JSN_TRIG_HOLD_US        12      /* Trigger pulse width (µs) */
#define JSN_ECHO_TIMEOUT_US     30000   /* 30 ms – max round-trip at 600 cm */
#define JSN_MIN_DISTANCE_CM     20.0f   /* Sensor spec minimum */
#define JSN_MAX_DISTANCE_CM     600.0f  /* Sensor spec maximum */
#define JSN_SPEED_OF_SOUND_CM   0.0343f /* cm per µs at ~20 °C */

/*=============================================================================
 * MODULE STATE
 *============================================================================*/
static bool s_initialized = false;

/*=============================================================================
 * INIT
 *============================================================================*/
static esp_err_t jsn_sr04t_init(void)
{
    ESP_LOGI(TAG, "JSN-SR04T init: TRIG=GPIO%d, ECHO=GPIO%d",
             CONFIG_SENSOR_JSN_TRIG_PIN, CONFIG_SENSOR_JSN_ECHO_PIN);

    /* ---- Configure trigger pin as output, default LOW ---- */
    gpio_config_t trig_cfg = {
        .pin_bit_mask  = (1ULL << CONFIG_SENSOR_JSN_TRIG_PIN),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&trig_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Trigger GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, 0);

    /* ---- Configure echo pin as input ---- */
    gpio_config_t echo_cfg = {
        .pin_bit_mask  = (1ULL << CONFIG_SENSOR_JSN_ECHO_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&echo_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Echo GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "JSN-SR04T initialized");
    return ESP_OK;
}

/*=============================================================================
 * READ
 *============================================================================*/
static esp_err_t jsn_sr04t_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* ---- 1. Generate trigger pulse: LOW→HIGH(12µs)→LOW ---- */
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, 1);
    esp_rom_delay_us(JSN_TRIG_HOLD_US);
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, 0);

    /* ---- 2. Wait for echo to go HIGH (start of return pulse) ---- */
    int64_t wait_start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)CONFIG_SENSOR_JSN_ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - wait_start) > JSN_ECHO_TIMEOUT_US) {
            ESP_LOGE(TAG, "Echo timeout: never went HIGH");
            return ESP_ERR_TIMEOUT;
        }
    }

    /* ---- 3. Measure echo HIGH duration ---- */
    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)CONFIG_SENSOR_JSN_ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - echo_start) > JSN_ECHO_TIMEOUT_US) {
            ESP_LOGE(TAG, "Echo timeout: never went LOW");
            return ESP_ERR_TIMEOUT;
        }
    }
    int64_t echo_end = esp_timer_get_time();

    int64_t duration_us = echo_end - echo_start;

    /* ---- 4. Convert time-of-flight → one-way distance (cm) ---- */
    float distance_cm = (float)duration_us * JSN_SPEED_OF_SOUND_CM / 2.0f;

    /* ---- 5. Validate against sensor spec range ---- */
    if (distance_cm < JSN_MIN_DISTANCE_CM || distance_cm > JSN_MAX_DISTANCE_CM) {
        ESP_LOGW(TAG, "Reading out of range: %.1f cm (valid: %.0f–%.0f cm)",
                 distance_cm, JSN_MIN_DISTANCE_CM, JSN_MAX_DISTANCE_CM);
        return ESP_ERR_INVALID_SIZE;
    }

    /* ---- 6. Populate output struct ---- */
    memset(data, 0, sizeof(sensor_data_t));
    data->distance.value.f32    = distance_cm;
    data->distance.valid        = true;
    data->distance.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGD(TAG, "JSN-SR04T: duration=%lld us, distance=%.1f cm",
             (long long)duration_us, distance_cm);
    return ESP_OK;
}

/*=============================================================================
 * SLEEP / WAKEUP
 *
 * The JSN-SR04T has no software low-power mode. To minimise quiescent current
 * the power-gating layer can cut VCC between reads.  The driver keeps its
 * GPIO configuration intact so wakeup is a no-op.
 *============================================================================*/
static esp_err_t jsn_sr04t_sleep(void)
{
    /* Drive trigger LOW – ensures no spurious pulses while idle */
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, 0);
    return ESP_OK;
}

static esp_err_t jsn_sr04t_wakeup(void)
{
    /* Nothing to do – sensor is always ready after power-on */
    return ESP_OK;
}

/*=============================================================================
 * DEINIT
 *============================================================================*/
static esp_err_t jsn_sr04t_deinit(void)
{
    /* Revert GPIOs to safe floating inputs to avoid back-powering */
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_JSN_TRIG_PIN, GPIO_FLOATING);
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_JSN_ECHO_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_JSN_ECHO_PIN, GPIO_FLOATING);

    s_initialized = false;
    ESP_LOGI(TAG, "JSN-SR04T de-initialized");
    return ESP_OK;
}

/*=============================================================================
 * METADATA
 *============================================================================*/
static const sensor_info_t s_info = {
    .name               = "JSN-SR04T Ultrasonic Distance",
    .model              = "JSN-SR04T",
    .capabilities       = CAP_DISTANCE,
    .min_interval_ms    = 100,        /* ~60 ms round-trip at 600 cm + margin */
    .default_interval_ms = 10000,
    .supports_sleep     = true,
};

static const sensor_info_t *jsn_sr04t_get_info(void) { return &s_info; }

/*=============================================================================
 * OPERATIONS TABLE & EXPORTED SYMBOL
 *============================================================================*/
static const sensor_ops_t jsn_sr04t_ops = {
    .init     = jsn_sr04t_init,
    .read     = jsn_sr04t_read,
    .sleep    = jsn_sr04t_sleep,
    .wakeup   = jsn_sr04t_wakeup,
    .get_info = jsn_sr04t_get_info,
    .deinit   = jsn_sr04t_deinit,
};

const sensor_ops_t jsn_sr04t_sensor_ops = jsn_sr04t_ops;
