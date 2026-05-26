/**
 * @file hx711_sensor.c
 * @brief HX711 Load Cell Amplifier Driver (2-Wire GPIO Serial)
 *
 * Reads a 24-bit ADC value from the HX711 using a bit-banged serial
 * protocol on two GPIOs (PD_SCK for clock, DOUT for data).
 *
 * Protocol:
 *   1. Wait for DOUT to go LOW (data ready, up to 200 ms timeout)
 *   2. Enter critical section (disable interrupts)
 *   3. Clock out 24 data bits, MSB first:
 *        - SCK HIGH → 1 µs delay → read DOUT → SCK LOW → 1 µs delay
 *   4. Send 25th clock pulse (selects Channel A, Gain 128 for next read)
 *   5. Exit critical section
 *   6. Sign-extend 24-bit two's-complement to 32-bit signed integer
 *
 * Weight conversion:
 *   weight_kg = (raw - offset) / cal_factor
 *   Default offset  = 8388608 (mid-range of 24-bit ADC)
 *   Default cal_factor = 23000.0 (must be calibrated per load cell)
 *
 * Sleep/wakeup:
 *   - Holding PD_SCK HIGH for > 60 µs puts the HX711 into power-down mode.
 *   - Pulling PD_SCK LOW wakes it up (requires ~400 µs settling).
 *
 * Capabilities: CAP_WEIGHT
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

#define TAG "HX711"

/*=============================================================================
 * KCONFIG DEFAULTS
 *============================================================================*/
#ifndef CONFIG_SENSOR_HX711_SCK_PIN
#define CONFIG_SENSOR_HX711_SCK_PIN 10
#endif
#ifndef CONFIG_SENSOR_HX711_DOUT_PIN
#define CONFIG_SENSOR_HX711_DOUT_PIN 11
#endif

/*=============================================================================
 * CALIBRATION DEFAULTS
 *
 * These values are hardware-specific and should be determined during
 * production calibration with known reference weights.
 *============================================================================*/
#define HX711_RAW_OFFSET     8388608    /* 2^23 – ADC mid-range zero point */
#define HX711_CAL_FACTOR     23000.0f   /* Counts per kg (load-cell specific) */

/*=============================================================================
 * TIMING
 *============================================================================*/
#define HX711_DOUT_TIMEOUT_MS  200      /* Max wait for data ready */
#define HX711_POWERDOWN_US     100      /* SCK HIGH duration to enter power-down */
#define HX711_WAKEUP_SETTLE_US 400      /* Settling time after wakeup */

/*=============================================================================
 * MODULE STATE
 *============================================================================*/
static bool s_initialized = false;
static portMUX_TYPE s_hx711_mux = portMUX_INITIALIZER_UNLOCKED;

/*=============================================================================
 * INIT
 *============================================================================*/
static esp_err_t hx711_init(void)
{
    ESP_LOGI(TAG, "HX711 init: SCK=GPIO%d, DOUT=GPIO%d",
             CONFIG_SENSOR_HX711_SCK_PIN, CONFIG_SENSOR_HX711_DOUT_PIN);

    /* ---- Configure PD_SCK as output, default LOW (active / awake) ---- */
    gpio_config_t sck_cfg = {
        .pin_bit_mask  = (1ULL << CONFIG_SENSOR_HX711_SCK_PIN),
        .mode          = GPIO_MODE_OUTPUT,
        .pull_up_en    = GPIO_PULLUP_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&sck_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SCK GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 0);

    /* ---- Configure DOUT as input with pull-up ---- */
    gpio_config_t dout_cfg = {
        .pin_bit_mask  = (1ULL << CONFIG_SENSOR_HX711_DOUT_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    err = gpio_config(&dout_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DOUT GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "HX711 initialized (Channel A, Gain 128)");
    return ESP_OK;
}

/*=============================================================================
 * RAW 24-BIT READ (critical section)
 *============================================================================*/
/**
 * @brief Read a single 24-bit sample from the HX711.
 *
 * The entire bit-bang sequence runs inside a FreeRTOS critical section to
 * guarantee sub-microsecond SCK timing is not disrupted by interrupts.
 *
 * @param[out] raw_val  Signed 32-bit result (sign-extended from 24-bit)
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if DOUT never goes LOW
 */
static esp_err_t hx711_read_raw(int32_t *raw_val)
{
    /* ---- Wait for DOUT LOW → data ready ---- */
    int timeout = HX711_DOUT_TIMEOUT_MS;
    while (gpio_get_level((gpio_num_t)CONFIG_SENSOR_HX711_DOUT_PIN) == 1 && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    }
    if (timeout <= 0) {
        ESP_LOGE(TAG, "HX711 DOUT data-ready timeout (%d ms)", HX711_DOUT_TIMEOUT_MS);
        return ESP_ERR_TIMEOUT;
    }

    uint32_t val = 0;
    portENTER_CRITICAL(&s_hx711_mux);

    /* ---- Clock out 24 data bits (MSB first) ---- */
    for (int i = 0; i < 24; i++) {
        gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        val <<= 1;
        if (gpio_get_level((gpio_num_t)CONFIG_SENSOR_HX711_DOUT_PIN)) {
            val |= 1;
        }
        gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
    }

    /* ---- 25th clock pulse: select Channel A, Gain 128 for next conversion ---- */
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 0);
    esp_rom_delay_us(1);

    portEXIT_CRITICAL(&s_hx711_mux);

    /* ---- Sign-extend 24-bit two's-complement → signed 32-bit ---- */
    if (val & 0x800000) {
        val |= 0xFF000000;
    }

    *raw_val = (int32_t)val;
    return ESP_OK;
}

/*=============================================================================
 * READ
 *============================================================================*/
static esp_err_t hx711_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    int32_t raw = 0;
    esp_err_t err = hx711_read_raw(&raw);
    if (err != ESP_OK) {
        return err;
    }

    /* ---- Convert raw ADC counts → weight in kilograms ---- */
    float weight_kg = (float)(raw - HX711_RAW_OFFSET) / HX711_CAL_FACTOR;

    /* ---- Populate output struct ---- */
    memset(data, 0, sizeof(sensor_data_t));
    data->weight.value.f32    = weight_kg;
    data->weight.valid        = true;
    data->weight.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGD(TAG, "HX711: raw=%ld, weight=%.3f kg", (long)raw, weight_kg);
    return ESP_OK;
}

/*=============================================================================
 * SLEEP
 *
 * Holding PD_SCK HIGH for > 60 µs puts the HX711 into power-down mode
 * (~1 µA quiescent current vs. ~1.5 mA active).
 *============================================================================*/
static esp_err_t hx711_sleep(void)
{
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 1);
    esp_rom_delay_us(HX711_POWERDOWN_US);
    ESP_LOGD(TAG, "HX711 entered power-down mode");
    return ESP_OK;
}

/*=============================================================================
 * WAKEUP
 *
 * Pulling PD_SCK LOW wakes the HX711. The datasheet recommends waiting
 * at least 400 µs for the internal oscillator to stabilise before the
 * first conversion.
 *============================================================================*/
static esp_err_t hx711_wakeup(void)
{
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 0);
    esp_rom_delay_us(HX711_WAKEUP_SETTLE_US);
    ESP_LOGD(TAG, "HX711 woken up");
    return ESP_OK;
}

/*=============================================================================
 * DEINIT
 *============================================================================*/
static esp_err_t hx711_deinit(void)
{
    /* Put HX711 into power-down before releasing GPIOs */
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, 1);
    esp_rom_delay_us(HX711_POWERDOWN_US);

    /* Revert GPIOs to safe floating inputs */
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_HX711_SCK_PIN, GPIO_FLOATING);
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_HX711_DOUT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_HX711_DOUT_PIN, GPIO_FLOATING);

    s_initialized = false;
    ESP_LOGI(TAG, "HX711 de-initialized");
    return ESP_OK;
}

/*=============================================================================
 * METADATA
 *============================================================================*/
static const sensor_info_t s_info = {
    .name               = "HX711 Load Cell",
    .model              = "HX711",
    .capabilities       = CAP_WEIGHT,
    .min_interval_ms    = 100,        /* HX711 runs at 10/80 SPS internally */
    .default_interval_ms = 10000,
    .supports_sleep     = true,
};

static const sensor_info_t *hx711_get_info(void) { return &s_info; }

/*=============================================================================
 * OPERATIONS TABLE & EXPORTED SYMBOL
 *============================================================================*/
static const sensor_ops_t hx711_ops = {
    .init     = hx711_init,
    .read     = hx711_read,
    .sleep    = hx711_sleep,
    .wakeup   = hx711_wakeup,
    .get_info = hx711_get_info,
    .deinit   = hx711_deinit,
};

const sensor_ops_t hx711_sensor_ops = hx711_ops;
