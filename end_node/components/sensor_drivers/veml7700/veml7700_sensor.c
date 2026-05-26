/**
 * @file veml7700_sensor.c
 * @brief Vishay VEML7700 High-Accuracy Ambient Light Sensor Driver
 *
 * I2C interface. 16-bit resolution, 0.0036 to 120,000 lux range (with gain
 * and integration time adjustments).
 * Address: 0x10 (fixed, no address pin)
 *
 * This driver configures the VEML7700 with:
 *   - Gain: x1 (ALS_GAIN = 01)
 *   - Integration time: 100ms (ALS_IT = 0000)
 *   - Power saving mode: PSM mode 1 (enabled to reduce current)
 *
 * Lux conversion at gain=1, IT=100ms:
 *   lux = raw * 0.0576
 *
 * For high-lux accuracy (>1000 lux), the Vishay application note recommends
 * a polynomial correction:
 *   lux_corrected = 6.0135e-13 * raw^4 - 9.3924e-9 * raw^3
 *                 + 8.1488e-5 * raw^2 + 1.0023 * raw
 *
 * This driver applies the simple linear formula for efficiency and applies
 * the non-linearity correction when the computed lux exceeds 1000.
 *
 * Capabilities: ILLUMINANCE
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

#define TAG "VEML7700"

/*=============================================================================
 * VEML7700 I2C ADDRESS & PORT
 *============================================================================*/
#define VEML7700_I2C_ADDR           0x10    /* Fixed address, no ADDR pin */
#define VEML7700_I2C_PORT           0

/*=============================================================================
 * VEML7700 COMMAND REGISTER MAP
 *
 * The VEML7700 uses 16-bit command codes. Each register is accessed by
 * writing a command byte followed by 2 data bytes (LSB first).
 *============================================================================*/
#define VEML7700_REG_ALS_CONF       0x00    /* ALS configuration register */
#define VEML7700_REG_ALS_WH         0x01    /* ALS high threshold window */
#define VEML7700_REG_ALS_WL         0x02    /* ALS low threshold window */
#define VEML7700_REG_PSM            0x03    /* Power saving mode */
#define VEML7700_REG_ALS_DATA       0x04    /* ALS output data (16-bit raw count) */
#define VEML7700_REG_WHITE_DATA     0x05    /* White channel output data */
#define VEML7700_REG_ALS_INT        0x06    /* Interrupt status */

/*=============================================================================
 * ALS_CONF REGISTER FIELD DEFINITIONS (register 0x00)
 *
 * Bits [12:11] : ALS_GAIN   — Gain setting
 *     00 = x1,  01 = x2,  10 = x(1/8),  11 = x(1/4)
 * Bits [9:6]   : ALS_IT     — Integration time
 *     1100 = 25ms, 1000 = 50ms, 0000 = 100ms, 0001 = 200ms,
 *     0010 = 400ms, 0011 = 800ms
 * Bits [5:4]   : ALS_PERS   — ALS persistence protect number
 *     00 = 1, 01 = 2, 10 = 4, 11 = 8
 * Bit  [1]     : ALS_INT_EN — Interrupt enable (0 = disable, 1 = enable)
 * Bit  [0]     : ALS_SD     — Shut down (0 = power on, 1 = shut down)
 *============================================================================*/

/* Gain settings (bits 12:11) */
#define VEML7700_GAIN_1             (0x00 << 11)    /* x1 gain */
#define VEML7700_GAIN_2             (0x01 << 11)    /* x2 gain */
#define VEML7700_GAIN_1_8           (0x02 << 11)    /* x(1/8) gain */
#define VEML7700_GAIN_1_4           (0x03 << 11)    /* x(1/4) gain */

/* Integration time settings (bits 9:6) */
#define VEML7700_IT_25MS            (0x0C << 6)     /* 25ms */
#define VEML7700_IT_50MS            (0x08 << 6)     /* 50ms */
#define VEML7700_IT_100MS           (0x00 << 6)     /* 100ms */
#define VEML7700_IT_200MS           (0x01 << 6)     /* 200ms */
#define VEML7700_IT_400MS           (0x02 << 6)     /* 400ms */
#define VEML7700_IT_800MS           (0x03 << 6)     /* 800ms */

/* Power state (bit 0) */
#define VEML7700_ALS_SD_ON          0x0000  /* Power on (clear SD bit) */
#define VEML7700_ALS_SD_OFF         0x0001  /* Shut down (set SD bit) */

/*=============================================================================
 * PSM REGISTER FIELD DEFINITIONS (register 0x03)
 *
 * Bits [2:1] : PSM mode
 *     00 = mode 1, 01 = mode 2, 10 = mode 3, 11 = mode 4
 * Bit  [0]   : PSM_EN (0 = disable, 1 = enable)
 *============================================================================*/
#define VEML7700_PSM_MODE_1         (0x00 << 1)     /* Refresh: IT + 500ms */
#define VEML7700_PSM_MODE_2         (0x01 << 1)     /* Refresh: IT + 1000ms */
#define VEML7700_PSM_MODE_3         (0x02 << 1)     /* Refresh: IT + 2000ms */
#define VEML7700_PSM_MODE_4         (0x03 << 1)     /* Refresh: IT + 4000ms */
#define VEML7700_PSM_ENABLE         0x01

/* Default configuration: Gain x1, IT 100ms, no interrupt, power on */
#define VEML7700_DEFAULT_CONF       (VEML7700_GAIN_1 | VEML7700_IT_100MS | VEML7700_ALS_SD_ON)

/*
 * Lux resolution at Gain=1, IT=100ms:
 *   Resolution = 0.0576 lux/count (from VEML7700 datasheet Table 1)
 */
#define VEML7700_LUX_RESOLUTION     0.0576f

/* Timing: wait at least one integration period + margin after power-on */
#define VEML7700_STARTUP_DELAY_MS   150     /* 100ms IT + 50ms margin */

/*=============================================================================
 * I2C PIN DEFAULTS (overridden by Kconfig)
 *============================================================================*/
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
static i2c_master_dev_handle_t s_veml7700_dev = NULL;
static bool s_initialized = false;

/*=============================================================================
 * LOW-LEVEL I2C REGISTER ACCESS
 *
 * The VEML7700 uses 16-bit little-endian register values. All writes are
 * 3 bytes (command code + data LSB + data MSB) and all reads return
 * 2 bytes (data LSB + data MSB) after sending the command code.
 *============================================================================*/

/**
 * @brief Write a 16-bit value to a VEML7700 register
 * @param reg   Register/command code (e.g., 0x00 for ALS_CONF)
 * @param value 16-bit value to write (sent as LSB, MSB)
 * @return ESP_OK on success
 */
static esp_err_t veml7700_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {
        reg,
        (uint8_t)(value & 0xFF),        /* LSB first */
        (uint8_t)((value >> 8) & 0xFF)  /* MSB second */
    };
    return i2c_master_transmit(s_veml7700_dev, buf, 3, -1);
}

/**
 * @brief Read a 16-bit value from a VEML7700 register
 * @param reg    Register/command code
 * @param value  Pointer to store the 16-bit result
 * @return ESP_OK on success
 */
static esp_err_t veml7700_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t rx_buf[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(s_veml7700_dev, &reg, 1, rx_buf, 2, -1);
    if (err != ESP_OK) {
        return err;
    }
    /* Reassemble 16-bit value (LSB first) */
    *value = ((uint16_t)rx_buf[1] << 8) | rx_buf[0];
    return ESP_OK;
}

/*=============================================================================
 * NON-LINEARITY CORRECTION
 *
 * The VEML7700 output becomes non-linear above ~1000 lux. Vishay's
 * application note (AN84323) provides a 4th-order polynomial correction:
 *
 *   lux_corrected = 6.0135e-13 * lux^4 - 9.3924e-9 * lux^3
 *                 + 8.1488e-5 * lux^2 + 1.0023 * lux
 *
 * This is only applied when the linear lux value exceeds the threshold.
 *============================================================================*/
#define VEML7700_NONLINEAR_THRESHOLD 1000.0f

/**
 * @brief Apply non-linearity correction for high-lux readings
 *
 * Uses the Vishay AN84323 polynomial correction formula. Only called
 * when the linear lux reading exceeds 1000 lux.
 *
 * @param lux_linear Linear lux value (raw * resolution)
 * @return Corrected lux value
 */
static float veml7700_correct_nonlinearity(float lux_linear)
{
    /*
     * Polynomial coefficients from Vishay Application Note AN84323:
     *   y = 6.0135e-13 * x^4 - 9.3924e-9 * x^3 + 8.1488e-5 * x^2 + 1.0023 * x
     *
     * Using Horner's method for numerical stability and efficiency:
     *   y = x * (1.0023 + x * (8.1488e-5 + x * (-9.3924e-9 + x * 6.0135e-13)))
     */
    float x = lux_linear;
    float corrected = x * (1.0023f + x * (8.1488e-5f + x * (-9.3924e-9f + x * 6.0135e-13f)));
    return corrected;
}

/*=============================================================================
 * SENSOR_OPS_T INTERFACE IMPLEMENTATION
 *============================================================================*/

/**
 * @brief Initialize VEML7700 sensor
 *
 * Creates the I2C bus and device handle, configures the sensor with
 * gain x1, integration time 100ms, and power saving mode 1.
 *
 * @return ESP_OK on success
 */
static esp_err_t veml7700_init(void)
{
    ESP_LOGI(TAG, "VEML7700 init using shared I2C bus");

    s_i2c_bus = sensor_registry_get_i2c_bus();
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus");
        return ESP_FAIL;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VEML7700_I2C_ADDR,
        .scl_speed_hz = CONFIG_SENSOR_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_veml7700_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add VEML7700 device: %s", esp_err_to_name(err));
        s_i2c_bus = NULL;
        return err;
    }

    /*
     * Configure ALS:
     *   - Gain x1 (suitable for 0-3775 lux; extend with gain 1/8 if needed)
     *   - Integration time 100ms (good balance of resolution and speed)
     *   - No interrupt
     *   - Power on (SD bit = 0)
     */
    err = veml7700_write_reg(VEML7700_REG_ALS_CONF, VEML7700_DEFAULT_CONF);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "VEML7700 ALS_CONF write failed — device not responding");
        i2c_master_bus_rm_device(s_veml7700_dev);
        s_veml7700_dev = NULL;
        s_i2c_bus = NULL;
        return err;
    }

    /*
     * Enable Power Saving Mode 1:
     *   - Reduces current consumption by cycling the sensor
     *   - Refresh period = IT + 500ms (mode 1) = 600ms total
     */
    err = veml7700_write_reg(VEML7700_REG_PSM,
                             VEML7700_PSM_MODE_1 | VEML7700_PSM_ENABLE);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "PSM write failed: %s (non-fatal)", esp_err_to_name(err));
        /* PSM failure is non-fatal — sensor works without it, just draws more current */
    }

    /* Wait for at least one integration cycle to complete before first read */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_STARTUP_DELAY_MS));

    s_initialized = true;
    ESP_LOGI(TAG, "VEML7700 initialized at address 0x%02X (gain=x1, IT=100ms, PSM=1)",
             VEML7700_I2C_ADDR);
    return ESP_OK;
}

/**
 * @brief Read illuminance from VEML7700
 *
 * Reads the 16-bit ALS output register, converts to lux using the
 * resolution factor for the configured gain and integration time,
 * and applies non-linearity correction for high-lux readings.
 *
 * @param[out] data Sensor data structure to fill
 * @return ESP_OK on success
 */
static esp_err_t veml7700_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Read 16-bit ALS output register (0x04) */
    uint16_t raw_als = 0;
    esp_err_t err = veml7700_read_reg(VEML7700_REG_ALS_DATA, &raw_als);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ALS data: %s", esp_err_to_name(err));
        return err;
    }

    /*
     * Convert raw count to lux:
     *   At gain=1, IT=100ms: resolution = 0.0576 lux/count
     *   Maximum measurable lux = 65535 * 0.0576 = ~3775 lux
     */
    float lux = (float)raw_als * VEML7700_LUX_RESOLUTION;

    /*
     * Apply non-linearity correction for high lux values.
     * Below 1000 lux the linear formula is sufficiently accurate.
     */
    if (lux > VEML7700_NONLINEAR_THRESHOLD) {
        lux = veml7700_correct_nonlinearity(lux);
    }

    /* Clamp to non-negative (should never be negative, but be safe) */
    if (lux < 0.0f) lux = 0.0f;

    /* Populate output structure */
    memset(data, 0, sizeof(sensor_data_t));
    data->illuminance.value.f32 = lux;
    data->illuminance.valid = true;
    data->illuminance.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGD(TAG, "VEML7700: raw=%u, lux=%.2f", raw_als, lux);
    return ESP_OK;
}

/**
 * @brief Put VEML7700 into shutdown mode
 *
 * Sets the ALS_SD bit in ALS_CONF to shut down the sensor.
 * Current consumption drops to ~0.5 µA in shutdown.
 *
 * @return ESP_OK on success
 */
static esp_err_t veml7700_sleep(void)
{
    if (!s_initialized) return ESP_OK;

    /* Set SD bit (bit 0) while preserving other configuration bits */
    uint16_t conf = VEML7700_DEFAULT_CONF | VEML7700_ALS_SD_OFF;
    return veml7700_write_reg(VEML7700_REG_ALS_CONF, conf);
}

/**
 * @brief Wake VEML7700 from shutdown mode
 *
 * Clears the ALS_SD bit to power on the sensor. A startup delay of
 * at least one integration period is needed before the first valid
 * reading, but this is handled by the read interval.
 *
 * @return ESP_OK on success
 */
static esp_err_t veml7700_wakeup(void)
{
    if (!s_initialized) return ESP_OK;

    /* Clear SD bit (bit 0) — restore default operating configuration */
    esp_err_t err = veml7700_write_reg(VEML7700_REG_ALS_CONF, VEML7700_DEFAULT_CONF);
    if (err != ESP_OK) return err;

    /*
     * After wakeup, allow at least one integration cycle for a valid reading.
     * The caller should wait before calling read(), but we add a minimal
     * delay here to avoid reading stale data on an immediate read.
     */
    vTaskDelay(pdMS_TO_TICKS(VEML7700_STARTUP_DELAY_MS));
    return ESP_OK;
}

/**
 * @brief Deinitialize VEML7700 and release I2C resources
 * @return ESP_OK on success
 */
static esp_err_t veml7700_deinit(void)
{
    /* Shut down the sensor before releasing the bus */
    if (s_veml7700_dev) {
        veml7700_write_reg(VEML7700_REG_ALS_CONF,
                           VEML7700_DEFAULT_CONF | VEML7700_ALS_SD_OFF);
        i2c_master_bus_rm_device(s_veml7700_dev);
        s_veml7700_dev = NULL;
    }
    s_i2c_bus = NULL;
    s_initialized = false;
    ESP_LOGI(TAG, "VEML7700 deinitialized");
    return ESP_OK;
}

/*=============================================================================
 * SENSOR INFO & OPS EXPORT
 *============================================================================*/

static const sensor_info_t s_info = {
    .name = "Vishay VEML7700",
    .model = "VEML7700",
    .capabilities = CAP_ILLUMINANCE,
    .min_interval_ms = 600,     /* IT(100ms) + PSM mode 1 refresh (500ms) */
    .default_interval_ms = 5000,
    .supports_sleep = true,
};

static const sensor_info_t *veml7700_get_info(void) { return &s_info; }

static const sensor_ops_t veml7700_ops = {
    .init     = veml7700_init,
    .read     = veml7700_read,
    .sleep    = veml7700_sleep,
    .wakeup   = veml7700_wakeup,
    .get_info = veml7700_get_info,
    .deinit   = veml7700_deinit,
};

const sensor_ops_t veml7700_sensor_ops = veml7700_ops;
