/**
 * @file scd41_sensor.c
 * @brief Sensirion SCD41 CO2/Temperature/Humidity Sensor Driver (Plugin)
 *
 * I2C interface in single-shot low-power mode.
 * Address: 0x62 (fixed)
 *
 * Protocol flow:
 *   1. Wake up (0x36F6), wait 20ms
 *   2. Measure single-shot (0x219D), wait 5000ms
 *   3. Read measurement (0xEC05): 9 bytes = 3 words × (2 data + 1 CRC)
 *   4. Power down (0x36E5) for sleep
 *
 * CRC-8: polynomial 0x31, init 0xFF (Sensirion standard)
 *
 * Capabilities: CAP_CO2
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "SCD41"

/* ── I2C address ────────────────────────────────────────────────────────── */
#define SCD41_I2C_ADDR          0x62
#define SCD41_I2C_PORT          0

/* ── SCD41 commands (16-bit, MSB first on the wire) ─────────────────────── */
#define SCD41_CMD_WAKE_UP           0x36F6  /* Exit idle / power-down state     */
#define SCD41_CMD_MEASURE_SINGLE    0x219D  /* Trigger single-shot measurement  */
#define SCD41_CMD_READ_MEASUREMENT  0xEC05  /* Read CO2 + T + RH result         */
#define SCD41_CMD_POWER_DOWN        0x36E5  /* Enter ultra-low-power idle state */
#define SCD41_CMD_REINIT            0x3646  /* Re-initialise sensor             */
#define SCD41_CMD_GET_SERIAL        0x3682  /* Read unique 48-bit serial number */

/* ── Timing ─────────────────────────────────────────────────────────────── */
#define SCD41_WAKEUP_DELAY_MS       30      /* Datasheet: 20 ms typ, margin     */
#define SCD41_SINGLE_SHOT_DELAY_MS  5000    /* Single-shot conversion time      */
#define SCD41_READ_DELAY_MS         5       /* Time between cmd and data fetch  */

/* ── Shared I2C bus pin defaults (overridden by Kconfig) ────────────────── */
#ifndef CONFIG_SENSOR_I2C_SDA_PIN
#define CONFIG_SENSOR_I2C_SDA_PIN   6
#endif
#ifndef CONFIG_SENSOR_I2C_SCL_PIN
#define CONFIG_SENSOR_I2C_SCL_PIN   7
#endif
#ifndef CONFIG_SENSOR_I2C_FREQ_HZ
#define CONFIG_SENSOR_I2C_FREQ_HZ   100000
#endif

/* ── Module state ───────────────────────────────────────────────────────── */
static i2c_master_bus_handle_t s_i2c_bus   = NULL;
static i2c_master_dev_handle_t s_scd41_dev = NULL;
static bool s_initialized = false;

/* ========================================================================
 *  CRC-8 (Sensirion)
 *  Polynomial: x^8 + x^5 + x^4 + 1  →  0x31
 *  Initialisation: 0xFF
 *  Computed over each 2-byte data word; result compared to 3rd byte.
 * ======================================================================== */
static uint8_t scd41_crc8(const uint8_t *data, uint8_t len)
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

/* ========================================================================
 *  Low-level: send a 16-bit command to the SCD41
 * ======================================================================== */
static esp_err_t scd41_send_cmd(uint16_t cmd)
{
    uint8_t tx_buf[2] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF)
    };
    return i2c_master_transmit(s_scd41_dev, tx_buf, 2, -1);
}

/* ========================================================================
 *  init()  – Create the I2C bus & device handle, wake the sensor
 * ======================================================================== */
static esp_err_t scd41_init(void)
{
    ESP_LOGI(TAG, "SCD41 init using shared I2C bus");

    /* ── Get shared I2C master bus ──────────────────────────────────────── */
    s_i2c_bus = sensor_registry_get_i2c_bus();
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "Failed to get shared I2C bus");
        return ESP_FAIL;
    }

    /* ── Add the SCD41 device on the bus ────────────────────────────────── */
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SCD41_I2C_ADDR,
        .scl_speed_hz    = CONFIG_SENSOR_I2C_FREQ_HZ,
    };

    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_scd41_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SCD41 device: %s", esp_err_to_name(err));
        s_i2c_bus = NULL;
        return err;
    }

    /* ── Wake up the sensor (in case it was powered-down) ───────────────── */
    scd41_send_cmd(SCD41_CMD_WAKE_UP);
    vTaskDelay(pdMS_TO_TICKS(SCD41_WAKEUP_DELAY_MS));

    /* ── Re-initialise to reset any stale state ─────────────────────────── */
    err = scd41_send_cmd(SCD41_CMD_REINIT);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SCD41 reinit cmd failed (may still work): %s",
                 esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(SCD41_WAKEUP_DELAY_MS));

    s_initialized = true;
    ESP_LOGI(TAG, "SCD41 initialised (single-shot mode)");
    return ESP_OK;
}

/* ========================================================================
 *  read()  – Trigger a single-shot measurement, read 9 bytes, validate CRC
 *
 *  Response layout (9 bytes):
 *    [0..1] CO2  data   [2] CRC
 *    [3..4] Temp data   [5] CRC
 *    [6..7] RH   data   [8] CRC
 *
 *  CO2  = raw_co2  (ppm, unsigned 16-bit, direct)
 *  Temp = -45 + 175 × raw_temp / 65536   (°C)
 *  RH   = 100 × raw_hum / 65536          (%)
 *
 *  Only CAP_CO2 is exported; temp/RH are logged but not stored.
 * ======================================================================== */
static esp_err_t scd41_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err;

    /* 1. Wake up (idempotent if already awake) */
    err = scd41_send_cmd(SCD41_CMD_WAKE_UP);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wake-up failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(SCD41_WAKEUP_DELAY_MS));

    /* 2. Trigger single-shot measurement */
    err = scd41_send_cmd(SCD41_CMD_MEASURE_SINGLE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Single-shot trigger failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 3. Wait for conversion (~5 seconds) */
    vTaskDelay(pdMS_TO_TICKS(SCD41_SINGLE_SHOT_DELAY_MS));

    /* 4. Send read-measurement command */
    err = scd41_send_cmd(SCD41_CMD_READ_MEASUREMENT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Read-measurement cmd failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(SCD41_READ_DELAY_MS));

    /* 5. Read 9 response bytes */
    uint8_t rx_buf[9] = {0};
    err = i2c_master_receive(s_scd41_dev, rx_buf, sizeof(rx_buf), -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 6. Validate CRC on each 2-byte word */
    if (scd41_crc8(&rx_buf[0], 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "CO2 word CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (scd41_crc8(&rx_buf[3], 2) != rx_buf[5]) {
        ESP_LOGE(TAG, "Temperature word CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (scd41_crc8(&rx_buf[6], 2) != rx_buf[8]) {
        ESP_LOGE(TAG, "Humidity word CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    /* 7. Decode raw values */
    uint16_t raw_co2  = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    uint16_t raw_temp = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
    uint16_t raw_hum  = ((uint16_t)rx_buf[6] << 8) | rx_buf[7];

    float co2_ppm     = (float)raw_co2;
    float temperature = -45.0f + 175.0f * (float)raw_temp / 65536.0f;
    float humidity    = 100.0f * (float)raw_hum / 65536.0f;

    /* 8. Populate output struct */
    memset(data, 0, sizeof(sensor_data_t));

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    data->co2.value.f32     = co2_ppm;
    data->co2.valid         = true;
    data->co2.timestamp_ms  = now_ms;

    ESP_LOGD(TAG, "SCD41: CO2=%.0f ppm, T=%.2f°C, RH=%.1f%%",
             co2_ppm, temperature, humidity);
    return ESP_OK;
}

/* ========================================================================
 *  sleep()  – Send power-down command to enter ultra-low-power idle
 * ======================================================================== */
static esp_err_t scd41_sleep(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = scd41_send_cmd(SCD41_CMD_POWER_DOWN);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Power-down command failed: %s", esp_err_to_name(err));
    }
    ESP_LOGD(TAG, "SCD41 powered down");
    return err;
}

/* ========================================================================
 *  wakeup()  – Send wake-up command and wait for the sensor to be ready
 * ======================================================================== */
static esp_err_t scd41_wakeup(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = scd41_send_cmd(SCD41_CMD_WAKE_UP);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Wake-up command failed: %s", esp_err_to_name(err));
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(SCD41_WAKEUP_DELAY_MS));
    ESP_LOGD(TAG, "SCD41 woken up");
    return ESP_OK;
}

/* ========================================================================
 *  deinit()  – Remove I2C device, tear down bus
 * ======================================================================== */
static esp_err_t scd41_deinit(void)
{
    if (s_initialized) {
        /* Try to power-down the sensor gracefully before removing the bus */
        scd41_send_cmd(SCD41_CMD_POWER_DOWN);
    }

    if (s_scd41_dev) {
        i2c_master_bus_rm_device(s_scd41_dev);
        s_scd41_dev = NULL;
    }
    s_i2c_bus = NULL;

    s_initialized = false;
    ESP_LOGI(TAG, "SCD41 deinitialised");
    return ESP_OK;
}

/* ========================================================================
 *  Sensor metadata
 * ======================================================================== */
static const sensor_info_t s_info = {
    .name               = "Sensirion SCD41 CO2",
    .model              = "SCD41",
    .capabilities       = CAP_CO2,
    .min_interval_ms    = 5000,       /* Single-shot takes ~5 s itself */
    .default_interval_ms = 30000,
    .supports_sleep     = true,
};

static const sensor_info_t *scd41_get_info(void) { return &s_info; }

/* ========================================================================
 *  Exported operations vtable
 * ======================================================================== */
static const sensor_ops_t scd41_ops = {
    .init     = scd41_init,
    .read     = scd41_read,
    .sleep    = scd41_sleep,
    .wakeup   = scd41_wakeup,
    .get_info = scd41_get_info,
    .deinit   = scd41_deinit,
};

const sensor_ops_t scd41_sensor_ops = scd41_ops;
