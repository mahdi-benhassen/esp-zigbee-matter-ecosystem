/**
 * @file bh1750_sensor.c
 * @brief ROHM BH1750FVI Ambient Light Sensor Driver
 *
 * I2C interface. 16-bit digital output, 1-65535 lux range.
 * Resolution: 1 lux in High Resolution Mode.
 * Address: 0x23 (ADDR=LOW) or 0x5C (ADDR=HIGH)
 *
 * This driver uses One-Time High Resolution Mode (opcode 0x20). In this mode
 * the sensor performs a single measurement and then automatically returns to
 * power-down state, minimizing current consumption between reads.
 *
 * Conversion time: typ. 120ms, max 180ms (high resolution mode).
 * Lux formula: lux = raw_value / 1.2
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

#define TAG "BH1750"

/*=============================================================================
 * BH1750 I2C ADDRESS & PORT
 *============================================================================*/
#define BH1750_I2C_ADDR             0x23    /* ADDR pin = LOW (GND) */
#define BH1750_I2C_PORT             0

/*=============================================================================
 * BH1750 INSTRUCTION SET (opcodes)
 *
 * The BH1750 uses single-byte commands. No register addresses — each
 * I2C write is an instruction opcode.
 *============================================================================*/
#define BH1750_CMD_POWER_DOWN       0x00    /* Enter power-down state */
#define BH1750_CMD_POWER_ON         0x01    /* Exit power-down state (waiting for measurement) */
#define BH1750_CMD_RESET            0x07    /* Reset data register (only in power-on state) */

/* Continuous measurement modes */
#define BH1750_CMD_CONT_HRES        0x10    /* Continuous H-Res mode (1 lx, 120ms) */
#define BH1750_CMD_CONT_HRES2       0x11    /* Continuous H-Res mode 2 (0.5 lx, 120ms) */
#define BH1750_CMD_CONT_LRES        0x13    /* Continuous L-Res mode (4 lx, 16ms) */

/* One-time measurement modes (auto power-down after measurement) */
#define BH1750_CMD_OT_HRES          0x20    /* One-Time H-Res mode (1 lx, 120ms) */
#define BH1750_CMD_OT_HRES2         0x21    /* One-Time H-Res mode 2 (0.5 lx, 120ms) */
#define BH1750_CMD_OT_LRES          0x23    /* One-Time L-Res mode (4 lx, 16ms) */

/* Timing */
#define BH1750_MEASURE_DELAY_MS     180     /* Max conversion time for H-Res mode */

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
static i2c_master_dev_handle_t s_bh1750_dev = NULL;
static bool s_initialized = false;

/*=============================================================================
 * LOW-LEVEL I2C HELPERS
 *============================================================================*/

/**
 * @brief Send a single-byte instruction opcode to the BH1750
 *
 * The BH1750 uses opcode-based commands rather than register writes.
 * Each command is a single byte transmitted as an I2C write.
 *
 * @param cmd BH1750 instruction opcode
 * @return ESP_OK on success
 */
static esp_err_t bh1750_send_cmd(uint8_t cmd)
{
    return i2c_master_transmit(s_bh1750_dev, &cmd, 1, -1);
}

/*=============================================================================
 * SENSOR_OPS_T INTERFACE IMPLEMENTATION
 *============================================================================*/

/**
 * @brief Initialize BH1750 sensor
 *
 * Creates the I2C bus and device handle, powers on the sensor, resets the
 * data register, then powers down to await measurement requests.
 *
 * @return ESP_OK on success
 */
static esp_err_t bh1750_init(void)
{
    ESP_LOGI(TAG, "BH1750 init: SDA=GPIO%d, SCL=GPIO%d",
             CONFIG_SENSOR_I2C_SDA_PIN, CONFIG_SENSOR_I2C_SCL_PIN);

    i2c_master_bus_config_t bus_config = {
        .i2c_port = BH1750_I2C_PORT,
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
        .device_address = BH1750_I2C_ADDR,
        .scl_speed_hz = CONFIG_SENSOR_I2C_FREQ_HZ,
    };

    err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_bh1750_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add BH1750 device: %s", esp_err_to_name(err));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    /*
     * Initialization sequence:
     *   1. Power on (required before reset can be issued)
     *   2. Reset data register to clear any stale measurement
     *   3. Power down to save current until first read
     */
    err = bh1750_send_cmd(BH1750_CMD_POWER_ON);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BH1750 power on failed — device not responding");
        i2c_master_bus_rm_device(s_bh1750_dev);
        s_bh1750_dev = NULL;
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return err;
    }

    /* Small delay for power-on to take effect */
    vTaskDelay(pdMS_TO_TICKS(1));

    /* Reset clears the data register value (only works when powered on) */
    err = bh1750_send_cmd(BH1750_CMD_RESET);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "BH1750 reset failed: %s", esp_err_to_name(err));
        /* Non-fatal — continue initialization */
    }

    /* Power down to minimize current draw until first measurement */
    bh1750_send_cmd(BH1750_CMD_POWER_DOWN);

    s_initialized = true;
    ESP_LOGI(TAG, "BH1750FVI initialized at address 0x%02X", BH1750_I2C_ADDR);
    return ESP_OK;
}

/**
 * @brief Read illuminance from BH1750
 *
 * Triggers a one-time high-resolution measurement, waits for the conversion
 * to complete (max 180ms), reads the 16-bit raw value, and converts to lux
 * using the formula: lux = raw / 1.2
 *
 * After measurement, the sensor automatically returns to power-down state.
 *
 * @param[out] data Sensor data structure to fill
 * @return ESP_OK on success
 */
static esp_err_t bh1750_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Trigger one-time high resolution measurement (0x20).
     * The sensor will power on, perform the measurement, and automatically
     * return to power-down state when complete.
     */
    esp_err_t err = bh1750_send_cmd(BH1750_CMD_OT_HRES);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to trigger measurement: %s", esp_err_to_name(err));
        return err;
    }

    /* Wait for conversion to complete (max 180ms for high resolution mode) */
    vTaskDelay(pdMS_TO_TICKS(BH1750_MEASURE_DELAY_MS));

    /*
     * Read 2 bytes: MSB first, then LSB.
     * The BH1750 sends measurement data as a simple I2C read (no register address).
     */
    uint8_t rx_buf[2] = {0};
    err = i2c_master_receive(s_bh1750_dev, rx_buf, 2, -1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement: %s", esp_err_to_name(err));
        return err;
    }

    /* Assemble 16-bit raw value (MSB:LSB) */
    uint16_t raw_value = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];

    /*
     * Convert raw to lux:
     *   lux = raw_value / 1.2
     *
     * The 1.2 factor comes from the BH1750 datasheet measurement accuracy
     * specification for the H-resolution mode default sensitivity (MTreg=69).
     */
    float lux = (float)raw_value / 1.2f;

    /* Populate output structure */
    memset(data, 0, sizeof(sensor_data_t));
    data->illuminance.value.f32 = lux;
    data->illuminance.valid = true;
    data->illuminance.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGD(TAG, "BH1750: raw=%u, lux=%.1f", raw_value, lux);
    return ESP_OK;
}

/**
 * @brief Put BH1750 into power-down mode
 *
 * The sensor draws <1 µA in power-down state. In one-time measurement
 * mode the sensor already powers down after each read, but this provides
 * an explicit power-down for the sleep interface.
 *
 * @return ESP_OK on success
 */
static esp_err_t bh1750_sleep(void)
{
    if (!s_initialized) return ESP_OK;
    return bh1750_send_cmd(BH1750_CMD_POWER_DOWN);
}

/**
 * @brief Wake BH1750 from power-down mode
 *
 * Powers on the sensor so it is ready to accept measurement commands.
 * Note: one-time measurement mode (used in read) implicitly powers on
 * the sensor, so this is primarily for interface completeness.
 *
 * @return ESP_OK on success
 */
static esp_err_t bh1750_wakeup(void)
{
    if (!s_initialized) return ESP_OK;
    return bh1750_send_cmd(BH1750_CMD_POWER_ON);
}

/**
 * @brief Deinitialize BH1750 and release I2C resources
 * @return ESP_OK on success
 */
static esp_err_t bh1750_deinit(void)
{
    /* Power down before releasing hardware */
    if (s_bh1750_dev) {
        bh1750_send_cmd(BH1750_CMD_POWER_DOWN);
        i2c_master_bus_rm_device(s_bh1750_dev);
        s_bh1750_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    s_initialized = false;
    ESP_LOGI(TAG, "BH1750 deinitialized");
    return ESP_OK;
}

/*=============================================================================
 * SENSOR INFO & OPS EXPORT
 *============================================================================*/

static const sensor_info_t s_info = {
    .name = "ROHM BH1750FVI",
    .model = "BH1750FVI",
    .capabilities = CAP_ILLUMINANCE,
    .min_interval_ms = 200,
    .default_interval_ms = 5000,
    .supports_sleep = true,
};

static const sensor_info_t *bh1750_get_info(void) { return &s_info; }

static const sensor_ops_t bh1750_ops = {
    .init     = bh1750_init,
    .read     = bh1750_read,
    .sleep    = bh1750_sleep,
    .wakeup   = bh1750_wakeup,
    .get_info = bh1750_get_info,
    .deinit   = bh1750_deinit,
};

const sensor_ops_t bh1750_sensor_ops = bh1750_ops;
