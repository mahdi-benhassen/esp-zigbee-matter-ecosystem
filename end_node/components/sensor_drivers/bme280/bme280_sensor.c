/**
 * @file bme280_sensor.c
 * @brief Bosch BME280 Temperature/Humidity/Pressure Sensor Driver
 *
 * I2C interface. Combined digital humidity, pressure, and temperature sensor.
 * Accuracy: +/- 1.0C temperature, +/- 3% RH humidity, +/- 1 hPa pressure.
 * Address: 0x76 (SDO=GND) or 0x77 (SDO=VCC)
 *
 * Implements the sensor_ops_t plugin interface for the universal sensor
 * registry. Uses forced mode with 1x oversampling for all measurements.
 * Calibration data is stored in RTC_DATA_ATTR to survive deep sleep.
 *
 * Compensation formulas follow Bosch BME280 datasheet rev 1.6, Section 4.2.3.
 *
 * Capabilities: TEMPERATURE + HUMIDITY + PRESSURE
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

#define TAG "BME280"

/*=============================================================================
 * BME280 I2C ADDRESSES & PORT
 *============================================================================*/
#define BME280_I2C_ADDR_PRIMARY     0x76    /* SDO pin connected to GND */
#define BME280_I2C_ADDR_SECONDARY   0x77    /* SDO pin connected to VCC */
#define BME280_I2C_PORT             0

/*=============================================================================
 * BME280 REGISTER MAP
 *============================================================================*/
#define BME280_REG_CHIP_ID          0xD0
#define BME280_REG_RESET            0xE0
#define BME280_REG_CTRL_HUM         0xF2    /* Humidity oversampling (must write before ctrl_meas) */
#define BME280_REG_STATUS           0xF3
#define BME280_REG_CTRL_MEAS        0xF4    /* Temperature/Pressure oversampling + mode */
#define BME280_REG_CONFIG           0xF5    /* Standby, filter, SPI3w */

/* Calibration data register ranges */
#define BME280_REG_CALIB_T_P_START  0x88    /* dig_T1..dig_P9: 26 bytes (0x88..0xA1) */
#define BME280_REG_CALIB_H1         0xA1    /* dig_H1: 1 byte */
#define BME280_REG_CALIB_H2_START   0xE1    /* dig_H2..dig_H6: 7 bytes (0xE1..0xE7) */

/* Raw data registers */
#define BME280_REG_DATA_START       0xF7    /* press_msb through hum_lsb: 8 bytes */

/* Expected chip ID */
#define BME280_CHIP_ID_VALUE        0x60

/* Mode definitions */
#define BME280_MODE_SLEEP           0x00
#define BME280_MODE_FORCED          0x01    /* Single measurement then return to sleep */
#define BME280_MODE_NORMAL          0x03

/* Oversampling definitions (bits 7:5 = osrs_t, bits 4:2 = osrs_p in ctrl_meas) */
#define BME280_OSRS_T_1X            (0x01 << 5) /* Temperature oversampling 1x */
#define BME280_OSRS_P_1X            (0x01 << 2) /* Pressure oversampling 1x */
#define BME280_OSRS_H_1X            0x01        /* Humidity oversampling 1x */

/* Timing: forced mode measurement time with 1x oversampling for T+P+H */
#define BME280_MEASURE_DELAY_MS     10

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
 * BME280 CALIBRATION DATA STRUCTURE
 *
 * Stored in RTC slow memory so calibration survives deep sleep cycles.
 * This avoids re-reading calibration registers after every wakeup.
 *============================================================================*/
typedef struct {
    /* Temperature compensation (unsigned/signed per Bosch datasheet) */
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    /* Pressure compensation */
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;

    /* Humidity compensation (mixed sizes per Bosch datasheet) */
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;    /* 12-bit signed, stored across two registers */
    int16_t  dig_H5;    /* 12-bit signed, stored across two registers */
    int8_t   dig_H6;

    bool     valid;     /* Set true after successful calibration read */
} bme280_calib_t;

/* Retain calibration across deep sleep in RTC slow memory */
static RTC_DATA_ATTR bme280_calib_t s_calib = { .valid = false };

/*=============================================================================
 * MODULE STATE
 *============================================================================*/
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_bme280_dev = NULL;
static bool s_initialized = false;
static uint8_t s_active_addr = BME280_I2C_ADDR_PRIMARY;

/*=============================================================================
 * LOW-LEVEL I2C REGISTER ACCESS
 *============================================================================*/

/**
 * @brief Write a single byte to a BME280 register
 * @param reg Register address
 * @param val Value to write
 * @return ESP_OK on success
 */
static esp_err_t bme280_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_bme280_dev, buf, 2, -1);
}

/**
 * @brief Read a contiguous block of registers from the BME280
 * @param reg  Starting register address
 * @param data Buffer to receive data
 * @param len  Number of bytes to read
 * @return ESP_OK on success
 */
static esp_err_t bme280_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_bme280_dev, &reg, 1, data, len, -1);
}

/*=============================================================================
 * CALIBRATION DATA READ
 *
 * The BME280 stores factory calibration in two separate register banks:
 *   Bank 1: 0x88..0xA1 (26 bytes) — temperature and pressure trimming
 *   Bank 2: 0xA1       (1 byte)   — humidity H1
 *   Bank 3: 0xE1..0xE7 (7 bytes)  — humidity H2..H6
 *
 * Several humidity values are packed across nibble boundaries.
 *============================================================================*/

/**
 * @brief Read all calibration/trimming data from BME280 registers
 *
 * Must be called once after chip ID verification. Results are stored
 * in the RTC-retained s_calib structure.
 *
 * @return ESP_OK on success
 */
static esp_err_t bme280_read_calibration(void)
{
    uint8_t buf[26];
    esp_err_t err;

    /*--- Bank 1: Temperature + Pressure (0x88..0xA1, 26 bytes) ---*/
    err = bme280_read_regs(BME280_REG_CALIB_T_P_START, buf, 26);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read T/P calibration: %s", esp_err_to_name(err));
        return err;
    }

    /* Temperature trimming: dig_T1 (unsigned), dig_T2, dig_T3 (signed) */
    s_calib.dig_T1 = (uint16_t)(buf[1]  << 8) | buf[0];
    s_calib.dig_T2 = (int16_t)((buf[3]  << 8) | buf[2]);
    s_calib.dig_T3 = (int16_t)((buf[5]  << 8) | buf[4]);

    /* Pressure trimming: dig_P1 (unsigned), dig_P2..dig_P9 (signed) */
    s_calib.dig_P1 = (uint16_t)(buf[7]  << 8) | buf[6];
    s_calib.dig_P2 = (int16_t)((buf[9]  << 8) | buf[8]);
    s_calib.dig_P3 = (int16_t)((buf[11] << 8) | buf[10]);
    s_calib.dig_P4 = (int16_t)((buf[13] << 8) | buf[12]);
    s_calib.dig_P5 = (int16_t)((buf[15] << 8) | buf[14]);
    s_calib.dig_P6 = (int16_t)((buf[17] << 8) | buf[16]);
    s_calib.dig_P7 = (int16_t)((buf[19] << 8) | buf[18]);
    s_calib.dig_P8 = (int16_t)((buf[21] << 8) | buf[20]);
    s_calib.dig_P9 = (int16_t)((buf[23] << 8) | buf[22]);

    /*--- Bank 2: Humidity H1 at 0xA1 (1 byte) ---*/
    uint8_t h1;
    err = bme280_read_regs(BME280_REG_CALIB_H1, &h1, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read H1 calibration: %s", esp_err_to_name(err));
        return err;
    }
    s_calib.dig_H1 = h1;

    /*--- Bank 3: Humidity H2..H6 at 0xE1..0xE7 (7 bytes) ---*/
    uint8_t hbuf[7];
    err = bme280_read_regs(BME280_REG_CALIB_H2_START, hbuf, 7);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read H2-H6 calibration: %s", esp_err_to_name(err));
        return err;
    }

    /* dig_H2: signed 16-bit little-endian at [E1:E2] */
    s_calib.dig_H2 = (int16_t)((hbuf[1] << 8) | hbuf[0]);

    /* dig_H3: unsigned 8-bit at [E3] */
    s_calib.dig_H3 = hbuf[2];

    /* dig_H4: 12-bit signed = [E4] bits 11:4, [E5] bits 3:0
     *   dig_H4 = (E4 << 4) | (E5 & 0x0F) */
    s_calib.dig_H4 = (int16_t)(((int16_t)hbuf[3] << 4) | (hbuf[4] & 0x0F));

    /* dig_H5: 12-bit signed = [E5] bits 7:4, [E6] bits 11:4
     *   dig_H5 = (E6 << 4) | (E5 >> 4) */
    s_calib.dig_H5 = (int16_t)(((int16_t)hbuf[5] << 4) | (hbuf[4] >> 4));

    /* dig_H6: signed 8-bit at [E7] */
    s_calib.dig_H6 = (int8_t)hbuf[6];

    s_calib.valid = true;

    ESP_LOGI(TAG, "Calibration loaded: T1=%u T2=%d T3=%d P1=%u H1=%u H2=%d",
             s_calib.dig_T1, s_calib.dig_T2, s_calib.dig_T3,
             s_calib.dig_P1, s_calib.dig_H1, s_calib.dig_H2);

    return ESP_OK;
}

/*=============================================================================
 * BOSCH COMPENSATION FORMULAS (from BME280 datasheet rev 1.6, Sec. 4.2.3)
 *
 * These are the official 32-bit integer compensation routines. t_fine is a
 * shared intermediate variable that links temperature to the pressure
 * and humidity compensations.
 *============================================================================*/

/** Global fine temperature used by pressure and humidity compensation */
static int32_t s_t_fine = 0;

/**
 * @brief Compensate raw temperature reading
 * @param adc_T 20-bit raw temperature from registers 0xFA..0xFC
 * @return Temperature in hundredths of a degree Celsius (e.g., 2345 = 23.45°C)
 */
static int32_t bme280_compensate_temperature(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) *
            ((int32_t)s_calib.dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12) *
            ((int32_t)s_calib.dig_T3)) >> 14;

    s_t_fine = var1 + var2;
    T = (s_t_fine * 5 + 128) >> 8;
    return T;  /* In hundredths of °C */
}

/**
 * @brief Compensate raw pressure reading
 *
 * Must be called AFTER bme280_compensate_temperature() so that s_t_fine
 * is valid.
 *
 * @param adc_P 20-bit raw pressure from registers 0xF7..0xF9
 * @return Pressure in Pa as unsigned 32-bit (divide by 256 for Pa, then /100 for hPa)
 */
static uint32_t bme280_compensate_pressure(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)s_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) +
           ((var1 * (int64_t)s_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;

    if (var1 == 0) {
        return 0;  /* Avoid division by zero */
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

    return (uint32_t)p;  /* Pressure in Pa * 256 */
}

/**
 * @brief Compensate raw humidity reading
 *
 * Must be called AFTER bme280_compensate_temperature() so that s_t_fine
 * is valid.
 *
 * @param adc_H 16-bit raw humidity from registers 0xFD..0xFE
 * @return Humidity in Q22.10 format (divide by 1024 to get %RH)
 */
static uint32_t bme280_compensate_humidity(int32_t adc_H)
{
    int32_t v_x1_u32r;

    v_x1_u32r = (s_t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)s_calib.dig_H4) << 20) -
                    (((int32_t)s_calib.dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)s_calib.dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)s_calib.dig_H3)) >> 11) +
                       ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) *
                   ((int32_t)s_calib.dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                                ((int32_t)s_calib.dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

    return (uint32_t)(v_x1_u32r >> 12);  /* Q22.10 format */
}

/*=============================================================================
 * SENSOR_OPS_T INTERFACE IMPLEMENTATION
 *============================================================================*/

/**
 * @brief Initialize BME280 sensor
 *
 * Creates the I2C bus and device handle, verifies chip ID (tries both
 * 0x76 and 0x77 addresses), reads calibration data, and configures
 * the sensor for forced mode with 1x oversampling.
 *
 * @return ESP_OK on success
 */
static esp_err_t bme280_init(void)
{
    ESP_LOGI(TAG, "BME280 init: SDA=GPIO%d, SCL=GPIO%d",
             CONFIG_SENSOR_I2C_SDA_PIN, CONFIG_SENSOR_I2C_SCL_PIN);

    /* Create the I2C master bus */
    i2c_master_bus_config_t bus_config = {
        .i2c_port = BME280_I2C_PORT,
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

    /*
     * Try primary address (0x76) first, then secondary (0x77).
     * The SDO pin determines which address the BME280 uses.
     */
    uint8_t chip_id = 0;
    uint8_t addrs[] = { BME280_I2C_ADDR_PRIMARY, BME280_I2C_ADDR_SECONDARY };

    for (int i = 0; i < 2; i++) {
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addrs[i],
            .scl_speed_hz = CONFIG_SENSOR_I2C_FREQ_HZ,
        };

        err = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_bme280_dev);
        if (err != ESP_OK) {
            continue;
        }

        /* Try reading the chip ID register */
        err = bme280_read_regs(BME280_REG_CHIP_ID, &chip_id, 1);
        if (err == ESP_OK && chip_id == BME280_CHIP_ID_VALUE) {
            s_active_addr = addrs[i];
            ESP_LOGI(TAG, "BME280 found at 0x%02X (chip_id=0x%02X)", addrs[i], chip_id);
            break;
        }

        /* Wrong chip or no response — remove and try next address */
        ESP_LOGW(TAG, "No BME280 at 0x%02X (chip_id=0x%02X, err=%s)",
                 addrs[i], chip_id, esp_err_to_name(err));
        i2c_master_bus_rm_device(s_bme280_dev);
        s_bme280_dev = NULL;
    }

    if (s_bme280_dev == NULL) {
        ESP_LOGE(TAG, "BME280 not found on either address");
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return ESP_ERR_NOT_FOUND;
    }

    /* Soft reset the sensor (write 0xB6 to reset register) */
    err = bme280_write_reg(BME280_REG_RESET, 0xB6);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Soft reset failed: %s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(5));  /* Wait for reset to complete */

    /* Read calibration data if not already retained from a previous deep sleep */
    if (!s_calib.valid) {
        err = bme280_read_calibration();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Calibration read failed");
            i2c_master_bus_rm_device(s_bme280_dev);
            s_bme280_dev = NULL;
            i2c_del_master_bus(s_i2c_bus);
            s_i2c_bus = NULL;
            return err;
        }
    } else {
        ESP_LOGI(TAG, "Using RTC-retained calibration data");
    }

    /*
     * Configure the sensor:
     *   1. Set humidity oversampling (ctrl_hum 0xF2) — MUST be written before ctrl_meas
     *   2. Set config register (filter off, standby 0.5ms, no SPI)
     *   3. Set ctrl_meas (temp/pressure oversampling + sleep mode)
     *
     * We keep the sensor in sleep mode; forced mode is triggered per-read.
     */
    err = bme280_write_reg(BME280_REG_CTRL_HUM, BME280_OSRS_H_1X);
    if (err != ESP_OK) return err;

    err = bme280_write_reg(BME280_REG_CONFIG, 0x00);  /* No filter, no standby, no SPI3w */
    if (err != ESP_OK) return err;

    /* Sleep mode initially — forced mode triggered in read() */
    err = bme280_write_reg(BME280_REG_CTRL_MEAS,
                           BME280_OSRS_T_1X | BME280_OSRS_P_1X | BME280_MODE_SLEEP);
    if (err != ESP_OK) return err;

    s_initialized = true;
    ESP_LOGI(TAG, "BME280 initialized on address 0x%02X", s_active_addr);
    return ESP_OK;
}

/**
 * @brief Read temperature, pressure, and humidity from BME280
 *
 * Triggers a forced-mode measurement, waits for completion, reads the
 * raw ADC values, then applies Bosch compensation formulas.
 *
 * @param[out] data Sensor data structure to fill
 * @return ESP_OK on success
 */
static esp_err_t bme280_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * Trigger forced-mode measurement:
     * Write ctrl_hum first (required by datasheet for changes to take effect),
     * then write ctrl_meas with forced mode bit set.
     */
    esp_err_t err = bme280_write_reg(BME280_REG_CTRL_HUM, BME280_OSRS_H_1X);
    if (err != ESP_OK) return err;

    err = bme280_write_reg(BME280_REG_CTRL_MEAS,
                           BME280_OSRS_T_1X | BME280_OSRS_P_1X | BME280_MODE_FORCED);
    if (err != ESP_OK) return err;

    /* Wait for measurement to complete */
    vTaskDelay(pdMS_TO_TICKS(BME280_MEASURE_DELAY_MS));

    /* Poll status register for measuring bit to clear (bit 3) */
    uint8_t status = 0;
    int retries = 10;
    do {
        err = bme280_read_regs(BME280_REG_STATUS, &status, 1);
        if (err != ESP_OK) return err;
        if (!(status & 0x08)) break;  /* Bit 3 = measuring */
        vTaskDelay(pdMS_TO_TICKS(2));
    } while (--retries > 0);

    if (status & 0x08) {
        ESP_LOGW(TAG, "BME280 measurement timeout");
        return ESP_ERR_TIMEOUT;
    }

    /*
     * Read all 8 data registers in a single burst:
     *   0xF7: press_msb
     *   0xF8: press_lsb
     *   0xF9: press_xlsb (bits 7:4)
     *   0xFA: temp_msb
     *   0xFB: temp_lsb
     *   0xFC: temp_xlsb (bits 7:4)
     *   0xFD: hum_msb
     *   0xFE: hum_lsb
     */
    uint8_t raw[8];
    err = bme280_read_regs(BME280_REG_DATA_START, raw, 8);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data registers: %s", esp_err_to_name(err));
        return err;
    }

    /* Assemble 20-bit raw pressure */
    int32_t adc_P = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | ((int32_t)raw[2] >> 4);

    /* Assemble 20-bit raw temperature */
    int32_t adc_T = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | ((int32_t)raw[5] >> 4);

    /* Assemble 16-bit raw humidity */
    int32_t adc_H = ((int32_t)raw[6] << 8) | (int32_t)raw[7];

    /* Apply Bosch compensation (temperature MUST be first — sets s_t_fine) */
    int32_t temp_raw = bme280_compensate_temperature(adc_T);
    float temp_c = (float)temp_raw / 100.0f;

    uint32_t press_raw = bme280_compensate_pressure(adc_P);
    float pressure_hpa = (float)press_raw / 256.0f / 100.0f;  /* Pa*256 -> Pa -> hPa */

    uint32_t hum_raw = bme280_compensate_humidity(adc_H);
    float humidity = (float)hum_raw / 1024.0f;  /* Q22.10 -> %RH */

    /* Clamp humidity to valid range */
    if (humidity < 0.0f) humidity = 0.0f;
    if (humidity > 100.0f) humidity = 100.0f;

    /* Populate output structure */
    memset(data, 0, sizeof(sensor_data_t));
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    data->temperature.value.f32 = temp_c;
    data->temperature.valid = true;
    data->temperature.timestamp_ms = now_ms;

    data->humidity.value.f32 = humidity;
    data->humidity.valid = true;
    data->humidity.timestamp_ms = now_ms;

    data->pressure.value.f32 = pressure_hpa;
    data->pressure.valid = true;
    data->pressure.timestamp_ms = now_ms;

    ESP_LOGD(TAG, "BME280: T=%.2f°C, H=%.1f%%, P=%.2f hPa",
             temp_c, humidity, pressure_hpa);

    return ESP_OK;
}

/**
 * @brief Put BME280 into sleep mode
 *
 * The BME280 automatically returns to sleep after a forced-mode measurement,
 * but this explicitly ensures sleep mode for consistent power management.
 *
 * @return ESP_OK on success
 */
static esp_err_t bme280_sleep(void)
{
    if (!s_initialized) return ESP_OK;

    /* Write sleep mode to ctrl_meas (keep oversampling settings) */
    return bme280_write_reg(BME280_REG_CTRL_MEAS,
                            BME280_OSRS_T_1X | BME280_OSRS_P_1X | BME280_MODE_SLEEP);
}

/**
 * @brief Wake BME280 from sleep mode
 *
 * The BME280 wakes automatically on any I2C transaction, and forced mode
 * is triggered per-read. This is effectively a no-op but included for
 * interface completeness.
 *
 * @return ESP_OK on success
 */
static esp_err_t bme280_wakeup(void)
{
    /* No explicit wakeup needed — forced mode triggers in read() */
    return ESP_OK;
}

/**
 * @brief Deinitialize BME280 and release I2C resources
 * @return ESP_OK on success
 */
static esp_err_t bme280_deinit(void)
{
    if (s_bme280_dev) {
        i2c_master_bus_rm_device(s_bme280_dev);
        s_bme280_dev = NULL;
    }
    if (s_i2c_bus) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    s_initialized = false;
    ESP_LOGI(TAG, "BME280 deinitialized");
    return ESP_OK;
}

/*=============================================================================
 * SENSOR INFO & OPS EXPORT
 *============================================================================*/

static const sensor_info_t s_info = {
    .name = "Bosch BME280",
    .model = "BME280",
    .capabilities = CAP_TEMPERATURE | CAP_HUMIDITY | CAP_PRESSURE,
    .min_interval_ms = 2000,
    .default_interval_ms = 10000,
    .supports_sleep = true,
};

static const sensor_info_t *bme280_get_info(void) { return &s_info; }

static const sensor_ops_t bme280_ops = {
    .init     = bme280_init,
    .read     = bme280_read,
    .sleep    = bme280_sleep,
    .wakeup   = bme280_wakeup,
    .get_info = bme280_get_info,
    .deinit   = bme280_deinit,
};

const sensor_ops_t bme280_sensor_ops = bme280_ops;
