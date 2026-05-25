#include "sensor_hub.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "SENSOR_HUB";

/* Shared I2C resources */
static i2c_master_bus_handle_t s_i2c_bus = NULL;
#if CONFIG_ENABLE_BME280
static i2c_master_dev_handle_t s_bme280_dev = NULL;
#endif
#if CONFIG_ENABLE_BH1750
static i2c_master_dev_handle_t s_bh1750_dev = NULL;
#endif
#if CONFIG_ENABLE_SCD41
static i2c_master_dev_handle_t s_scd41_dev = NULL;
#endif

/* ADC Oneshot resources */
#if CONFIG_ENABLE_SOIL_MOISTURE
static adc_oneshot_unit_handle_t s_adc1_handle = NULL;
static adc_cali_handle_t s_adc1_cali_handle = NULL;
static bool s_adc_cali_enabled = false;
#endif

/* Calibration structures for BME280 */
#if CONFIG_ENABLE_BME280
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
    int32_t  t_fine;
} bme280_calib_t;

RTC_DATA_ATTR static bme280_calib_t s_bme280_calib;
RTC_DATA_ATTR static bool s_bme280_calib_valid = false;
#endif

/*=============================================================================
 * LOW LEVEL HELPER FUNCTIONS
 *============================================================================*/

#if CONFIG_ENABLE_SCD41 || CONFIG_ENABLE_DS18B20
static uint8_t crc8_dvb_s2(uint8_t crc, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
#endif

#if CONFIG_ENABLE_DS18B20
static uint8_t ds18b20_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) {
                crc ^= 0x8C;
            }
            inbyte >>= 1;
        }
    }
    return crc;
}
#endif

/*=============================================================================
 * POWER GATING IMPLEMENTATION
 *============================================================================*/
void sensor_hub_power_on(void)
{
    ESP_LOGI(TAG, "Power-Gating ON: Energizing sensor VCC rail (GPIO %d)", SENSOR_POWER_CTRL_GPIO);
    /* For high-side P-MOSFET, pull GPIO LOW to conduct. For active-high load switch, pull HIGH. */
    /* We assume active-high configuration here; adjust level logic if using P-MOSFET */
    gpio_set_level((gpio_num_t)SENSOR_POWER_CTRL_GPIO, 1);
}

void sensor_hub_power_off(void)
{
    ESP_LOGI(TAG, "Power-Gating OFF: De-energizing sensor VCC rail");
    gpio_set_level((gpio_num_t)SENSOR_POWER_CTRL_GPIO, 0);
}

/*=============================================================================
 * DS18B20 BIT-BANGED 1-WIRE PLATFORM INDEPENDENT DRIVER (RISC-V COMPATIBLE)
 *============================================================================*/
#if CONFIG_ENABLE_DS18B20
static inline void ds18b20_write_low(void)
{
    gpio_set_direction((gpio_num_t)DS18B20_1WIRE_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)DS18B20_1WIRE_GPIO, 0);
}

static inline void ds18b20_release(void)
{
    gpio_set_direction((gpio_num_t)DS18B20_1WIRE_GPIO, GPIO_MODE_INPUT);
}

static inline int ds18b20_read_pin(void)
{
    return gpio_get_level((gpio_num_t)DS18B20_1WIRE_GPIO);
}

static bool ds18b20_reset(void)
{
    bool presence = false;
    ds18b20_write_low();
    esp_rom_delay_us(480);
    ds18b20_release();
    esp_rom_delay_us(70);
    presence = (ds18b20_read_pin() == 0);
    esp_rom_delay_us(410);
    return presence;
}

static void ds18b20_write_bit(bool bit)
{
    if (bit) {
        ds18b20_write_low();
        esp_rom_delay_us(6);
        ds18b20_release();
        esp_rom_delay_us(64);
    } else {
        ds18b20_write_low();
        esp_rom_delay_us(60);
        ds18b20_release();
        esp_rom_delay_us(10);
    }
}

static bool ds18b20_read_bit(void)
{
    bool bit = false;
    ds18b20_write_low();
    esp_rom_delay_us(3);
    ds18b20_release();
    esp_rom_delay_us(10);
    bit = (ds18b20_read_pin() != 0);
    esp_rom_delay_us(57);
    return bit;
}

static void ds18b20_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t ds18b20_read_byte(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (ds18b20_read_bit()) {
            data |= (1 << i);
        }
    }
    return data;
}
#endif

/*=============================================================================
 * BME280 DRIVER IMPLEMENTATION
 *============================================================================*/
#if CONFIG_ENABLE_BME280
static esp_err_t bme280_read_regs(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_bme280_dev, &reg_addr, 1, data, len, -1);
}

static esp_err_t bme280_write_reg(uint8_t reg_addr, uint8_t val)
{
    uint8_t tx_buf[2] = {reg_addr, val};
    return i2c_master_transmit(s_bme280_dev, tx_buf, 2, -1);
}

static esp_err_t bme280_init_sensor(void)
{
    if (s_bme280_calib_valid) {
        ESP_LOGI(TAG, "BME280: Using RTC retained calibration parameters");
        return ESP_OK;
    }

    uint8_t id = 0;
    esp_err_t err = bme280_read_regs(0xD0, &id, 1);
    if (err != ESP_OK || id != 0x60) {
        ESP_LOGE(TAG, "BME280 connection check failed (ID: 0x%02X, Err: %s)", id, esp_err_to_name(err));
        return (err == ESP_OK) ? ESP_FAIL : err;
    }

    /* Soft Reset */
    bme280_write_reg(0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Read Calibration parameters */
    uint8_t calib1[26];
    err = bme280_read_regs(0x88, calib1, 26);
    if (err != ESP_OK) return err;

    s_bme280_calib.dig_T1 = (calib1[1] << 8) | calib1[0];
    s_bme280_calib.dig_T2 = (int16_t)((calib1[3] << 8) | calib1[2]);
    s_bme280_calib.dig_T3 = (int16_t)((calib1[5] << 8) | calib1[4]);
    s_bme280_calib.dig_P1 = (calib1[7] << 8) | calib1[6];
    s_bme280_calib.dig_P2 = (int16_t)((calib1[9] << 8) | calib1[8]);
    s_bme280_calib.dig_P3 = (int16_t)((calib1[11] << 8) | calib1[10]);
    s_bme280_calib.dig_P4 = (int16_t)((calib1[13] << 8) | calib1[12]);
    s_bme280_calib.dig_P5 = (int16_t)((calib1[15] << 8) | calib1[14]);
    s_bme280_calib.dig_P6 = (int16_t)((calib1[17] << 8) | calib1[16]);
    s_bme280_calib.dig_P7 = (int16_t)((calib1[19] << 8) | calib1[18]);
    s_bme280_calib.dig_P8 = (int16_t)((calib1[21] << 8) | calib1[20]);
    s_bme280_calib.dig_P9 = (int16_t)((calib1[23] << 8) | calib1[22]);
    s_bme280_calib.dig_H1 = calib1[25];

    uint8_t calib2[7];
    err = bme280_read_regs(0xE1, calib2, 7);
    if (err != ESP_OK) return err;

    s_bme280_calib.dig_H2 = (int16_t)((calib2[1] << 8) | calib2[0]);
    s_bme280_calib.dig_H3 = calib2[2];
    s_bme280_calib.dig_H4 = (int16_t)((calib2[3] << 4) | (calib2[4] & 0x0F));
    s_bme280_calib.dig_H5 = (int16_t)((calib2[5] << 4) | ((calib2[4] >> 4) & 0x0F));
    s_bme280_calib.dig_H6 = (int8_t)calib2[6];

    s_bme280_calib_valid = true;
    ESP_LOGI(TAG, "BME280 calibration data loaded successfully");
    return ESP_OK;
}

static esp_err_t bme280_read_data(float *temp, float *humidity, float *press)
{
    /* Set sensor configuration */
    /* Humidity oversampling = x1 */
    esp_err_t err = bme280_write_reg(0xF2, 0x01);
    if (err != ESP_OK) return err;

    /* Temp oversampling = x1, Press oversampling = x1, forced mode = 1 */
    err = bme280_write_reg(0xF4, (1 << 5) | (1 << 2) | 0x01);
    if (err != ESP_OK) return err;

    /* Wait for measurement to complete (usually < 10ms for 1x oversampling) */
    uint8_t status = 0;
    int timeout = 50;
    do {
        vTaskDelay(pdMS_TO_TICKS(2));
        err = bme280_read_regs(0xF3, &status, 1);
        timeout--;
    } while (err == ESP_OK && (status & 0x09) && timeout > 0);

    if (timeout <= 0 || err != ESP_OK) {
        ESP_LOGE(TAG, "BME280 measurement timeout or read error: %s", esp_err_to_name(err));
        return ESP_ERR_TIMEOUT;
    }

    /* Read burst data: Press (3 bytes) + Temp (3 bytes) + Hum (2 bytes) */
    uint8_t raw[8];
    err = bme280_read_regs(0xF7, raw, 8);
    if (err != ESP_OK) return err;

    int32_t adc_P = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);
    int32_t adc_T = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);
    int32_t adc_H = (raw[6] << 8) | raw[7];

    /* Temperature compensation */
    double var1 = (((double)adc_T) / 16384.0 - ((double)s_bme280_calib.dig_T1) / 1024.0) * ((double)s_bme280_calib.dig_T2);
    double var2 = ((((double)adc_T) / 131072.0 - ((double)s_bme280_calib.dig_T1) / 8192.0) *
                   (((double)adc_T) / 131072.0 - ((double)s_bme280_calib.dig_T1) / 8192.0)) * ((double)s_bme280_calib.dig_T3);
    s_bme280_calib.t_fine = (int32_t)(var1 + var2);
    *temp = (float)((var1 + var2) / 5120.0);

    /* Pressure compensation */
    var1 = ((double)s_bme280_calib.t_fine / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)s_bme280_calib.dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)s_bme280_calib.dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)s_bme280_calib.dig_P4) * 65536.0);
    var1 = (((double)s_bme280_calib.dig_P3) * var1 * var1 / 524288.0 + ((double)s_bme280_calib.dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)s_bme280_calib.dig_P1);
    if (var1 != 0.0) {
        double p = 1048576.0 - (double)adc_P;
        p = (((p - (var2 / 4096.0)) * 6250.0) / var1);
        var1 = ((double)s_bme280_calib.dig_P9) * p * p / 2147483648.0;
        var2 = p * ((double)s_bme280_calib.dig_P8) / 32768.0;
        *press = (float)((p + (var1 + var2 + ((double)s_bme280_calib.dig_P7)) / 16.0) / 100.0);
    } else {
        *press = 0.0f;
    }

    /* Humidity compensation */
    double h = ((double)s_bme280_calib.t_fine - 76800.0);
    h = (((double)adc_H) - (((double)s_bme280_calib.dig_H4) * 64.0 + ((double)s_bme280_calib.dig_H5) / 16384.0 * h)) *
        (((double)s_bme280_calib.dig_H2) / 65536.0 * (1.0 + ((double)s_bme280_calib.dig_H6) / 67108864.0 * h *
        (1.0 + ((double)s_bme280_calib.dig_H3) / 67108864.0 * h)));
    h = h * (1.0 - ((double)s_bme280_calib.dig_H1) * h / 524288.0);
    if (h > 100.0) h = 100.0;
    else if (h < 0.0) h = 0.0;
    *humidity = (float)h;

    return ESP_OK;
}
#endif

/*=============================================================================
 * BH1750 DRIVER IMPLEMENTATION
 *============================================================================*/
#if CONFIG_ENABLE_BH1750
static esp_err_t bh1750_read_lux(float *lux)
{
    /* Send One-Time High Resolution Mode instruction (0x20) */
    uint8_t cmd = 0x20;
    esp_err_t err = i2c_master_transmit(s_bh1750_dev, &cmd, 1, -1);
    if (err != ESP_OK) return err;

    /* High resolution mode requires max 180ms conversion delay */
    vTaskDelay(pdMS_TO_TICKS(180));

    uint8_t raw[2];
    err = i2c_master_receive(s_bh1750_dev, raw, 2, -1);
    if (err != ESP_OK) return err;

    uint16_t level = (raw[0] << 8) | raw[1];
    *lux = (float)level / 1.2f;
    return ESP_OK;
}
#endif

/*=============================================================================
 * SCD41 DRIVER IMPLEMENTATION (SINGLE SHOT LOW POWER WORKFLOW)
 *============================================================================*/
#if CONFIG_ENABLE_SCD41
static esp_err_t scd41_send_cmd(uint16_t cmd)
{
    uint8_t tx_buf[2] = {(uint8_t)(cmd >> 8), (uint8_t)(cmd & 0xFF)};
    return i2c_master_transmit(s_scd41_dev, tx_buf, 2, -1);
}

static esp_err_t scd41_read_measurement(float *co2, float *temp, float *humidity)
{
    /* Read measurement command (0xEC05) */
    esp_err_t err = scd41_send_cmd(0xEC05);
    if (err != ESP_OK) return err;

    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t rx_buf[9];
    err = i2c_master_receive(s_scd41_dev, rx_buf, 9, -1);
    if (err != ESP_OK) return err;

    /* Validate CRCs */
    if (crc8_dvb_s2(0xFF, rx_buf, 2) != rx_buf[2]) {
        ESP_LOGE(TAG, "SCD41 CO2 CRC Mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (crc8_dvb_s2(0xFF, &rx_buf[3], 2) != rx_buf[5]) {
        ESP_LOGE(TAG, "SCD41 Temp CRC Mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (crc8_dvb_s2(0xFF, &rx_buf[6], 2) != rx_buf[8]) {
        ESP_LOGE(TAG, "SCD41 Hum CRC Mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_co2 = (rx_buf[0] << 8) | rx_buf[1];
    uint16_t raw_temp = (rx_buf[3] << 8) | rx_buf[4];
    uint16_t raw_hum = (rx_buf[6] << 8) | rx_buf[7];

    *co2 = (float)raw_co2;
    *temp = -45.0f + 175.0f * (float)raw_temp / 65536.0f;
    *humidity = 100.0f * (float)raw_hum / 65536.0f;

    return ESP_OK;
}
#endif

/*=============================================================================
 * WINSEN ZE03-NH3 DRIVER IMPLEMENTATION (Q&A TRANSACTIONS)
 *============================================================================*/
#if CONFIG_ENABLE_ZE03_NH3
static esp_err_t ze03_read_nh3(float *nh3_ppm)
{
    /* Q&A Mode Read Command Packet */
    uint8_t cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
    uart_write_bytes(ZE03_UART_PORT, (const char *)cmd, 9);

    /* Read Response Packet (9 bytes) */
    uint8_t rx[9];
    int len = uart_read_bytes(ZE03_UART_PORT, rx, 9, pdMS_TO_TICKS(1000));
    if (len < 9) {
        ESP_LOGE(TAG, "ZE03 UART Timeout / incomplete read (len=%d)", len);
        return ESP_ERR_TIMEOUT;
    }

    /* Checksum verification */
    uint8_t sum = 0;
    for (int i = 1; i < 8; i++) {
        sum += rx[i];
    }
    uint8_t checksum = (~sum) + 1;
    if (checksum != rx[8]) {
        ESP_LOGE(TAG, "ZE03 Checksum Mismatch (Expected: 0x%02X, Got: 0x%02X)", checksum, rx[8]);
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t conc = (rx[2] << 8) | rx[3];
    *nh3_ppm = (float)conc;
    return ESP_OK;
}
#endif

/*=============================================================================
 * JSN-SR04T ULTRASONIC DEPTH DRIVER IMPLEMENTATION
 *============================================================================*/
#if CONFIG_ENABLE_JSN_SR04T
static esp_err_t jsn_read_distance(float *distance_cm)
{
    /* Drive Trig High for 12 microseconds */
    gpio_set_level((gpio_num_t)JSN_TRIG_GPIO, 0);
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)JSN_TRIG_GPIO, 1);
    esp_rom_delay_us(12);
    gpio_set_level((gpio_num_t)JSN_TRIG_GPIO, 0);

    /* Wait for Echo to go High */
    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level((gpio_num_t)JSN_ECHO_GPIO) == 0) {
        if ((esp_timer_get_time() - start_wait) > 30000) { /* 30ms timeout */
            ESP_LOGE(TAG, "JSN Echo Signal timeout (never went HIGH)");
            return ESP_ERR_TIMEOUT;
        }
    }

    int64_t start_echo = esp_timer_get_time();
    /* Wait for Echo to go Low */
    while (gpio_get_level((gpio_num_t)JSN_ECHO_GPIO) == 1) {
        if ((esp_timer_get_time() - start_echo) > 30000) { /* 30ms timeout */
            ESP_LOGE(TAG, "JSN Echo Signal timeout (never went LOW)");
            return ESP_ERR_TIMEOUT;
        }
    }
    int64_t end_echo = esp_timer_get_time();

    int64_t duration = end_echo - start_echo;
    /* Distance = (duration * speed of sound) / 2 */
    /* Speed of sound = 343 m/s = 0.0343 cm/us */
    *distance_cm = (float)duration * 0.0343f / 2.0f;

    /* Clamp distance to sensor specifications (20cm - 600cm) */
    if (*distance_cm < 20.0f || *distance_cm > 600.0f) {
        ESP_LOGW(TAG, "JSN Reading out of range: %.1f cm", *distance_cm);
        return ESP_ERR_INVALID_SIZE;
    }

    return ESP_OK;
}
#endif

/*=============================================================================
 * HX711 2-WIRE SERIAL LOAD CELL DRIVER IMPLEMENTATION
 *============================================================================*/
#if CONFIG_ENABLE_HX711
static esp_err_t hx711_read_raw(int32_t *raw_val)
{
    /* Wait for DOUT to go LOW (data ready indication) */
    int timeout = 200;
    while (gpio_get_level((gpio_num_t)HX711_DOUT_GPIO) == 1 && timeout > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
        timeout--;
    }

    if (timeout <= 0) {
        ESP_LOGE(TAG, "HX711 DOUT Data Ready timeout");
        return ESP_ERR_TIMEOUT;
    }

    uint32_t val = 0;
    /* Read 24 bits */
    for (int i = 0; i < 24; i++) {
        gpio_set_level((gpio_num_t)HX711_PD_SCK_GPIO, 1);
        esp_rom_delay_us(1);
        val <<= 1;
        if (gpio_get_level((gpio_num_t)HX711_DOUT_GPIO)) {
            val |= 1;
        }
        gpio_set_level((gpio_num_t)HX711_PD_SCK_GPIO, 0);
        esp_rom_delay_us(1);
    }

    /* 25th pulse sets channel A gain 128 for the next conversion */
    gpio_set_level((gpio_num_t)HX711_PD_SCK_GPIO, 1);
    esp_rom_delay_us(1);
    gpio_set_level((gpio_num_t)HX711_PD_SCK_GPIO, 0);
    esp_rom_delay_us(1);

    /* Sign extend 24-bit to signed 32-bit */
    if (val & 0x800000) {
        val |= 0xFF000000;
    }

    *raw_val = (int32_t)val;
    return ESP_OK;
}
#endif

/*=============================================================================
 * SOIL MOISTURE ADC DRIVER IMPLEMENTATION
 *============================================================================*/
#if CONFIG_ENABLE_SOIL_MOISTURE
static esp_err_t soil_moisture_read_vwc(float *vwc_percent)
{
    int raw_val = 0;
    esp_err_t err = adc_oneshot_read(s_adc1_handle, (adc_channel_t)SOIL_MOISTURE_ADC_CHANNEL, &raw_val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC Soil Moisture Read Failed: %s", esp_err_to_name(err));
        return err;
    }

    int voltage = 0;
    if (s_adc_cali_enabled) {
        adc_cali_raw_to_voltage(s_adc1_cali_handle, raw_val, &voltage);
    } else {
        /* Simple linear approximation of voltage: V = raw * 3300 / 4095 */
        voltage = (raw_val * 3300) / 4095;
    }

    /* Calibration limits for Capacitive Moisture Sensor v1.2 */
    /* Dry voltage: ~2500mV, Wet voltage: ~1100mV */
    const float dry_voltage = 2500.0f;
    const float wet_voltage = 1100.0f;

    float vwc = 100.0f * (dry_voltage - (float)voltage) / (dry_voltage - wet_voltage);
    if (vwc > 100.0f) vwc = 100.0f;
    if (vwc < 0.0f) vwc = 0.0f;

    *vwc_percent = vwc;
    return ESP_OK;
}
#endif

/*=============================================================================
 * MAIN INTERFACE IMPLEMENTATION
 *============================================================================*/
esp_err_t sensor_hub_init(void)
{
    ESP_LOGI(TAG, "Initializing hardware interfaces...");

    /* 1. Configure Power Control GPIO Pin */
    gpio_config_t pwr_cfg = {
        .pin_bit_mask = (1ULL << SENSOR_POWER_CTRL_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&pwr_cfg);
    gpio_set_level((gpio_num_t)SENSOR_POWER_CTRL_GPIO, 0); /* Keep powered down initially */

    /* 2. Configure I2C Shared Bus */
#if CONFIG_ENABLE_BME280 || CONFIG_ENABLE_BH1750 || CONFIG_ENABLE_SCD41
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = SENSOR_I2C_PORT,
        .sda_io_num = SENSOR_I2C_SDA_GPIO,
        .scl_io_num = SENSOR_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create shared I2C bus: %s", esp_err_to_name(err));
        return err;
    }
#endif

    /* 3. Configure individual I2C devices */
#if CONFIG_ENABLE_BME280
    i2c_device_config_t bme280_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x76,
        .scl_speed_hz = SENSOR_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &bme280_cfg, &s_bme280_dev));
#endif

#if CONFIG_ENABLE_BH1750
    i2c_device_config_t bh1750_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x23,
        .scl_speed_hz = SENSOR_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &bh1750_cfg, &s_bh1750_dev));
#endif

#if CONFIG_ENABLE_SCD41
    i2c_device_config_t scd41_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x62,
        .scl_speed_hz = SENSOR_I2C_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_i2c_bus, &scd41_cfg, &s_scd41_dev));
#endif

    /* 4. Configure DS18B20 1-Wire Pin */
#if CONFIG_ENABLE_DS18B20
    gpio_config_t ow_cfg = {
        .pin_bit_mask = (1ULL << DS18B20_1WIRE_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&ow_cfg);
#endif

    /* 5. Configure ZE03-NH3 UART */
#if CONFIG_ENABLE_ZE03_NH3
    uart_config_t uart_cfg = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(ZE03_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(ZE03_UART_PORT, ZE03_TX_GPIO, ZE03_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(ZE03_UART_PORT, 256, 0, 0, NULL, 0));
#endif

    /* 6. Configure JSN-SR04T GPIO Pins */
#if CONFIG_ENABLE_JSN_SR04T
    gpio_config_t trig_cfg = {
        .pin_bit_mask = (1ULL << JSN_TRIG_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig_cfg);
    gpio_set_level((gpio_num_t)JSN_TRIG_GPIO, 0);

    gpio_config_t echo_cfg = {
        .pin_bit_mask = (1ULL << JSN_ECHO_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLDOWN_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo_cfg);
#endif

    /* 7. Configure HX711 Serial Pins */
#if CONFIG_ENABLE_HX711
    gpio_config_t sck_cfg = {
        .pin_bit_mask = (1ULL << HX711_PD_SCK_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sck_cfg);
    gpio_set_level((gpio_num_t)HX711_PD_SCK_GPIO, 0);

    gpio_config_t dout_cfg = {
        .pin_bit_mask = (1ULL << HX711_DOUT_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dout_cfg);
#endif

    /* 8. Configure Soil Moisture ADC1 oneshot unit */
#if CONFIG_ENABLE_SOIL_MOISTURE
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = 0,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &s_adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, /* Attenuation calibrated for 0 - 3.3V range */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1_handle, (adc_channel_t)SOIL_MOISTURE_ADC_CHANNEL, &config));

    /* Calibration scheme setup */
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = (adc_channel_t)SOIL_MOISTURE_ADC_CHANNEL,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &s_adc1_cali_handle) == ESP_OK) {
        s_adc_cali_enabled = true;
    }
#endif

    return ESP_OK;
}

void sensor_hub_trigger_conversions(void)
{
    /* 1. Trigger DS18B20 Temp Conversion (takes ~750ms) */
#if CONFIG_ENABLE_DS18B20
    if (ds18b20_reset()) {
        ds18b20_write_byte(0xCC); /* Skip ROM */
        ds18b20_write_byte(0x44); /* Convert T */
    }
#endif

    /* 2. Trigger SCD41 Single-Shot CO2 Conversion (takes ~5s) */
#if CONFIG_ENABLE_SCD41
    scd41_send_cmd(0x36F6); /* Wake up the SCD41 first */
    vTaskDelay(pdMS_TO_TICKS(20));
    scd41_send_cmd(0x219D); /* Measure Single Shot */
#endif
}

esp_err_t sensor_hub_collect(sensor_hub_data_t *out_data)
{
    if (out_data == NULL) return ESP_ERR_INVALID_ARG;
    memset(out_data, 0, sizeof(sensor_hub_data_t));
    esp_err_t final_status = ESP_FAIL;

    /* A. Initialize sensors and registers once they have stabilized under VCC */
#if CONFIG_ENABLE_BME280
    if (bme280_init_sensor() == ESP_OK) {
        float t, h, p;
        if (bme280_read_data(&t, &h, &p) == ESP_OK) {
            out_data->bme280.temperature = t;
            out_data->bme280.humidity = h;
            out_data->bme280.pressure = p;
            out_data->bme280.valid = true;
            final_status = ESP_OK;
        }
    }
#endif

#if CONFIG_ENABLE_BH1750
    float lux = 0;
    if (bh1750_read_lux(&lux) == ESP_OK) {
        out_data->bh1750.lux = lux;
        out_data->bh1750.valid = true;
        final_status = ESP_OK;
    }
#endif

#if CONFIG_ENABLE_SOIL_MOISTURE
    float vwc = 0;
    if (soil_moisture_read_vwc(&vwc) == ESP_OK) {
        out_data->soil_moisture.vwc = vwc;
        out_data->soil_moisture.valid = true;
        final_status = ESP_OK;
    }
#endif

#if CONFIG_ENABLE_ZE03_NH3
    float nh3 = 0;
    if (ze03_read_nh3(&nh3) == ESP_OK) {
        out_data->winsen_nh3.nh3 = nh3;
        out_data->winsen_nh3.valid = true;
        final_status = ESP_OK;
    }
#endif

#if CONFIG_ENABLE_JSN_SR04T
    float dist = 0;
    if (jsn_read_distance(&dist) == ESP_OK) {
        out_data->jsn_sr04t.distance_cm = dist;
        out_data->jsn_sr04t.valid = true;
        final_status = ESP_OK;
    }
#endif

#if CONFIG_ENABLE_HX711
    int32_t hx_raw = 0;
    if (hx711_read_raw(&hx_raw) == ESP_OK) {
        /* Simple load cell calibration math: weight_kg = (raw - offset) / calibration_factor */
        /* Offset and factor would be calibrated in production, using defaults here */
        const int32_t offset = 8388608;
        const float cal_factor = 23000.0f;
        out_data->hx711.weight_kg = (float)(hx_raw - offset) / cal_factor;
        out_data->hx711.valid = true;
        final_status = ESP_OK;
    }
#endif

    /* B. Read slow sensors whose conversions were triggered beforehand */
#if CONFIG_ENABLE_DS18B20
    if (ds18b20_reset()) {
        ds18b20_write_byte(0xCC); /* Skip ROM */
        ds18b20_write_byte(0xBE); /* Read Scratchpad */
        uint8_t scratch[9];
        for (int i = 0; i < 9; i++) {
            scratch[i] = ds18b20_read_byte();
        }
        if (ds18b20_crc8(scratch, 8) == scratch[8]) {
            int16_t raw_temp = (scratch[1] << 8) | scratch[0];
            out_data->ds18b20.temperature = (float)raw_temp * 0.0625f;
            out_data->ds18b20.valid = true;
            final_status = ESP_OK;
        } else {
            ESP_LOGE(TAG, "DS18B20 Scratchpad CRC Mismatch");
        }
    }
#endif

#if CONFIG_ENABLE_SCD41
    float co2, scd_t, scd_h;
    if (scd41_read_measurement(&co2, &scd_t, &scd_h) == ESP_OK) {
        out_data->scd41.co2 = co2;
        out_data->scd41.valid = true;
        scd41_send_cmd(0x36E5); /* Put SCD41 back to sleep */
        final_status = ESP_OK;
    }
#endif

    return final_status;
}
