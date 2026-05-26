/**
 * @file ds18b20_sensor.c
 * @brief Maxim DS18B20 1-Wire Digital Temperature Sensor Driver (Plugin)
 *
 * Bit-banged 1-Wire protocol on a single GPIO pin.
 * Uses esp_rom_delay_us() for microsecond-accurate timing and
 * portENTER_CRITICAL / portEXIT_CRITICAL to protect timing-critical
 * read/write bit slots from preemption.
 *
 * Protocol flow:
 *   1. Reset pulse   → 480 µs low, release, 70 µs wait, check presence
 *   2. Skip ROM      → 0xCC (single device on bus)
 *   3. Convert T     → 0x44, wait 750 ms for 12-bit conversion
 *   4. Reset pulse
 *   5. Skip ROM      → 0xCC
 *   6. Read Scratchpad → 0xBE, read 9 bytes, CRC8 on first 8
 *   7. temp = int16 × 0.0625  (°C, 12-bit resolution)
 *
 * CRC-8: reflected polynomial 0x8C  (equivalent to normal 0x31 reflected)
 *
 * Capabilities: CAP_SOIL_TEMP
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

#define TAG "DS18B20"

/* ── Pin configuration (Kconfig, default GPIO 5) ────────────────────────── */
#ifndef CONFIG_SENSOR_DS18B20_PIN
#define CONFIG_SENSOR_DS18B20_PIN   5
#endif

/* ── 1-Wire ROM commands ────────────────────────────────────────────────── */
#define OW_CMD_SKIP_ROM         0xCC
#define OW_CMD_CONVERT_T        0x44
#define OW_CMD_READ_SCRATCHPAD  0xBE

/* ── Timing constants (microseconds) ────────────────────────────────────── */
#define OW_RESET_LOW_US         480     /* Master drives low during reset    */
#define OW_PRESENCE_WAIT_US     70      /* Wait before sampling presence     */
#define OW_RESET_TAIL_US        410     /* Remaining reset slot              */
#define OW_WRITE1_LOW_US        6       /* Write-1: brief low pulse          */
#define OW_WRITE1_RELEASE_US    64      /* Write-1: release remainder        */
#define OW_WRITE0_LOW_US        60      /* Write-0: long low pulse           */
#define OW_WRITE0_RELEASE_US    10      /* Write-0: release recovery         */
#define OW_READ_INIT_US         3       /* Read: initial low pulse           */
#define OW_READ_SAMPLE_US       10      /* Read: wait before sampling        */
#define OW_READ_TAIL_US         57      /* Read: remaining slot              */

/* ── Conversion time ────────────────────────────────────────────────────── */
#define DS18B20_CONVERT_DELAY_MS 750    /* 12-bit resolution max             */

/* ── Module state ───────────────────────────────────────────────────────── */
static bool s_initialized = false;

/* ========================================================================
 *  CRC-8 (Dallas / Maxim, reflected)
 *  Polynomial: 0x8C (reflected representation of x^8+x^5+x^4+1)
 *  Initialisation: 0x00
 * ======================================================================== */
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

/* ========================================================================
 *  1-Wire GPIO helpers
 *  The bus is open-drain with an external pull-up resistor.
 *  "Low"     = drive output low
 *  "Release" = switch to input (pull-up brings it high)
 * ======================================================================== */
static inline void ow_write_low(void)
{
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_DS18B20_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)CONFIG_SENSOR_DS18B20_PIN, 0);
}

static inline void ow_release(void)
{
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_DS18B20_PIN, GPIO_MODE_INPUT);
}

static inline int ow_read_pin(void)
{
    return gpio_get_level((gpio_num_t)CONFIG_SENSOR_DS18B20_PIN);
}

/* ========================================================================
 *  1-Wire reset  –  returns true if a presence pulse was detected
 * ======================================================================== */
static bool ow_reset(void)
{
    bool presence = false;

    ow_write_low();
    esp_rom_delay_us(OW_RESET_LOW_US);

    ow_release();
    esp_rom_delay_us(OW_PRESENCE_WAIT_US);

    presence = (ow_read_pin() == 0);
    esp_rom_delay_us(OW_RESET_TAIL_US);

    return presence;
}

/* ========================================================================
 *  1-Wire write / read single bit  –  timing-critical, runs in critical
 * ======================================================================== */
static portMUX_TYPE s_ow_mux = portMUX_INITIALIZER_UNLOCKED;

static void ow_write_bit(bool bit)
{
    portENTER_CRITICAL(&s_ow_mux);
    if (bit) {
        /* Write 1: short low then release */
        ow_write_low();
        esp_rom_delay_us(OW_WRITE1_LOW_US);
        ow_release();
        esp_rom_delay_us(OW_WRITE1_RELEASE_US);
    } else {
        /* Write 0: long low then release */
        ow_write_low();
        esp_rom_delay_us(OW_WRITE0_LOW_US);
        ow_release();
        esp_rom_delay_us(OW_WRITE0_RELEASE_US);
    }
    portEXIT_CRITICAL(&s_ow_mux);
}

static bool ow_read_bit(void)
{
    bool bit = false;
    portENTER_CRITICAL(&s_ow_mux);

    ow_write_low();
    esp_rom_delay_us(OW_READ_INIT_US);
    ow_release();
    esp_rom_delay_us(OW_READ_SAMPLE_US);
    bit = (ow_read_pin() != 0);
    esp_rom_delay_us(OW_READ_TAIL_US);

    portEXIT_CRITICAL(&s_ow_mux);
    return bit;
}

/* ========================================================================
 *  1-Wire write / read byte  –  LSB first
 * ======================================================================== */
static void ow_write_byte(uint8_t data)
{
    for (int i = 0; i < 8; i++) {
        ow_write_bit(data & 0x01);
        data >>= 1;
    }
}

static uint8_t ow_read_byte(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (ow_read_bit()) {
            data |= (1 << i);
        }
    }
    return data;
}

/* ========================================================================
 *  init()  – Configure the 1-Wire GPIO pin and verify sensor presence
 * ======================================================================== */
static esp_err_t ds18b20_init(void)
{
    ESP_LOGI(TAG, "DS18B20 init: 1-Wire GPIO%d", CONFIG_SENSOR_DS18B20_PIN);

    /* Configure the GPIO as input with internal pull-up
     * (external 4.7 kΩ pull-up recommended for production) */
    gpio_config_t ow_cfg = {
        .pin_bit_mask  = (1ULL << CONFIG_SENSOR_DS18B20_PIN),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&ow_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Attempt a reset to verify the sensor is on the bus */
    if (!ow_reset()) {
        ESP_LOGW(TAG, "No presence pulse detected – sensor may be absent or "
                 "bus not pulled up. Will retry on first read.");
    }

    s_initialized = true;
    ESP_LOGI(TAG, "DS18B20 initialised");
    return ESP_OK;
}

/* ========================================================================
 *  read()  – Full conversion cycle: Convert T → Read Scratchpad → CRC
 *
 *  Scratchpad layout (9 bytes):
 *    [0] Temp LSB   [1] Temp MSB
 *    [2] TH         [3] TL
 *    [4] Config Reg [5] Reserved
 *    [6] Reserved   [7] Reserved
 *    [8] CRC-8
 *
 *  Temperature = (int16_t)(scratch[1]<<8 | scratch[0]) × 0.0625  (°C)
 * ======================================================================== */
static esp_err_t ds18b20_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* ── Step 1: Trigger temperature conversion ─────────────────────────── */
    if (!ow_reset()) {
        ESP_LOGE(TAG, "Reset failed before Convert T – no presence pulse");
        return ESP_ERR_NOT_FOUND;
    }
    ow_write_byte(OW_CMD_SKIP_ROM);
    ow_write_byte(OW_CMD_CONVERT_T);

    /* Wait for 12-bit conversion (750 ms max) */
    vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERT_DELAY_MS));

    /* ── Step 2: Read scratchpad ────────────────────────────────────────── */
    if (!ow_reset()) {
        ESP_LOGE(TAG, "Reset failed before Read Scratchpad – no presence pulse");
        return ESP_ERR_NOT_FOUND;
    }
    ow_write_byte(OW_CMD_SKIP_ROM);
    ow_write_byte(OW_CMD_READ_SCRATCHPAD);

    uint8_t scratch[9];
    for (int i = 0; i < 9; i++) {
        scratch[i] = ow_read_byte();
    }

    /* ── Step 3: CRC validation ─────────────────────────────────────────── */
    if (ds18b20_crc8(scratch, 8) != scratch[8]) {
        ESP_LOGE(TAG, "Scratchpad CRC mismatch (expected 0x%02X, got 0x%02X)",
                 ds18b20_crc8(scratch, 8), scratch[8]);
        return ESP_ERR_INVALID_CRC;
    }

    /* ── Step 4: Decode temperature ─────────────────────────────────────── */
    int16_t raw_temp = (int16_t)(((uint16_t)scratch[1] << 8) | scratch[0]);
    float temperature = (float)raw_temp * 0.0625f;

    /* Sanity check: DS18B20 range is −55 °C to +125 °C */
    if (temperature < -55.0f || temperature > 125.0f) {
        ESP_LOGW(TAG, "Temperature %.2f°C outside sensor range", temperature);
    }

    /* ── Step 5: Populate output struct ─────────────────────────────────── */
    memset(data, 0, sizeof(sensor_data_t));

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    data->soil_temp.value.f32    = temperature;
    data->soil_temp.valid        = true;
    data->soil_temp.timestamp_ms = now_ms;

    ESP_LOGD(TAG, "DS18B20: T=%.2f°C (raw=0x%04X)", temperature,
             (unsigned)((uint16_t)raw_temp));
    return ESP_OK;
}

/* ========================================================================
 *  sleep()  – The DS18B20 auto-enters standby after conversion.
 *             Nothing extra to do; the pin is left floating (input).
 * ======================================================================== */
static esp_err_t ds18b20_sleep(void)
{
    ESP_LOGD(TAG, "DS18B20 idle (inherently low-power between conversions)");
    return ESP_OK;
}

/* ========================================================================
 *  wakeup()  – Presence pulse is enough to wake the device
 * ======================================================================== */
static esp_err_t ds18b20_wakeup(void)
{
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    if (!ow_reset()) {
        ESP_LOGW(TAG, "No presence detected on wake-up");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGD(TAG, "DS18B20 woken (presence detected)");
    return ESP_OK;
}

/* ========================================================================
 *  deinit()  – Release the GPIO and reset state
 * ======================================================================== */
static esp_err_t ds18b20_deinit(void)
{
    /* Float the pin to avoid parasitic current draw */
    gpio_set_direction((gpio_num_t)CONFIG_SENSOR_DS18B20_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_DS18B20_PIN, GPIO_FLOATING);

    s_initialized = false;
    ESP_LOGI(TAG, "DS18B20 deinitialised");
    return ESP_OK;
}

/* ========================================================================
 *  Sensor metadata
 * ======================================================================== */
static const sensor_info_t s_info = {
    .name                = "Maxim DS18B20 1-Wire",
    .model               = "DS18B20",
    .capabilities        = CAP_SOIL_TEMP,
    .min_interval_ms     = 1000,       /* ≥ 750 ms conversion time          */
    .default_interval_ms = 10000,
    .supports_sleep      = true,
};

static const sensor_info_t *ds18b20_get_info(void) { return &s_info; }

/* ========================================================================
 *  Exported operations vtable
 * ======================================================================== */
static const sensor_ops_t ds18b20_ops = {
    .init     = ds18b20_init,
    .read     = ds18b20_read,
    .sleep    = ds18b20_sleep,
    .wakeup   = ds18b20_wakeup,
    .get_info = ds18b20_get_info,
    .deinit   = ds18b20_deinit,
};

const sensor_ops_t ds18b20_sensor_ops = ds18b20_ops;
