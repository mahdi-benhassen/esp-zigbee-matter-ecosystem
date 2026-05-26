/**
 * @file ze03_nh3_sensor.c
 * @brief Winsen ZE03-NH3 Electrochemical Ammonia Gas Sensor Driver (Plugin)
 *
 * UART (9600 8N1) Q&A mode interface.
 *
 * Protocol:
 *   TX command (9 bytes): {0xFF, 0x01, 0x86, 0x00…0x00, checksum}
 *   RX response (9 bytes): {0xFF, 0x86, high, low, …, checksum}
 *
 *   Checksum = ~(sum of bytes[1..7]) + 1   (modular inverse)
 *   Concentration (ppm) = (byte[2] << 8) | byte[3]
 *
 * The sensor does not have a hardware sleep mode, so sleep/wakeup are no-ops.
 *
 * Capabilities: CAP_NH3
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define TAG "ZE03_NH3"

/* ── UART pin / port configuration (Kconfig, with defaults) ─────────────── */
#ifndef CONFIG_SENSOR_ZE03_UART_PORT
#define CONFIG_SENSOR_ZE03_UART_PORT    1
#endif
#ifndef CONFIG_SENSOR_ZE03_TX_PIN
#define CONFIG_SENSOR_ZE03_TX_PIN       24
#endif
#ifndef CONFIG_SENSOR_ZE03_RX_PIN
#define CONFIG_SENSOR_ZE03_RX_PIN       23
#endif

/* ── UART parameters ────────────────────────────────────────────────────── */
#define ZE03_BAUD_RATE          9600
#define ZE03_RX_BUF_SIZE        256     /* UART RX ring buffer size          */
#define ZE03_RESPONSE_LEN       9       /* Fixed 9-byte response frame       */
#define ZE03_RX_TIMEOUT_MS      1000    /* Max wait for a complete response  */

/* ── Q&A mode read command ──────────────────────────────────────────────── */
static const uint8_t s_read_cmd[9] = {
    0xFF, 0x01, 0x86,
    0x00, 0x00, 0x00, 0x00, 0x00,
    0x79   /* checksum: ~(0x01+0x86) + 1 = 0x79 */
};

/* ── Module state ───────────────────────────────────────────────────────── */
static bool s_initialized = false;

/* ========================================================================
 *  Checksum verification
 *  Algorithm: sum bytes [1..7], negate, add 1  →  compare with byte [8]
 * ======================================================================== */
static bool ze03_verify_checksum(const uint8_t *frame)
{
    uint8_t sum = 0;
    for (int i = 1; i < 8; i++) {
        sum += frame[i];
    }
    uint8_t expected = (~sum) + 1;
    return (expected == frame[8]);
}

/* ========================================================================
 *  init()  – Configure UART peripheral, flush any stale data
 * ======================================================================== */
static esp_err_t ze03_nh3_init(void)
{
    ESP_LOGI(TAG, "ZE03-NH3 init: UART%d, TX=GPIO%d, RX=GPIO%d, %d baud",
             CONFIG_SENSOR_ZE03_UART_PORT,
             CONFIG_SENSOR_ZE03_TX_PIN,
             CONFIG_SENSOR_ZE03_RX_PIN,
             ZE03_BAUD_RATE);

    /* ── UART configuration ─────────────────────────────────────────────── */
    uart_config_t uart_cfg = {
        .baud_rate  = ZE03_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(CONFIG_SENSOR_ZE03_UART_PORT, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(CONFIG_SENSOR_ZE03_UART_PORT,
                       CONFIG_SENSOR_ZE03_TX_PIN,
                       CONFIG_SENSOR_ZE03_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(CONFIG_SENSOR_ZE03_UART_PORT,
                              ZE03_RX_BUF_SIZE,  /* RX buffer */
                              0,                  /* TX buffer (0 = blocking) */
                              0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Flush any boot-time noise from the RX FIFO */
    uart_flush(CONFIG_SENSOR_ZE03_UART_PORT);

    s_initialized = true;
    ESP_LOGI(TAG, "ZE03-NH3 initialised (Q&A mode)");
    return ESP_OK;
}

/* ========================================================================
 *  read()  – Send Q&A read command, parse 9-byte response, validate CRC
 *
 *  Response frame:
 *    [0] 0xFF   start byte
 *    [1] 0x86   command echo
 *    [2] conc_high
 *    [3] conc_low
 *    [4..7] reserved / full-range bytes (ignored)
 *    [8] checksum
 *
 *  NH3 ppm = (byte[2] << 8) | byte[3]
 * ======================================================================== */
static esp_err_t ze03_nh3_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Flush stale data before a fresh transaction */
    uart_flush_input(CONFIG_SENSOR_ZE03_UART_PORT);

    /* ── Send the read command ──────────────────────────────────────────── */
    int written = uart_write_bytes(CONFIG_SENSOR_ZE03_UART_PORT,
                                   (const char *)s_read_cmd,
                                   sizeof(s_read_cmd));
    if (written != sizeof(s_read_cmd)) {
        ESP_LOGE(TAG, "UART write failed (wrote %d / %d bytes)",
                 written, (int)sizeof(s_read_cmd));
        return ESP_FAIL;
    }

    /* ── Read 9-byte response ───────────────────────────────────────────── */
    uint8_t rx[ZE03_RESPONSE_LEN] = {0};
    int len = uart_read_bytes(CONFIG_SENSOR_ZE03_UART_PORT,
                              rx, ZE03_RESPONSE_LEN,
                              pdMS_TO_TICKS(ZE03_RX_TIMEOUT_MS));
    if (len < ZE03_RESPONSE_LEN) {
        ESP_LOGE(TAG, "UART timeout / incomplete response (got %d / %d bytes)",
                 len, ZE03_RESPONSE_LEN);
        return ESP_ERR_TIMEOUT;
    }

    /* ── Validate start byte ────────────────────────────────────────────── */
    if (rx[0] != 0xFF) {
        ESP_LOGE(TAG, "Invalid start byte: 0x%02X (expected 0xFF)", rx[0]);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* ── Validate checksum ──────────────────────────────────────────────── */
    if (!ze03_verify_checksum(rx)) {
        uint8_t sum = 0;
        for (int i = 1; i < 8; i++) sum += rx[i];
        uint8_t expected = (~sum) + 1;
        ESP_LOGE(TAG, "Checksum mismatch (expected 0x%02X, got 0x%02X)",
                 expected, rx[8]);
        return ESP_ERR_INVALID_CRC;
    }

    /* ── Decode NH3 concentration ───────────────────────────────────────── */
    uint16_t raw_conc = ((uint16_t)rx[2] << 8) | rx[3];
    float nh3_ppm = (float)raw_conc;

    /* ── Populate output struct ─────────────────────────────────────────── */
    memset(data, 0, sizeof(sensor_data_t));

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    data->nh3.value.f32    = nh3_ppm;
    data->nh3.valid        = true;
    data->nh3.timestamp_ms = now_ms;

    ESP_LOGD(TAG, "ZE03-NH3: %.0f ppm (raw=0x%04X)", nh3_ppm, raw_conc);
    return ESP_OK;
}

/* ========================================================================
 *  sleep() / wakeup()
 *  The ZE03-NH3 electrochemical cell has no hardware sleep mode.
 *  These are intentional no-ops; the sensor draws ~15 mA continuously.
 *  Power gating via an external MOSFET switch is the recommended
 *  low-power strategy.
 * ======================================================================== */
static esp_err_t ze03_nh3_sleep(void)
{
    ESP_LOGD(TAG, "ZE03-NH3 has no sleep mode (electrochemical cell)");
    return ESP_OK;
}

static esp_err_t ze03_nh3_wakeup(void)
{
    ESP_LOGD(TAG, "ZE03-NH3 has no wakeup mode (always active)");
    return ESP_OK;
}

/* ========================================================================
 *  deinit()  – Uninstall UART driver, float the pins
 * ======================================================================== */
static esp_err_t ze03_nh3_deinit(void)
{
    if (s_initialized) {
        uart_driver_delete(CONFIG_SENSOR_ZE03_UART_PORT);

        /* Float TX/RX pins to avoid current leakage when sensor is powered off */
        gpio_set_direction((gpio_num_t)CONFIG_SENSOR_ZE03_TX_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_ZE03_TX_PIN, GPIO_FLOATING);
        gpio_set_direction((gpio_num_t)CONFIG_SENSOR_ZE03_RX_PIN, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)CONFIG_SENSOR_ZE03_RX_PIN, GPIO_FLOATING);
    }

    s_initialized = false;
    ESP_LOGI(TAG, "ZE03-NH3 deinitialised");
    return ESP_OK;
}

/* ========================================================================
 *  Sensor metadata
 * ======================================================================== */
static const sensor_info_t s_info = {
    .name                = "Winsen ZE03-NH3 Ammonia",
    .model               = "ZE03-NH3",
    .capabilities        = CAP_NH3,
    .min_interval_ms     = 1000,        /* Sensor can respond ~1 s minimum   */
    .default_interval_ms = 30000,
    .supports_sleep      = false,       /* No hardware sleep available       */
};

static const sensor_info_t *ze03_nh3_get_info(void) { return &s_info; }

/* ========================================================================
 *  Exported operations vtable
 * ======================================================================== */
static const sensor_ops_t ze03_nh3_ops = {
    .init     = ze03_nh3_init,
    .read     = ze03_nh3_read,
    .sleep    = ze03_nh3_sleep,
    .wakeup   = ze03_nh3_wakeup,
    .get_info = ze03_nh3_get_info,
    .deinit   = ze03_nh3_deinit,
};

const sensor_ops_t ze03_nh3_sensor_ops = ze03_nh3_ops;
