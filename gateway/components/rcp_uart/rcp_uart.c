/**
 * @file rcp_uart.c
 * @brief Radio Co-Processor UART Implementation
 *
 * Manages UART communication between ESP32-S3 (Host) and ESP32-H2 (RCP).
 * The ESP32-H2 runs the ot_rcp (OpenThread RCP) or ZBOSS RCP firmware.
 *
 * Wiring (ESP32-S3 <-> ESP32-H2):
 *   S3 GPIO5 (TX)  -> H2 GPIO8 (RX)
 *   S3 GPIO4 (RX)  -> H2 GPIO9 (TX)
 *   S3 3.3V        -> H2 3.3V
 *   S3 GND         -> H2 GND
 *   S3 GPIO? (RST) -> H2 RST (optional)
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-dualsoc
 */

#include "rcp_uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <string.h>

/*=============================================================================
 * DEFINITIONS
 *============================================================================*/

#define TAG "RCP_UART"

/** RCP protocol magic bytes for frame detection */
#define RCP_FRAME_MAGIC_1       0x7E
#define RCP_FRAME_MAGIC_2       0x7C

/** RCP ready response pattern */
#define RCP_READY_PATTERN       "RCP_READY"
#define RCP_READY_PATTERN_LEN   9

/** Maximum RCP frame size (802.15.4 max PSDU + overhead) */
#define RCP_MAX_FRAME_SIZE      280

/*=============================================================================
 * MODULE STATE
 *============================================================================*/

static volatile rcp_state_t s_rcp_state = RCP_STATE_UNINITIALIZED;
static rcp_stats_t s_rcp_stats = {0};
static SemaphoreHandle_t s_rcp_mutex = NULL;
static esp_timer_handle_t s_heartbeat_timer = NULL;

/*=============================================================================
 * STATIC FUNCTIONS
 *============================================================================*/

/**
 * @brief Configure GPIO pins for RCP control (reset, boot)
 */
static esp_err_t configure_rcp_gpio(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };

    /* Configure RESET pin if connected */
    if (RCP_RESET_GPIO >= 0) {
        io_conf.pin_bit_mask = (1ULL << RCP_RESET_GPIO);
        gpio_config(&io_conf);
        gpio_set_level(RCP_RESET_GPIO, 1); /* Release reset (active low) */
    }

    /* Configure BOOT pin if connected */
    if (RCP_BOOT_GPIO >= 0) {
        io_conf.pin_bit_mask = (1ULL << RCP_BOOT_GPIO);
        gpio_config(&io_conf);
        gpio_set_level(RCP_BOOT_GPIO, 1); /* Normal boot mode */
    }

    return ESP_OK;
}

/**
 * @brief Perform hardware reset of ESP32-H2
 */
static esp_err_t hardware_reset_h2(void)
{
    if (RCP_RESET_GPIO < 0) {
        ESP_LOGW(TAG, "No RESET GPIO configured, skipping hardware reset");
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGI(TAG, "Performing hardware reset of ESP32-H2...");

    /* Assert reset (active low) */
    gpio_set_level(RCP_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Release reset */
    gpio_set_level(RCP_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ESP32-H2 reset complete");
    return ESP_OK;
}

/**
 * @brief Try to detect RCP ready signal in RX buffer
 */
static esp_err_t poll_rcp_ready(uint32_t timeout_ms)
{
    uint8_t rx_buf[128];
    uint32_t start_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    while ((uint32_t)(esp_timer_get_time() / 1000ULL) - start_ms < timeout_ms) {
        size_t rx_len = 0;
        uart_get_buffered_data_len(RCP_UART_PORT_NUM, &rx_len);

        if (rx_len >= RCP_READY_PATTERN_LEN) {
            int read = uart_read_bytes(RCP_UART_PORT_NUM, rx_buf,
                                       sizeof(rx_buf), pdMS_TO_TICKS(100));
            if (read > 0) {
                /* Search for ready pattern */
                for (int i = 0; i <= read - RCP_READY_PATTERN_LEN; i++) {
                    if (memcmp(&rx_buf[i], RCP_READY_PATTERN,
                               RCP_READY_PATTERN_LEN) == 0) {
                        ESP_LOGI(TAG, "RCP ready signal detected!");
                        return ESP_OK;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return ESP_ERR_TIMEOUT;
}

/*=============================================================================
 * PUBLIC API
 *============================================================================*/

esp_err_t rcp_uart_init(void)
{
    if (s_rcp_state != RCP_STATE_UNINITIALIZED) {
        ESP_LOGW(TAG, "RCP UART already initialized (state: %s)",
                 rcp_uart_state_name(s_rcp_state));
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "=== Initializing RCP UART ===");
    ESP_LOGI(TAG, "  Port: UART%d", RCP_UART_PORT_NUM);
    ESP_LOGI(TAG, "  TX Pin: GPIO%d", RCP_UART_TX_PIN);
    ESP_LOGI(TAG, "  RX Pin: GPIO%d", RCP_UART_RX_PIN);
    ESP_LOGI(TAG, "  Baud: %d", RCP_UART_BAUD_RATE);

    /* Create mutex */
    if (s_rcp_mutex == NULL) {
        s_rcp_mutex = xSemaphoreCreateMutex();
        if (s_rcp_mutex == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    /* Configure control GPIOs */
    ESP_ERROR_CHECK(configure_rcp_gpio());

    /* Configure UART */
    uart_config_t uart_config = {
        .baud_rate = RCP_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(RCP_UART_PORT_NUM,
                                        RCP_UART_BUF_SIZE_RX,
                                        RCP_UART_BUF_SIZE_TX,
                                        0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_param_config(RCP_UART_PORT_NUM, &uart_config);
    if (err != ESP_OK) {
        uart_driver_delete(RCP_UART_PORT_NUM);
        return err;
    }

    err = uart_set_pin(RCP_UART_PORT_NUM,
                       RCP_UART_TX_PIN,
                       RCP_UART_RX_PIN,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(RCP_UART_PORT_NUM);
        return err;
    }

    ESP_LOGI(TAG, "UART driver installed successfully");

    /* Reset and wait for RCP */
    s_rcp_state = RCP_STATE_RESETTING;
    ESP_ERROR_CHECK(rcp_uart_reset());

    return ESP_OK;
}

esp_err_t rcp_uart_deinit(void)
{
    ESP_LOGI(TAG, "Deinitializing RCP UART...");

    if (s_heartbeat_timer != NULL) {
        esp_timer_stop(s_heartbeat_timer);
        esp_timer_delete(s_heartbeat_timer);
        s_heartbeat_timer = NULL;
    }

    uart_driver_delete(RCP_UART_PORT_NUM);
    s_rcp_state = RCP_STATE_UNINITIALIZED;

    ESP_LOGI(TAG, "RCP UART deinitialized");
    return ESP_OK;
}

esp_err_t rcp_uart_reset(void)
{
    xSemaphoreTake(s_rcp_mutex, portMAX_DELAY);

    s_rcp_state = RCP_STATE_RESETTING;
    s_rcp_stats.resets++;

    /* Flush buffers */
    uart_flush_input(RCP_UART_PORT_NUM);

    /* Try hardware reset first */
    esp_err_t err = hardware_reset_h2();
    if (err != ESP_OK) {
        /* Fall back to software reset via UART */
        ESP_LOGI(TAG, "Sending software reset to RCP...");
        uint8_t reset_cmd[] = {RCP_FRAME_MAGIC_1, 0x01, 0xFF};
        uart_write_bytes(RCP_UART_PORT_NUM, (const char *)reset_cmd, sizeof(reset_cmd));
    }

    /* Wait for RCP to boot */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Wait for ready signal */
    s_rcp_state = RCP_STATE_WAITING_READY;
    xSemaphoreGive(s_rcp_mutex);

    err = rcp_uart_wait_ready(RCP_READY_TIMEOUT_MS);
    if (err == ESP_OK) {
        s_rcp_state = RCP_STATE_READY;
        ESP_LOGI(TAG, "RCP is ready and operational!");
    } else {
        s_rcp_state = RCP_STATE_ERROR;
        ESP_LOGW(TAG, "RCP did not respond within %d ms", RCP_READY_TIMEOUT_MS);
    }

    return err;
}

esp_err_t rcp_uart_wait_ready(uint32_t timeout_ms)
{
    ESP_LOGI(TAG, "Waiting for RCP ready (timeout: %lu ms)...", timeout_ms);

    esp_err_t err = poll_rcp_ready(timeout_ms);

    if (err == ESP_OK) {
        s_rcp_state = RCP_STATE_READY;
    }

    return err;
}

rcp_state_t rcp_uart_get_state(void)
{
    return s_rcp_state;
}

const char *rcp_uart_state_name(rcp_state_t state)
{
    switch (state) {
        case RCP_STATE_UNINITIALIZED:   return "UNINITIALIZED";
        case RCP_STATE_RESETTING:       return "RESETTING";
        case RCP_STATE_WAITING_READY:   return "WAITING_READY";
        case RCP_STATE_READY:           return "READY";
        case RCP_STATE_ERROR:           return "ERROR";
        case RCP_STATE_DISCONNECTED:    return "DISCONNECTED";
        default:                        return "UNKNOWN";
    }
}

esp_err_t rcp_uart_get_stats(rcp_stats_t *stats)
{
    if (stats == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    xSemaphoreTake(s_rcp_mutex, portMAX_DELAY);
    *stats = s_rcp_stats;
    xSemaphoreGive(s_rcp_mutex);
    return ESP_OK;
}

void rcp_uart_reset_stats(void)
{
    xSemaphoreTake(s_rcp_mutex, portMAX_DELAY);
    memset(&s_rcp_stats, 0, sizeof(s_rcp_stats));
    xSemaphoreGive(s_rcp_mutex);
}

bool rcp_uart_is_operational(void)
{
    return (s_rcp_state == RCP_STATE_READY);
}

esp_err_t rcp_uart_send_raw(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_rcp_state != RCP_STATE_READY) {
        return ESP_ERR_INVALID_STATE;
    }

    int written = uart_write_bytes(RCP_UART_PORT_NUM, (const char *)data, len);
    if (written < 0) {
        return ESP_FAIL;
    }

    xSemaphoreTake(s_rcp_mutex, portMAX_DELAY);
    s_rcp_stats.bytes_tx += written;
    xSemaphoreGive(s_rcp_mutex);

    return ESP_OK;
}

esp_err_t rcp_uart_recv_raw(uint8_t *buf, size_t buf_size, size_t *received)
{
    if (buf == NULL || received == NULL || buf_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    *received = 0;

    size_t rx_len = 0;
    uart_get_buffered_data_len(RCP_UART_PORT_NUM, &rx_len);
    if (rx_len == 0) {
        return ESP_ERR_TIMEOUT;
    }

    size_t to_read = (rx_len > buf_size) ? buf_size : rx_len;
    int read = uart_read_bytes(RCP_UART_PORT_NUM, buf, to_read, 0);
    if (read > 0) {
        *received = (size_t)read;
        xSemaphoreTake(s_rcp_mutex, portMAX_DELAY);
        s_rcp_stats.bytes_rx += *received;
        xSemaphoreGive(s_rcp_mutex);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

void rcp_uart_flush(void)
{
    uart_flush_input(RCP_UART_PORT_NUM);
    ESP_LOGD(TAG, "UART RX buffer flushed");
}

void rcp_uart_print_diagnostics(void)
{
    rcp_stats_t stats;
    rcp_uart_get_stats(&stats);

    ESP_LOGI(TAG, "=== RCP UART Diagnostics ===");
    ESP_LOGI(TAG, "  State: %s", rcp_uart_state_name(s_rcp_state));
    ESP_LOGI(TAG, "  Port: UART%d, Baud: %d", RCP_UART_PORT_NUM, RCP_UART_BAUD_RATE);
    ESP_LOGI(TAG, "  TX: GPIO%d -> H2 RX", RCP_UART_TX_PIN);
    ESP_LOGI(TAG, "  RX: GPIO%d <- H2 TX", RCP_UART_RX_PIN);
    ESP_LOGI(TAG, "  Bytes TX: %llu", stats.bytes_tx);
    ESP_LOGI(TAG, "  Bytes RX: %llu", stats.bytes_rx);
    ESP_LOGI(TAG, "  Resets: %lu", stats.resets);
    ESP_LOGI(TAG, "  Reconnects: %lu", stats.reconnects);
    ESP_LOGI(TAG, "============================");
}
