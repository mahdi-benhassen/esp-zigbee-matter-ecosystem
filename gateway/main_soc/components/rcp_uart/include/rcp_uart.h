/**
 * @file rcp_uart.h
 * @brief Radio Co-Processor UART Interface
 *
 * Manages the UART communication between ESP32-S3 (Host) and ESP32-H2 (RCP)
 * running the 802.15.4 radio firmware (ot_rcp / ZBOSS RCP).
 *
 * The ESP32-H2 must be flashed with the RCP firmware separately:
 *   cd $IDF_PATH/examples/openthread/ot_rcp && idf.py set-target esp32h2 flash
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-dualsoc
 */

#ifndef RCP_UART_H
#define RCP_UART_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * CONFIGURATION
 *============================================================================*/

/** UART port for RCP communication (default UART1) */
#ifndef RCP_UART_PORT_NUM
#define RCP_UART_PORT_NUM       CONFIG_ZB_UART_PORT
#endif

/** UART TX pin (ESP32-S3 TX -> ESP32-H2 RX) */
#ifndef RCP_UART_TX_PIN
#define RCP_UART_TX_PIN         CONFIG_ZB_UART_TX_PIN
#endif

/** UART RX pin (ESP32-S3 RX -> ESP32-H2 TX) */
#ifndef RCP_UART_RX_PIN
#define RCP_UART_RX_PIN         CONFIG_ZB_UART_RX_PIN
#endif

/** UART baud rate - must match ESP32-H2 RCP firmware */
#ifndef RCP_UART_BAUD_RATE
#define RCP_UART_BAUD_RATE      CONFIG_ZB_UART_BAUDRATE
#endif

/** UART buffer sizes */
#define RCP_UART_BUF_SIZE_TX    1024
#define RCP_UART_BUF_SIZE_RX    2048

/** RCP reset GPIO (optional - ESP32-H2 RESET pin) */
#ifndef RCP_RESET_GPIO
#ifdef CONFIG_RCP_RESET_GPIO
#define RCP_RESET_GPIO          CONFIG_RCP_RESET_GPIO
#else
#define RCP_RESET_GPIO          (-1)    /* -1 = not connected */
#endif
#endif

/** RCP boot mode GPIO (optional - pull low for download mode) */
#ifndef RCP_BOOT_GPIO
#define RCP_BOOT_GPIO           (-1)    /* -1 = not connected */
#endif

/** RCP ready timeout in milliseconds */
#define RCP_READY_TIMEOUT_MS    5000

/** RCP heartbeat interval in milliseconds */
#define RCP_HEARTBEAT_MS        3000

/*=============================================================================
 * DATA TYPES
 *============================================================================*/

/**
 * @brief RCP connection state
 */
typedef enum {
    RCP_STATE_UNINITIALIZED = 0,    /**< UART not initialized */
    RCP_STATE_RESETTING,            /**< Resetting ESP32-H2 */
    RCP_STATE_WAITING_READY,        /**< Waiting for RCP ready signal */
    RCP_STATE_READY,                /**< RCP ready, operational */
    RCP_STATE_ERROR,                /**< Communication error */
    RCP_STATE_DISCONNECTED,         /**< RCP disconnected */
} rcp_state_t;

/**
 * @brief RCP statistics
 */
typedef struct {
    uint64_t bytes_tx;              /**< Total bytes transmitted */
    uint64_t bytes_rx;              /**< Total bytes received */
    uint32_t frames_tx;             /**< 802.15.4 frames transmitted */
    uint32_t frames_rx;             /**< 802.15.4 frames received */
    uint32_t crc_errors;            /**< CRC errors detected */
    uint32_t resets;                /**< Number of RCP resets */
    uint32_t reconnects;            /**< Number of reconnections */
    uint32_t last_heartbeat_ms;     /**< Timestamp of last heartbeat */
} rcp_stats_t;

/*=============================================================================
 * API FUNCTIONS
 *============================================================================*/

/**
 * @brief Initialize RCP UART interface
 *
 * Configures UART peripheral, installs driver, and resets the ESP32-H2.
 * Must be called BEFORE esp_zb_init() so the Zigbee stack can communicate
 * with the 802.15.4 radio.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if already initialized
 */
esp_err_t rcp_uart_init(void);

/**
 * @brief Deinitialize RCP UART interface
 *
 * Uninstalls UART driver. Zigbee stack must be stopped first.
 *
 * @return ESP_OK on success
 */
esp_err_t rcp_uart_deinit(void);

/**
 * @brief Reset the ESP32-H2 RCP chip
 *
 * Toggles the RESET pin if connected, or sends a soft reset command.
 * Waits for RCP_READY_TIMEOUT_MS for the RCP to become ready.
 *
 * @return ESP_OK if RCP is ready after reset
 */
esp_err_t rcp_uart_reset(void);

/**
 * @brief Wait for RCP to become ready
 *
 * Blocks until the ESP32-H2 sends the ready signal or timeout.
 *
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout
 */
esp_err_t rcp_uart_wait_ready(uint32_t timeout_ms);

/**
 * @brief Get current RCP connection state
 *
 * @return Current RCP state
 */
rcp_state_t rcp_uart_get_state(void);

/**
 * @brief Get human-readable RCP state name
 *
 * @param state RCP state
 * @return Static string representation
 */
const char *rcp_uart_state_name(rcp_state_t state);

/**
 * @brief Get RCP communication statistics
 *
 * @param[out] stats Pointer to statistics structure
 * @return ESP_OK on success
 */
esp_err_t rcp_uart_get_stats(rcp_stats_t *stats);

/**
 * @brief Reset statistics counters
 */
void rcp_uart_reset_stats(void);

/**
 * @brief Check if RCP is currently operational
 *
 * @return true if RCP_STATE_READY
 */
bool rcp_uart_is_operational(void);

/**
 * @brief Send raw bytes to RCP (low-level)
 *
 * @param data Data buffer
 * @param len  Data length
 * @return ESP_OK on success
 */
esp_err_t rcp_uart_send_raw(const uint8_t *data, size_t len);

/**
 * @brief Receive raw bytes from RCP (non-blocking)
 *
 * @param[out] buf  Receive buffer
 * @param[in]  buf_size Buffer size
 * @param[out] received Number of bytes received
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no data
 */
esp_err_t rcp_uart_recv_raw(uint8_t *buf, size_t buf_size, size_t *received);

/**
 * @brief Flush UART RX buffer
 */
void rcp_uart_flush(void);

/**
 * @brief Print RCP diagnostics to console
 */
void rcp_uart_print_diagnostics(void);

#ifdef __cplusplus
}
#endif

#endif /* RCP_UART_H */
