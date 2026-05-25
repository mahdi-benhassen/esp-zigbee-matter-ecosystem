# Project Libraries and SDKs

This document outlines the standard libraries, Espressif frameworks, and component registry dependencies utilized in the ESP-Zigbee-Matter Ecosystem project.

---

## 1. Core Frameworks & SDKs

*   **ESP-IDF (Espressif IoT Development Framework)**
    *   **Version:** `v5.5.x` (or newer v5.3+)
    *   **Purpose:** The primary development framework providing the RTOS, hardware drivers, and core system functionalities for both the ESP32-S3 (Gateway Host) and ESP32-H2 (End Node / RCP).
*   **OpenThread / ZBOSS RCP Stack**
    *   **Purpose:** Runs on the ESP32-H2 Radio Co-Processor to handle IEEE 802.15.4 MAC and PHY layers, communicating with the Gateway Host via the Spinel protocol.

---

## 2. Managed Components (ESP Component Registry)

These dependencies are defined in the `idf_component.yml` files for the Gateway and End Node and are automatically downloaded during the build process.

*   **`espressif/esp-zigbee-lib`** (Version `^2.0.0`)
    *   **Purpose:** The official Espressif Zigbee SDK wrapper around the ZBOSS stack. It handles Zigbee network formation, joining, and ZCL (Zigbee Cluster Library) data model management.
    *   **Used In:** Gateway (Coordinator) and End Node (Sleepy End Device).
*   **`espressif/esp_matter`** (Version `^1.0.0`)
    *   **Purpose:** The Espressif Matter SDK used to bridge the Zigbee network to the Matter ecosystem.
    *   **Used In:** Gateway (Main SoC).
*   **`espressif/cjson`** (Version `^1.7.17`)
    *   **Purpose:** A lightweight JSON parser used for formatting and parsing structured data (e.g., web dashboard APIs, network registry exports).
    *   **Used In:** Gateway (Main SoC).

---

## 3. ESP-IDF Core Libraries & Drivers

The project makes extensive use of the ESP-IDF standard API for hardware interaction and system management.

### System & RTOS
*   **`freertos/FreeRTOS.h`, `freertos/task.h`, `freertos/semphr.h`**
    *   Provides task scheduling, delay functions, and Mutexes (`SemaphoreHandle_t`) for thread-safe operations across the network FSM and UART drivers.
*   **`esp_log.h`, `esp_err.h`, `esp_system.h`**
    *   Provides standardized error checking (`ESP_ERROR_CHECK`), system restarts, and leveled logging capabilities.
*   **`esp_timer.h`**
    *   Provides high-resolution hardware timers used for strict sensor boot timings, network timeouts, and precise bit-banging delays (e.g., 1-Wire, DHT22, HX711).
*   **`esp_sleep.h`**
    *   Manages Deep Sleep and Light Sleep modes for ultra-low power operation on the Sleepy End Device.

### Storage & Networking
*   **`nvs_flash.h`, `nvs.h`**
    *   Non-Volatile Storage (NVS) used to persist network configurations, node registries, and Zigbee network keys across reboots.
*   **`esp_wifi.h`, `esp_netif.h`, `esp_event.h`**
    *   Manages the Wi-Fi Station connection, IP acquisition, and event loop for the ESP32-S3 Gateway host.

### Hardware Drivers
*   **`driver/gpio.h`**
    *   Handles digital input/output for sensor power gating, strapping pins, and bit-banged protocols.
*   **`driver/uart.h`**
    *   Manages UART interfaces for both the Spinel RCP connection and external UART sensors (e.g., Winsen ZE03).
*   **`driver/i2c_master.h` / `driver/i2c.h`**
    *   Manages the shared I2C bus for sensors like the SCD41, BME280, and BH1750.
*   **`esp_adc/adc_oneshot.h`**
    *   Manages analog-to-digital conversions for sensors such as the analog Soil Moisture probe.

---

## 4. Standard C/C++ Libraries

*   **`<stdio.h>`, `<stdlib.h>`, `<string.h>`**
    *   Used for standard memory operations, string formatting, and buffer manipulations.
*   **`<stdbool.h>`, `<stdint.h>`**
    *   Used extensively to maintain strict type and width definitions for hardware registers, Zigbee payloads, and state machine variables.
*   **`<math.h>`**
    *   Used for processing and scaling analog readings and raw sensor data.
