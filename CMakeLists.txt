# Root CMakeLists.txt - Smart Home IoT Ecosystem
# Hardware: ESP32-S3 (Wi-Fi Host) + ESP32-H2 (802.15.4 RCP) - Dual-SoC
#
# Build commands:
#   Gateway:  cd gateway && idf.py set-target esp32s3 && idf.py build
#   End Node: cd end_node && idf.py set-target esp32h2 && idf.py build

cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "1.1.0-dualsoc-universal")
set(PROJECT_NAME "esp_zigbee_matter_ecosystem")

message(STATUS "========================================")
message(STATUS "ESP-Zigbee + Matter Ecosystem")
message(STATUS "Hardware: Dual-SoC (ESP32-S3 + ESP32-H2)")
message(STATUS "Version: ${PROJECT_VER}")
message(STATUS "========================================")
