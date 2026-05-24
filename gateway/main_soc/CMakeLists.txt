# Gateway CMakeLists.txt
# Hardware: ESP32-S3 (Wi-Fi Host + Application) + ESP32-H2 (802.15.4 Radio Co-Processor)
# Connection: UART (ot_rcp protocol)
# Role: Zigbee Coordinator, Matter Bridge, Web Server Host

cmake_minimum_required(VERSION 3.16)

set(PROJECT_VER "1.1.0-dualsoc")
set(GATEWAY_HW_MODE "DUAL_SOC_ESP32S3_ESP32H2")

# Default SDKCONFIG for ESP32-S3 host
if(NOT SDKCONFIG_DEFAULTS)
    set(SDKCONFIG_DEFAULTS "sdkconfig.defaults")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_zigbee_gateway)

# Validate target - must be ESP32-S3 for dual-SoC gateway host
if(CONFIG_IDF_TARGET AND NOT CONFIG_IDF_TARGET STREQUAL "esp32s3")
    message(FATAL_ERROR "Gateway Dual-SoC mode requires ESP32-S3 target. "
                        "Run: idf.py set-target esp32s3")
endif()

message(STATUS "========================================")
message(STATUS "Gateway Build Configuration")
message(STATUS "========================================")
message(STATUS "Project Version : ${PROJECT_VER}")
message(STATUS "Target Chip     : ${CONFIG_IDF_TARGET}")
message(STATUS "Hardware Mode   : ${GATEWAY_HW_MODE}")
message(STATUS "Radio Co-Proc   : ESP32-H2 via UART RCP")
message(STATUS "========================================")
