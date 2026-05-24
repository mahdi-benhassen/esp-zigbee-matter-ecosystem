# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# OpenThread RCP firmware for ESP32-H2 (Radio Co-Processor)
# This firmware runs on the ESP32-H2 and provides 802.15.4 radio
# access over UART using the Spinel/HDLC protocol.
# The ESP32-S3 host communicates with this RCP via the rcp_uart component.

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp_zigbee_rcp)
