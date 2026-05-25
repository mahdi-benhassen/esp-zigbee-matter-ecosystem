# Main component CMakeLists.txt for Zigbee Sleepy End Device
idf_component_register(
    SRCS
        "main.c"
        "sensor_hub.c"
        "zigbee_config.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp-zigbee-lib
        nvs_flash
        esp_timer
        driver
        esp_adc
)
