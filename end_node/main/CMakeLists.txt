# Universal End Node - Main Application
#
# Sensor-agnostic: build configuration selects sensor type(s)
# ZCL clusters auto-registered based on sensor capabilities

idf_component_register(
    SRCS
        "end_node_main.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        sensor_universal
        sensor_drivers
        zcl_clusters
        esp-zigbee-lib
        nvs_flash
        esp_timer
        driver
        esp_adc
        esp_driver_tsens
        spi_flash
)
