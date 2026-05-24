# Gateway main application

idf_component_register(
    SRCS "gateway_main.c"
    INCLUDE_DIRS "."
    REQUIRES
        network_fsm
        rcp_uart
        esp-zigbee-lib
        esp_wifi
        nvs_flash
        esp_http_server
        esp_timer
        driver
        spiffs
        fatfs
)
