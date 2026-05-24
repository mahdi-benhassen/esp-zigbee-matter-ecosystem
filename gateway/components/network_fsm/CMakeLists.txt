# Network FSM Component - Dual-SoC Gateway

idf_component_register(
    SRCS "network_fsm.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES
        freertos
        nvs_flash
        esp_timer
        esp-zigbee-lib
        rcp_uart
    PRIV_REQUIRES
)
