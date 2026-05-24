# RCP UART Component - Dual-SoC Radio Co-Processor UART Interface

idf_component_register(
    SRCS "rcp_uart.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
    PRIV_REQUIRES
        esp_hw_support
)
