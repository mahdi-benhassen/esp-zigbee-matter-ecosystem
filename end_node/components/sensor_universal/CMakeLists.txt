# Universal Sensor Registry Component

idf_component_register(
    SRCS "sensor_registry.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES
        esp_timer
        driver
        esp_driver_i2c
    PRIV_REQUIRES
        sensor_drivers
)

# Include math library for log10 in illuminance conversion
target_link_libraries(${COMPONENT_LIB} PRIVATE m)