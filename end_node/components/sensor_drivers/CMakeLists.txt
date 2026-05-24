# Sensor Drivers Collection - Universal
#
# Each driver is conditionally compiled based on Kconfig selection.
# Only the selected sensor(s) are included in the build.

set(SENSOR_DRIVER_SRCS "")

# Stub/Simulation (fallback when no real sensor)
if(CONFIG_SENSOR_STUB)
    list(APPEND SENSOR_DRIVER_SRCS "stub/stub_sensor.c")
endif()

# ESP32 Internal Temperature Sensor
if(CONFIG_SENSOR_INTERNAL)
    list(APPEND SENSOR_DRIVER_SRCS "internal/internal_sensor.c")
endif()

# I2C Sensors
if(CONFIG_SENSOR_SHT30)
    list(APPEND SENSOR_DRIVER_SRCS "sht30/sht30_sensor.c")
endif()

if(CONFIG_SENSOR_SHT4X)
    list(APPEND SENSOR_DRIVER_SRCS "sht4x/sht4x_sensor.c")
endif()

if(CONFIG_SENSOR_AHT20)
    list(APPEND SENSOR_DRIVER_SRCS "aht20/aht20_sensor.c")
endif()

# One-Wire Sensors
if(CONFIG_SENSOR_DHT22)
    list(APPEND SENSOR_DRIVER_SRCS "dht22/dht22_sensor.c")
endif()

# If no sensor selected, force stub
if(NOT SENSOR_DRIVER_SRCS)
    list(APPEND SENSOR_DRIVER_SRCS "stub/stub_sensor.c")
endif()

message(STATUS "Sensor drivers enabled: ${SENSOR_DRIVER_SRCS}")

idf_component_register(
    SRCS ${SENSOR_DRIVER_SRCS}
REQUIRES
        sensor_universal
        driver
        esp_adc
        esp_driver_tsens
    PRIV_REQUIRES
        freertos
)

# Include math library
target_link_libraries(${COMPONENT_LIB} PRIVATE m)

