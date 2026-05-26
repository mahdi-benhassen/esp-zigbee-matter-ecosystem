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

if(CONFIG_SENSOR_BME280)
    list(APPEND SENSOR_DRIVER_SRCS "bme280/bme280_sensor.c")
endif()

if(CONFIG_SENSOR_BH1750)
    list(APPEND SENSOR_DRIVER_SRCS "bh1750/bh1750_sensor.c")
endif()

if(CONFIG_SENSOR_VEML7700)
    list(APPEND SENSOR_DRIVER_SRCS "veml7700/veml7700_sensor.c")
endif()

if(CONFIG_SENSOR_SCD41)
    list(APPEND SENSOR_DRIVER_SRCS "scd41/scd41_sensor.c")
endif()

# One-Wire Sensors
if(CONFIG_SENSOR_DHT22)
    list(APPEND SENSOR_DRIVER_SRCS "dht22/dht22_sensor.c")
endif()

if(CONFIG_SENSOR_DS18B20)
    list(APPEND SENSOR_DRIVER_SRCS "ds18b20/ds18b20_sensor.c")
endif()

# UART Sensors
if(CONFIG_SENSOR_ZE03_NH3)
    list(APPEND SENSOR_DRIVER_SRCS "ze03_nh3/ze03_nh3_sensor.c")
endif()

# Analog / ADC Sensors
if(CONFIG_SENSOR_SOIL_MOISTURE)
    list(APPEND SENSOR_DRIVER_SRCS "soil_moisture/soil_moisture_sensor.c")
endif()

# GPIO Sensors
if(CONFIG_SENSOR_JSN_SR04T)
    list(APPEND SENSOR_DRIVER_SRCS "jsn_sr04t/jsn_sr04t_sensor.c")
endif()

if(CONFIG_SENSOR_HX711)
    list(APPEND SENSOR_DRIVER_SRCS "hx711/hx711_sensor.c")
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
    esp_rom
PRIV_REQUIRES
    freertos
    esp_driver_gpio
    esp_driver_uart
)

# Include math library
target_link_libraries(${COMPONENT_LIB} PRIVATE m)