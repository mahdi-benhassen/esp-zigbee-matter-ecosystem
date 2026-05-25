#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * SENSOR ENABLE CONFIGURATION (Generic/Modular Architecture)
 * Set to 1 to compile and enable, or 0 to disable
 *============================================================================*/
#define CONFIG_ENABLE_BME280        1
#define CONFIG_ENABLE_BH1750        1
#define CONFIG_ENABLE_SCD41         1
#define CONFIG_ENABLE_SOIL_MOISTURE 1
#define CONFIG_ENABLE_DS18B20       1
#define CONFIG_ENABLE_ZE03_NH3      1
#define CONFIG_ENABLE_JSN_SR04T     1
#define CONFIG_ENABLE_HX711         1

/*=============================================================================
 * HARDWARE INTERFACE PINOUT CONFIGURATION
 *============================================================================*/
/* Power Gating Control */
#define SENSOR_POWER_CTRL_GPIO      4  /* GPIO_NUM_4 controls PMOS / VCC rail */

/* I2C Shared Bus (BME280, BH1750, SCD41) */
#define SENSOR_I2C_PORT             0
#define SENSOR_I2C_SDA_GPIO         6
#define SENSOR_I2C_SCL_GPIO         7
#define SENSOR_I2C_FREQ_HZ          100000

/* One-Wire Bus (DS18B20) */
#define DS18B20_1WIRE_GPIO          5

/* UART Interface (Winsen ZE03-NH3) */
#define ZE03_UART_PORT              1
#define ZE03_TX_GPIO                24
#define ZE03_RX_GPIO                23

/* GPIO Trig/Echo Interface (JSN-SR04T) */
#define JSN_TRIG_GPIO               0
#define JSN_ECHO_GPIO               1

/* HX711 2-Wire Serial (Load Cell) */
#define HX711_PD_SCK_GPIO           10
#define HX711_DOUT_GPIO             11

/* Soil Moisture Analog Channel (ADC1) & GPIO Pin */
#define SOIL_MOISTURE_ADC_CHANNEL   1  /* ADC1 Channel 1 (GPIO2) */
#define SOIL_MOISTURE_ADC_GPIO      2

/*=============================================================================
 * DATA STRUCTURES
 *============================================================================*/
typedef struct {
    struct {
        float temperature;  /* °C */
        float humidity;     /* % */
        float pressure;     /* hPa */
        bool valid;
    } bme280;

    struct {
        float lux;          /* Lux */
        bool valid;
    } bh1750;

    struct {
        float co2;          /* ppm */
        bool valid;
    } scd41;

    struct {
        float vwc;          /* Volumetric Water Content % */
        bool valid;
    } soil_moisture;

    struct {
        float temperature;  /* °C */
        bool valid;
    } ds18b20;

    struct {
        float nh3;          /* ppm */
        bool valid;
    } winsen_nh3;

    struct {
        float distance_cm;  /* cm */
        bool valid;
    } jsn_sr04t;

    struct {
        float weight_kg;    /* kg */
        bool valid;
    } hx711;
} sensor_hub_data_t;

/*=============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/
/**
 * @brief Initialize all configured hardware interfaces and GPIOs.
 *        Called once during initial cold boot.
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t sensor_hub_init(void);

/**
 * @brief Power-Gate ON external sensors.
 *        Drives SENSOR_POWER_CTRL_GPIO low/high to power-gate VCC.
 */
void sensor_hub_power_on(void);

/**
 * @brief Power-Gate OFF external sensors.
 *        Drives SENSOR_POWER_CTRL_GPIO to cut off sensor VCC.
 */
void sensor_hub_power_off(void);

/**
 * @brief Kick off conversion for slow sensors (DS18B20, SCD41).
 *        Allows concurrent conversion during the stabilization/warmup phase.
 */
void sensor_hub_trigger_conversions(void);

/**
 * @brief Read data sequentially from all enabled hardware sensors.
 * @param[out] out_data Pointer to local structure to hold the readings.
 * @return ESP_OK if at least one sensor was read successfully, or error code.
 */
esp_err_t sensor_hub_collect(sensor_hub_data_t *out_data);

#ifdef __cplusplus
}
#endif
