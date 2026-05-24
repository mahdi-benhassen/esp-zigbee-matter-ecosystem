/**
 * @file sensor_registry.h
 * @brief Universal Sensor Plugin Architecture
 *
 * One codebase for ALL sensor node types. Each sensor "registers" as a plugin
 * at compile time via Kconfig. The ZCL clusters are auto-configured based on
 * which sensors are enabled.
 *
 * To add a new sensor type:
 *   1. Create driver in sensor_drivers/<name>/
 *   2. Add Kconfig option
 *   3. Implement sensor_ops_t interface
 *   4. Register in SENSOR_REGISTER() macro
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#ifndef SENSOR_REGISTRY_H
#define SENSOR_REGISTRY_H

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * SENSOR CAPABILITY FLAGS (determines ZCL clusters)
 *============================================================================*/

/** Bitmask of sensor capabilities - auto-generates ZCL clusters */
typedef enum {
    CAP_TEMPERATURE     = (1 << 0), /**< Temperature Measurement cluster */
    CAP_HUMIDITY        = (1 << 1), /**< Relative Humidity cluster */
    CAP_PRESSURE        = (1 << 2), /**< Pressure Measurement cluster */
    CAP_ILLUMINANCE     = (1 << 3), /**< Illuminance Measurement cluster */
    CAP_OCCUPANCY       = (1 << 4), /**< Occupancy Sensing cluster */
    CAP_ONOFF           = (1 << 5), /**< On/Off cluster (relay/switch) */
    CAP_LEVEL_CONTROL   = (1 << 6), /**< Level Control (dimmer) */
    CAP_COLOR           = (1 << 7), /**< Color Control cluster */
    CAP_IAS_ZONE        = (1 << 8), /**< IAS Zone (security sensor) */
    CAP_FLOW            = (1 << 9), /**< Flow Measurement cluster */
    CAP_SOIL_MOISTURE   = (1 << 10),/**< Soil Moisture (analog input) */
    CAP_CO2             = (1 << 11),/**< CO2 Measurement (analog) */
    CAP_PM25            = (1 << 12),/**< PM2.5 Measurement (analog) */
} sensor_capability_t;

/*=============================================================================
 * SENSOR DATA STRUCTURES
 *============================================================================*/

/** @brief Single sensor reading value */
typedef struct {
    union {
        float    f32;       /**< Floating point value (temperature, humidity, etc.) */
        int16_t  i16;       /**< Integer value (ZCL native) */
        uint16_t u16;       /**< Unsigned integer */
        bool     b;         /**< Boolean (occupancy, on/off) */
        uint8_t  u8;        /**< Byte (level 0-255) */
        uint32_t u32;       /**< Extended color value */
    } value;
    bool valid;             /**< true if reading succeeded */
    uint32_t timestamp_ms;  /**< Reading timestamp */
} sensor_value_t;

/** @brief Complete sensor data snapshot */
typedef struct {
    sensor_value_t temperature;     /**< Degrees Celsius (ZCL: hundredths) */
    sensor_value_t humidity;        /**< Percent 0-100 (ZCL: hundredths) */
    sensor_value_t pressure;        /**< hPa (ZCL: 10ths of hPa) */
    sensor_value_t illuminance;     /**< Lux (ZCL: 10000 * log10(lux) + 1) */
    sensor_value_t occupancy;       /**< true = occupied */
    sensor_value_t onoff;           /**< true = on */
    sensor_value_t level;           /**< 0-255 (dimmer level) */
    sensor_value_t soil_moisture;   /**< Percent 0-100 */
    sensor_value_t co2;             /**< ppm */
    sensor_value_t pm25;            /**< ug/m3 */
} sensor_data_t;

/** @brief Sensor driver metadata */
typedef struct {
    const char *name;               /**< Human-readable name */
    const char *model;              /**< Model identifier string */
    uint32_t capabilities;          /**< CAP_* bitmask */
    uint32_t min_interval_ms;       /**< Minimum read interval */
    uint32_t default_interval_ms;   /**< Default read interval */
    bool supports_sleep;            /**< Can enter low-power between reads */
} sensor_info_t;

/*=============================================================================
 * SENSOR OPERATIONS INTERFACE
 *============================================================================*/

/**
 * @brief Sensor operations vtable - every sensor implements this
 */
typedef struct sensor_ops {
    /**
     * @brief Initialize the sensor hardware
     * @return ESP_OK on success
     */
    esp_err_t (*init)(void);

    /**
     * @brief Read all supported values into data struct
     * @param[out] data Sensor data structure to fill
     * @return ESP_OK on success
     */
    esp_err_t (*read)(sensor_data_t *data);

    /**
     * @brief Put sensor in low-power mode
     * @return ESP_OK on success
     */
    esp_err_t (*sleep)(void);

    /**
     * @brief Wake sensor from low-power mode
     * @return ESP_OK on success
     */
    esp_err_t (*wakeup)(void);

    /**
     * @brief Get sensor info/metadata
     * @return Pointer to static info struct
     */
    const sensor_info_t *(*get_info)(void);

    /**
     * @brief Deinitialize sensor hardware
     * @return ESP_OK on success
     */
    esp_err_t (*deinit)(void);
} sensor_ops_t;

/*=============================================================================
 * SENSOR REGISTRY API
 *============================================================================*/

/**
 * @brief Initialize sensor subsystem
 *
 * Scans all compiled-in sensor drivers, calls their init(),
 * and builds the capability mask for ZCL auto-registration.
 *
 * @return ESP_OK on success
 */
esp_err_t sensor_registry_init(void);

/**
 * @brief Deinitialize all sensors
 * @return ESP_OK on success
 */
esp_err_t sensor_registry_deinit(void);

/**
 * @brief Read all enabled sensors
 * @param[out] data Filled with readings from all sensors
 * @return ESP_OK on success
 */
esp_err_t sensor_registry_read_all(sensor_data_t *data);

/**
 * @brief Put all sensors to sleep
 * @return ESP_OK on success
 */
esp_err_t sensor_registry_sleep_all(void);

/**
 * @brief Wake all sensors
 * @return ESP_OK on success
 */
esp_err_t sensor_registry_wakeup_all(void);

/**
 * @brief Get combined capability mask of all enabled sensors
 * @return Bitmask of CAP_* flags
 */
uint32_t sensor_registry_get_capabilities(void);

/**
 * @brief Get the number of registered (enabled) sensors
 * @return Count of active sensors
 */
uint8_t sensor_registry_get_count(void);

/**
 * @brief Get info for a specific sensor by index
 * @param[in]  idx  Sensor index (0 to get_count()-1)
 * @param[out] info Info structure to fill
 * @param[out] ops  Operations pointer (can be NULL)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if index invalid
 */
esp_err_t sensor_registry_get_sensor(uint8_t idx, const sensor_info_t **info,
                                      const sensor_ops_t **ops);

/**
 * @brief Print registered sensor list to log
 */
void sensor_registry_print(void);

/**
 * @brief Convert sensor data to ZCL attribute values
 *
 * @param[in]  data  Sensor data
 * @param[in]  cap   Capability to convert (CAP_TEMPERATURE, etc.)
 * @param[out] zcl_value Pointer to store ZCL-formatted value
 * @return ESP_OK if valid conversion, ESP_ERR_INVALID_STATE if not available
 */
esp_err_t sensor_data_to_zcl(const sensor_data_t *data, sensor_capability_t cap,
                              void *zcl_value);

/*=============================================================================
 * SENSOR REGISTRATION MACRO
 *
 * Each sensor driver calls this to register itself. Uses linker section
 * trick to auto-collect all sensors at compile time.
 *============================================================================*/

/**
 * @brief Register a sensor driver
 *
 * Usage in sensor driver .c file:
 *   SENSOR_REGISTER(my_sensor, my_sensor_ops);
 */
#define SENSOR_REGISTER(name, ops_ptr) \
    static const sensor_ops_t *sensor_##name##_ptr \
        __attribute__((used, section(".sensor_registry"))) = (ops_ptr)

/*=============================================================================
 * INDIVIDUAL SENSOR DRIVER EXTERNS (selected via Kconfig)
 *============================================================================*/

/* Each sensor driver provides this symbol */
#ifdef CONFIG_SENSOR_SHT30
extern const sensor_ops_t sht30_sensor_ops;
#endif
#ifdef CONFIG_SENSOR_SHT4X
extern const sensor_ops_t sht4x_sensor_ops;
#endif
#ifdef CONFIG_SENSOR_AHT20
extern const sensor_ops_t aht20_sensor_ops;
#endif
#ifdef CONFIG_SENSOR_DHT22
extern const sensor_ops_t dht22_sensor_ops;
#endif
#ifdef CONFIG_SENSOR_INTERNAL
extern const sensor_ops_t internal_sensor_ops;
#endif
#ifdef CONFIG_SENSOR_STUB
extern const sensor_ops_t stub_sensor_ops;
#endif

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_REGISTRY_H */
