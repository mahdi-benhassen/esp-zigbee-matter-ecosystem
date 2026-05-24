/**
 * @file zcl_cluster_config.h
 * @brief Dynamic ZCL Cluster Auto-Registration
 *
 * Creates ZCL clusters automatically based on sensor capabilities.
 * No hardcoded clusters - everything is derived from the sensor_registry
 * capability mask at runtime.
 *
 * This makes the firmware universal: compile once, select sensors at
 * build time, clusters are auto-configured.
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#ifndef ZCL_CLUSTER_CONFIG_H
#define ZCL_CLUSTER_CONFIG_H

#include "esp_err.h"
#include "sensor_registry.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * CONFIGURATION
 *============================================================================*/

/** Primary endpoint for sensor data */
#define ZCL_SENSOR_ENDPOINT     1

/** Identify endpoint */
#define ZCL_IDENTIFY_ENDPOINT   1

/** Profile ID - Home Automation */
#define ZCL_PROFILE_ID          0x0104

/*=============================================================================
 * CLUSTER REGISTRATION API
 *============================================================================*/

/**
 * @brief Initialize ZCL clusters based on sensor capabilities
 *
 * Reads the sensor_registry capability mask and creates the appropriate
 * ZCL clusters:
 *   - Basic (always)
 *   - Identify (always)
 *   - Power Config (always for battery devices)
 *   - Temperature Measurement (if CAP_TEMPERATURE)
 *   - Relative Humidity (if CAP_HUMIDITY)
 *   - Pressure Measurement (if CAP_PRESSURE)
 *   - Illuminance Measurement (if CAP_ILLUMINANCE)
 *   - Occupancy Sensing (if CAP_OCCUPANCY)
 *   - On/Off (if CAP_ONOFF)
 *   - Level Control (if CAP_LEVEL_CONTROL)
 *
 * @return ESP_OK on success
 */
esp_err_t zcl_clusters_init(void);

/**
 * @brief Update ZCL attribute values from sensor data
 *
 * Called after sensor_registry_read_all() to push new values
 * into the ZCL attribute table (triggers automatic reporting).
 *
 * @param[in] data Latest sensor readings
 * @return ESP_OK on success
 */
esp_err_t zcl_clusters_update_from_sensors(const sensor_data_t *data);

/**
 * @brief Get the ZCL device ID based on primary capability
 *
 * Maps the highest-priority capability to a HA device ID:
 *   - TEMPERATURE -> Temperature Sensor (0x0302)
 *   - HUMIDITY    -> Humidity Sensor (0x0307)
 *   - PRESSURE    -> Pressure Sensor (0x0305)
 *   - ILLUMINANCE -> Light Sensor (0x0106)
 *   - OCCUPANCY   -> Occupancy Sensor (0x0107)
 *   - ONOFF       -> On/Off Light (0x0100)
 *
 * @return ZCL device ID
 */
uint16_t zcl_clusters_get_device_id(void);

/**
 * @brief Get human-readable device type name
 * @return Static string
 */
const char *zcl_clusters_get_device_name(void);

/**
 * @brief Print active cluster configuration to log
 */
void zcl_clusters_print_config(void);

#ifdef __cplusplus
}
#endif

#endif /* ZCL_CLUSTER_CONFIG_H */
