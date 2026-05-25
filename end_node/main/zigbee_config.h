#pragma once

#include "esp_err.h"
#include "sensor_hub.h"

#ifdef __cplusplus
extern "C" {
#endif

/*=============================================================================
 * ZIGBEE DATA MODEL & ENDPOINT DEFINITIONS
 *============================================================================*/
#define ENDPOINT_ENV_PLATFORM       1   /* Endpoint for main environmental sensors */
#define ENDPOINT_ROOT_TEMP          2   /* Endpoint for DS18B20 Root-Zone temperature */
#define ENDPOINT_AMMONIA_GAS        3   /* Endpoint for Winsen ZE03 NH3 gas density */

#define ZCL_PROFILE_HA              0x0104  /* Home Automation (HA) Profile ID */

/* Custom / Proprietary Agricultural Cluster ID */
#define CLUSTER_ID_AGRI_EXTENSION   0xFF01  /* Manufacturer-specific cluster range */

/* Custom Cluster Attributes */
#define ATTR_ID_AGRI_SOIL_MOISTURE  0x0001  /* VWC% in hundredths of a percent (uint16) */
#define ATTR_ID_AGRI_BARO_PRESSURE  0x0002  /* hPa in tenths of a hPa (uint16) */
#define ATTR_ID_AGRI_SILO_DEPTH     0x0003  /* cm (uint16) */
#define ATTR_ID_AGRI_WEIGHT_SCALE   0x0004  /* Grams (uint32) */

/*=============================================================================
 * FUNCTION PROTOTYPES
 *============================================================================*/
/**
 * @brief Register all endpoints, clusters, and attributes with ZBOSS.
 *        Constructs a dynamic data model depending on enabled sensors.
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t zigbee_data_model_init(void);

/**
 * @brief Map captured sensor readings into their respective ZCL attributes
 *        and queue asynchronous attribute reports over Zigbee 3.0.
 * @param[in] data Pointer to the structure containing fresh sensor readings.
 * @return ESP_OK on success, or appropriate error code.
 */
esp_err_t zigbee_report_sensor_data(const sensor_hub_data_t *data);

/**
 * @brief Initialize basic configuration for sleepy end device.
 */
void zigbee_sed_stack_init(void);

#ifdef __cplusplus
}
#endif
