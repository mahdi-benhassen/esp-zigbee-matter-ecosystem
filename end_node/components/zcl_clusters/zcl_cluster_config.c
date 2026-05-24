/**
 * @file zcl_cluster_config.c
 * @brief Dynamic ZCL Cluster Auto-Registration Implementation
 *
 * Creates ZCL clusters at runtime based on sensor_registry capabilities.
 * Zero hardcoded clusters - everything derived from sensor selection.
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "zcl_cluster_config.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"
#include <string.h>
#include <math.h>

#define TAG "ZCL_AUTO"

/*=============================================================================
 * MODULE STATE
 *============================================================================*/

static uint32_t s_active_caps = 0;
static esp_zb_ep_list_t *s_ep_list = NULL;
static esp_zb_cluster_list_t *s_cluster_list = NULL;

/* ZCL value cache for reporting */
static int16_t  s_zcl_temperature = 0x8000;
static uint16_t s_zcl_humidity = 0xFFFF;
static int16_t  s_zcl_pressure = 0x8000;
static uint16_t s_zcl_illuminance = 0xFFFF;
static uint8_t  s_zcl_occupancy = 0;
static uint8_t  s_zcl_onoff = 0;
static uint8_t  s_zcl_level = 0;

/*============================================================================
 * CLUSTER CREATION HELPERS
 *============================================================================*/

/**
 * @brief Add Basic cluster (mandatory for all devices)
 */
static void add_basic_cluster(void)
{
    esp_zb_attribute_list_t *basic = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);

    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);

    uint8_t app_version = 0x01;
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);

    uint8_t stack_version = 0x02;
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_STACK_VERSION_ID, &stack_version);

    uint8_t hw_version = 0x01;
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_HW_VERSION_ID, &hw_version);

    char mfg[] = "ESP-IoT-Ecosystem";
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, mfg);

    /* Device model derived from capability */
    char model[32] = {0};
    snprintf(model, sizeof(model), "%s", zcl_clusters_get_device_name());
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model);

    char date[] = "20240520";
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID, date);

#ifdef CONFIG_POWER_SLEEPY
    uint8_t power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
#else
    uint8_t power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE;
#endif
    esp_zb_basic_cluster_add_attr(basic, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);

    esp_zb_cluster_list_add_basic_cluster(s_cluster_list, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Basic cluster");
}

/**
 * @brief Add Identify cluster (mandatory for HA profile)
 */
static void add_identify_cluster(void)
{
    esp_zb_attribute_list_t *identify = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(identify, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);
    esp_zb_cluster_list_add_identify_cluster(s_cluster_list, identify, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Identify cluster");
}

/**
 * @brief Add Power Configuration cluster (for battery monitoring)
 */
static void add_power_config_cluster(void)
{
#ifdef CONFIG_POWER_SLEEPY
    esp_zb_attribute_list_t *power = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);

    uint8_t battery_voltage = 30; /* 3.0V = 30 half-volts */
    uint8_t battery_size = ESP_ZB_ZCL_POWER_CONFIG_BATTERY_SIZE_AA;
    uint8_t battery_qty = 2;

    esp_zb_power_config_cluster_add_attr(power,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage);
    esp_zb_power_config_cluster_add_attr(power,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_SIZE_ID, &battery_size);
    esp_zb_power_config_cluster_add_attr(power,
        ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_QUANTITY_ID, &battery_qty);

    esp_zb_cluster_list_add_power_config_cluster(s_cluster_list, power, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Power Config cluster (battery)");
#endif
}

/**
 * @brief Add Temperature Measurement cluster
 */
static void add_temperature_cluster(void)
{
    esp_zb_attribute_list_t *temp = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT);

    int16_t measured = 0x8000;     /* Unknown initially */
    int16_t min_value = -4000;     /* -40.00 C */
    int16_t max_value = 8500;      /* +85.00 C */
    int16_t tolerance = 50;        /* 0.50 C */

    esp_zb_temperature_meas_cluster_add_attr(temp,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &measured);
    esp_zb_temperature_meas_cluster_add_attr(temp,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID, &min_value);
    esp_zb_temperature_meas_cluster_add_attr(temp,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID, &max_value);
    esp_zb_temperature_meas_cluster_add_attr(temp,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_ID, &tolerance);

    esp_zb_cluster_list_add_temperature_meas_cluster(s_cluster_list, temp, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Temperature Measurement cluster");
}

/**
 * @brief Add Relative Humidity Measurement cluster
 */
static void add_humidity_cluster(void)
{
    esp_zb_attribute_list_t *hum = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT);

    uint16_t measured = 0xFFFF;    /* Unknown initially */
    uint16_t min_value = 0;        /* 0% */
    uint16_t max_value = 10000;    /* 100.00% */
    uint16_t tolerance = 300;      /* 3.00% */

    esp_zb_humidity_meas_cluster_add_attr(hum,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &measured);
    esp_zb_humidity_meas_cluster_add_attr(hum,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_ID, &min_value);
    esp_zb_humidity_meas_cluster_add_attr(hum,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MAX_VALUE_ID, &max_value);
    esp_zb_humidity_meas_cluster_add_attr(hum,
        ESP_ZB_ZCL_ATTR_REL_HUMIDITY_TOLERANCE_ID, &tolerance);

    esp_zb_cluster_list_add_humidity_meas_cluster(s_cluster_list, hum, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Relative Humidity cluster");
}

/**
 * @brief Add Pressure Measurement cluster
 */
static void add_pressure_cluster(void)
{
    esp_zb_attribute_list_t *press = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT);

    int16_t measured = 0x8000;
    int16_t min_value = 3000;      /* 300 hPa */
    int16_t max_value = 11000;     /* 1100 hPa */
    int16_t tolerance = 10;        /* 1 hPa */

    esp_zb_pressure_meas_cluster_add_attr(press,
        ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID, &measured);
    esp_zb_pressure_meas_cluster_add_attr(press,
        ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MIN_VALUE_ID, &min_value);
    esp_zb_pressure_meas_cluster_add_attr(press,
        ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_MAX_VALUE_ID, &max_value);
    esp_zb_pressure_meas_cluster_add_attr(press,
        ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_TOLERANCE_ID, &tolerance);

    esp_zb_cluster_list_add_pressure_meas_cluster(s_cluster_list, press, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Pressure Measurement cluster");
}

/**
 * @brief Add Illuminance Measurement cluster
 */
static void add_illuminance_cluster(void)
{
    esp_zb_attribute_list_t *ill = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT);

    uint16_t measured = 0xFFFF;    /* Unknown */
    uint16_t min_value = 1;        /* 1 = 0.0001 lux in ZCL */
    uint32_t max_value = 0xFFFE;

    esp_zb_illuminance_meas_cluster_add_attr(ill,
        ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &measured);
    esp_zb_illuminance_meas_cluster_add_attr(ill,
        ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE_ID, &min_value);
    esp_zb_illuminance_meas_cluster_add_attr(ill,
        ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE_ID, &max_value);

    esp_zb_cluster_list_add_illuminance_meas_cluster(s_cluster_list, ill, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Illuminance Measurement cluster");
}

/**
 * @brief Add Occupancy Sensing cluster
 */
static void add_occupancy_cluster(void)
{
    esp_zb_attribute_list_t *occ = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);

    uint8_t occupancy = 0;         /* Unoccupied */
    uint8_t sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_PIR;

    esp_zb_occupancy_sensing_cluster_add_attr(occ,
        ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &occupancy);
    esp_zb_occupancy_sensing_cluster_add_attr(occ,
        ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ID, &sensor_type);

    esp_zb_cluster_list_add_occupancy_sensing_cluster(s_cluster_list, occ, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + Occupancy Sensing cluster");
}

/**
 * @brief Add On/Off cluster (for relay/switch nodes)
 */
static void add_onoff_cluster(void)
{
    esp_zb_attribute_list_t *onoff = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_ON_OFF);

    uint8_t on_off = 0;            /* Off */
    esp_zb_on_off_cluster_add_attr(onoff, ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &on_off);

    esp_zb_cluster_list_add_on_off_cluster(s_cluster_list, onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    ESP_LOGI(TAG, "  + On/Off cluster");
}

/*=============================================================================
 * PUBLIC API
 *============================================================================*/

esp_err_t zcl_clusters_init(void)
{
    s_active_caps = sensor_registry_get_capabilities();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Dynamic ZCL Cluster Registration");
    ESP_LOGI(TAG, "Capabilities: 0x%08X", s_active_caps);
    ESP_LOGI(TAG, "========================================");

    /* Create cluster list and endpoint list */
    s_cluster_list = esp_zb_zcl_cluster_list_create();
    s_ep_list = esp_zb_ep_list_create();

    /* Always add mandatory clusters */
    add_basic_cluster();
    add_identify_cluster();
    add_power_config_cluster();

    /* Add sensor-specific clusters based on capabilities */
    if (s_active_caps & CAP_TEMPERATURE)   add_temperature_cluster();
    if (s_active_caps & CAP_HUMIDITY)      add_humidity_cluster();
    if (s_active_caps & CAP_PRESSURE)      add_pressure_cluster();
    if (s_active_caps & CAP_ILLUMINANCE)   add_illuminance_cluster();
    if (s_active_caps & CAP_OCCUPANCY)     add_occupancy_cluster();
    if (s_active_caps & CAP_ONOFF)         add_onoff_cluster();

    /* Create endpoint config */
    esp_zb_endpoint_config_t ep_config = {
        .endpoint = ZCL_SENSOR_ENDPOINT,
        .app_profile_id = ZCL_PROFILE_ID,
        .app_device_id = zcl_clusters_get_device_id(),
        .app_device_version = 0,
    };

    esp_zb_ep_list_add_ep(s_ep_list, s_cluster_list, ep_config);

    /* Register with Zigbee device */
    esp_zb_device_register(s_ep_list);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ZCL registration complete");
    ESP_LOGI(TAG, "  Endpoint: %d", ZCL_SENSOR_ENDPOINT);
    ESP_LOGI(TAG, "  Profile:  HA (0x%04X)", ZCL_PROFILE_ID);
    ESP_LOGI(TAG, "  Device:   %s (0x%04X)",
             zcl_clusters_get_device_name(), zcl_clusters_get_device_id());
    ESP_LOGI(TAG, "========================================");

    return ESP_OK;
}

esp_err_t zcl_clusters_update_from_sensors(const sensor_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;

    /* Temperature */
    if ((s_active_caps & CAP_TEMPERATURE) && data->temperature.valid) {
        int16_t new_val = (int16_t)(data->temperature.value.f32 * 100.0f);
        if (new_val != s_zcl_temperature) {
            s_zcl_temperature = new_val;
            esp_zb_zcl_set_attribute_val(ZCL_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                &s_zcl_temperature, false);
        }
    }

    /* Humidity */
    if ((s_active_caps & CAP_HUMIDITY) && data->humidity.valid) {
        uint16_t new_val = (uint16_t)(data->humidity.value.f32 * 100.0f);
        if (new_val != s_zcl_humidity) {
            s_zcl_humidity = new_val;
            esp_zb_zcl_set_attribute_val(ZCL_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                &s_zcl_humidity, false);
        }
    }

    /* Pressure */
    if ((s_active_caps & CAP_PRESSURE) && data->pressure.valid) {
        int16_t new_val = (int16_t)(data->pressure.value.f32 * 10.0f);
        if (new_val != s_zcl_pressure) {
            s_zcl_pressure = new_val;
            esp_zb_zcl_set_attribute_val(ZCL_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_PRESSURE_MEASUREMENT_VALUE_ID,
                &s_zcl_pressure, false);
        }
    }

    /* Illuminance */
    if ((s_active_caps & CAP_ILLUMINANCE) && data->illuminance.valid) {
        float lux = data->illuminance.value.f32;
        if (lux <= 0) lux = 1.0f;
        uint16_t new_val = (uint16_t)(10000.0f * log10f(lux) + 1.0f);
        if (new_val != s_zcl_illuminance) {
            s_zcl_illuminance = new_val;
            esp_zb_zcl_set_attribute_val(ZCL_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
                &s_zcl_illuminance, false);
        }
    }

    /* Occupancy */
    if ((s_active_caps & CAP_OCCUPANCY) && data->occupancy.valid) {
        uint8_t new_val = data->occupancy.value.b ? 1 : 0;
        if (new_val != s_zcl_occupancy) {
            s_zcl_occupancy = new_val;
            esp_zb_zcl_set_attribute_val(ZCL_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                &s_zcl_occupancy, false);
        }
    }

    return ESP_OK;
}

uint16_t zcl_clusters_get_device_id(void)
{
    /* Return device ID based on highest-priority capability */
    if (s_active_caps & CAP_TEMPERATURE)   return ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID;
    if (s_active_caps & CAP_HUMIDITY)      return ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID; /* No dedicated humidity device ID in SDK */
    if (s_active_caps & CAP_PRESSURE)      return 0x0305; /* Pressure Sensor */
    if (s_active_caps & CAP_ILLUMINANCE)   return ESP_ZB_HA_LIGHT_SENSOR_DEVICE_ID;
    if (s_active_caps & CAP_OCCUPANCY)     return ESP_ZB_HA_IAS_ZONE_ID;
    if (s_active_caps & CAP_ONOFF)         return ESP_ZB_HA_ON_OFF_SWITCH_DEVICE_ID;
    return ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID;
}

const char *zcl_clusters_get_device_name(void)
{
    if (s_active_caps & CAP_TEMPERATURE)   return "MultiSensor-Temp";
    if (s_active_caps & CAP_HUMIDITY)      return "HumiditySensor";
    if (s_active_caps & CAP_PRESSURE)      return "PressureSensor";
    if (s_active_caps & CAP_ILLUMINANCE)   return "LightSensor";
    if (s_active_caps & CAP_OCCUPANCY)     return "OccupancySensor";
    if (s_active_caps & CAP_ONOFF)         return "OnOffSwitch";
    return "GenericSensor";
}

void zcl_clusters_print_config(void)
{
    ESP_LOGI(TAG, "ZCL Configuration:");
    ESP_LOGI(TAG, "  Device: %s (0x%04X)", zcl_clusters_get_device_name(),
             zcl_clusters_get_device_id());
    ESP_LOGI(TAG, "  Active capabilities: 0x%08X", s_active_caps);
    ESP_LOGI(TAG, "  Clusters registered: %d", ZCL_SENSOR_ENDPOINT);
}
