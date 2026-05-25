#include "zigbee_config.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_zigbee.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl/esp_zigbee_zcl_common.h"
#include "zcl/esp_zigbee_zcl_basic.h"

static const char *TAG = "ZB_CONFIG";

/*=============================================================================
 * STACK INITIALIZATION
 *============================================================================*/
void zigbee_sed_stack_init(void)
{
    ESP_LOGI(TAG, "Configuring Zigbee stack as Sleepy End Device (SED)");

    /* 1. Platform Config */
    esp_zb_platform_config_t plat_cfg = {
        .radio_config = {.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE},
        .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE},
    };
    ESP_ERROR_CHECK(esp_zb_platform_config(&plat_cfg));

    /* 2. End Device Config */
    esp_zb_cfg_t zb_cfg = {
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,
        .install_code_policy = false,
        .nwk_cfg = {
            .zed_cfg = {
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN,
                .keep_alive = 3000, /* 3s keep-alive poll interval */
            }
        }
    };
    esp_zb_init(&zb_cfg);
}

/*=============================================================================
 * ZBOSS DATA MODEL CONSTRUCTION
 *============================================================================*/
esp_err_t zigbee_data_model_init(void)
{
    ESP_LOGI(TAG, "Constructing ZBOSS Zigbee 3.0 Data Model...");

    /* Create the top level endpoint list */
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    if (!ep_list) {
        ESP_LOGE(TAG, "Failed to create endpoint list");
        return ESP_ERR_NO_MEM;
    }

    /*=========================================================================
     * ENDPOINT 1: Environmental Sensor Platform
     *========================================================================*/
    esp_zb_cluster_list_t *cluster_list_1 = esp_zb_zcl_cluster_list_create();
    if (!cluster_list_1) {
        ESP_LOGE(TAG, "Failed to create cluster list for EP1");
        return ESP_ERR_NO_MEM;
    }

    /* 1. Basic Cluster (Mandatory) */
    esp_zb_attribute_list_t *basic_attr = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    uint8_t zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE;
    uint8_t app_version = 1;
    uint8_t power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_BATTERY;
    char mfg_name[] = "Espressif";
    char model_id[] = "SED-SensorHub-H2";

    esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_ZCL_VERSION_ID, &zcl_version);
    esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &app_version);
    esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &power_source);
    esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, mfg_name);
    esp_zb_basic_cluster_add_attr(basic_attr, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, model_id);
    esp_zb_cluster_list_add_basic_cluster(cluster_list_1, basic_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* 2. Identify Cluster (Mandatory for HA standard) */
    esp_zb_attribute_list_t *identify_attr = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY);
    uint16_t identify_time = 0;
    esp_zb_identify_cluster_add_attr(identify_attr, ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &identify_time);
    esp_zb_cluster_list_add_identify_cluster(cluster_list_1, identify_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* 3. Power Configuration Cluster (Battery voltage reporting) */
    esp_zb_attribute_list_t *power_attr = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_POWER_CONFIG);
    uint8_t battery_voltage = 33; /* 3.3V = 33 in tenths of a volt */
    esp_zb_power_config_cluster_add_attr(power_attr, ESP_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &battery_voltage);
    esp_zb_cluster_list_add_power_config_cluster(cluster_list_1, power_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    /* 4. Temperature Measurement Cluster (BME280 Air Temperature) */
#if CONFIG_ENABLE_BME280
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
        .measured_value = 0x8000, /* Unknown initially */
        .min_value = -4000,       /* -40.00 °C */
        .max_value = 8500,        /* +85.00 °C */
    };
    esp_zb_attribute_list_t *temp_attr = esp_zb_temperature_meas_cluster_create(&temp_cfg);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list_1, temp_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

    /* 5. Relative Humidity Measurement Cluster (BME280 Humidity) */
#if CONFIG_ENABLE_BME280
    esp_zb_humidity_meas_cluster_cfg_t hum_cfg = {
        .measured_value = 0xFFFF, /* Unknown initially */
        .min_value = 0,           /* 0.00 % */
        .max_value = 10000,       /* 100.00 % */
    };
    esp_zb_attribute_list_t *hum_attr = esp_zb_humidity_meas_cluster_create(&hum_cfg);
    esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list_1, hum_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

    /* 6. Illuminance Measurement Cluster (BH1750 Light Intensity) */
#if CONFIG_ENABLE_BH1750
    esp_zb_illuminance_meas_cluster_cfg_t ill_cfg = {
        .measured_value = 0xFFFF, /* Unknown initially */
        .min_value = 1,
        .max_value = 0xFFFE,
    };
    esp_zb_attribute_list_t *ill_attr = esp_zb_illuminance_meas_cluster_create(&ill_cfg);
    esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list_1, ill_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

    /* 7. Carbon Dioxide Measurement Cluster (SCD41 CO2) */
#if CONFIG_ENABLE_SCD41
    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cfg = {
        .measured_value = ESP_ZB_ZCL_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_DEFAULT,
        .min_measured_value = 0.0f,
        .max_measured_value = 40000.0f,
    };
    esp_zb_attribute_list_t *co2_attr = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg);
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list_1, co2_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

    /* 8. Custom Agricultural Extension Cluster (Soil Moisture, Barometric Pressure, Depth, Weight) */
#if CONFIG_ENABLE_SOIL_MOISTURE || CONFIG_ENABLE_JSN_SR04T || CONFIG_ENABLE_HX711 || CONFIG_ENABLE_BME280
    esp_zb_attribute_list_t *agri_attr = esp_zb_zcl_attr_list_create(CLUSTER_ID_AGRI_EXTENSION);
    uint16_t init_soil = 0;
    uint16_t init_press = 0;
    uint16_t init_depth = 0;
    uint32_t init_weight = 0;

    esp_zb_custom_cluster_add_custom_attr(agri_attr, ATTR_ID_AGRI_SOIL_MOISTURE, ESP_ZB_ZCL_ATTR_TYPE_U16, 
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &init_soil);
    esp_zb_custom_cluster_add_custom_attr(agri_attr, ATTR_ID_AGRI_BARO_PRESSURE, ESP_ZB_ZCL_ATTR_TYPE_U16, 
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &init_press);
    esp_zb_custom_cluster_add_custom_attr(agri_attr, ATTR_ID_AGRI_SILO_DEPTH, ESP_ZB_ZCL_ATTR_TYPE_U16, 
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &init_depth);
    esp_zb_custom_cluster_add_custom_attr(agri_attr, ATTR_ID_AGRI_WEIGHT_SCALE, ESP_ZB_ZCL_ATTR_TYPE_U32, 
                                          ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &init_weight);
    
    esp_zb_cluster_list_add_custom_cluster(cluster_list_1, agri_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
#endif

    /* Add Endpoint 1 to list */
    esp_zb_endpoint_config_t ep_cfg_1 = {
        .endpoint = ENDPOINT_ENV_PLATFORM,
        .app_profile_id = ZCL_PROFILE_HA,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list_1, ep_cfg_1);

    /*=========================================================================
     * ENDPOINT 2: Root-Zone Temperature Sensor (DS18B20)
     *========================================================================*/
#if CONFIG_ENABLE_DS18B20
    esp_zb_cluster_list_t *cluster_list_2 = esp_zb_zcl_cluster_list_create();
    if (!cluster_list_2) {
        ESP_LOGE(TAG, "Failed to create cluster list for EP2");
        return ESP_ERR_NO_MEM;
    }

    esp_zb_temperature_meas_cluster_cfg_t root_temp_cfg = {
        .measured_value = 0x8000,
        .min_value = -5500,  /* -55.00 °C */
        .max_value = 12500,  /* +125.00 °C */
    };
    esp_zb_attribute_list_t *root_temp_attr = esp_zb_temperature_meas_cluster_create(&root_temp_cfg);
    esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list_2, root_temp_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t ep_cfg_2 = {
        .endpoint = ENDPOINT_ROOT_TEMP,
        .app_profile_id = ZCL_PROFILE_HA,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list_2, ep_cfg_2);
#endif

    /*=========================================================================
     * ENDPOINT 3: Winsen ZE03-NH3 Electrochemical Ammonia Gas Sensor
     *========================================================================*/
#if CONFIG_ENABLE_ZE03_NH3
    esp_zb_cluster_list_t *cluster_list_3 = esp_zb_zcl_cluster_list_create();
    if (!cluster_list_3) {
        ESP_LOGE(TAG, "Failed to create cluster list for EP3");
        return ESP_ERR_NO_MEM;
    }

    esp_zb_carbon_dioxide_measurement_cluster_cfg_t nh3_cfg = {
        .measured_value = ESP_ZB_ZCL_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_DEFAULT,
        .min_measured_value = 0.0f,
        .max_measured_value = 100.0f,
    };
    esp_zb_attribute_list_t *nh3_attr = esp_zb_carbon_dioxide_measurement_cluster_create(&nh3_cfg);
    esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(cluster_list_3, nh3_attr, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

    esp_zb_endpoint_config_t ep_cfg_3 = {
        .endpoint = ENDPOINT_AMMONIA_GAS,
        .app_profile_id = ZCL_PROFILE_HA,
        .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
        .app_device_version = 0,
    };
    esp_zb_ep_list_add_ep(ep_list, cluster_list_3, ep_cfg_3);
#endif

    /* 9. Register complete endpoint list with the stack */
    return esp_zb_device_register(ep_list);
}

/*=============================================================================
 * ATTRIBUTE REPORTING OPERATIONS
 *============================================================================*/
esp_err_t zigbee_report_sensor_data(const sensor_hub_data_t *data)
{
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Mapping collected sensor structures to ZCL and sending reports...");

    /* 1. BME280 Temperature and Humidity reports on EP1 */
#if CONFIG_ENABLE_BME280
    if (data->bme280.valid) {
        int16_t zcl_temp = (int16_t)(data->bme280.temperature * 100.0f);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                     &zcl_temp, true);

        uint16_t zcl_hum = (uint16_t)(data->bme280.humidity * 100.0f);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
                                     &zcl_hum, true);
    }
#endif

    /* 2. BH1750 Illuminance report on EP1 */
#if CONFIG_ENABLE_BH1750
    if (data->bh1750.valid) {
        float lux = data->bh1750.lux;
        if (lux < 1.0f) lux = 1.0f;
        uint16_t zcl_lux = (uint16_t)(10000.0f * log10f(lux) + 1.0f);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID,
                                     &zcl_lux, true);
    }
#endif

    /* 3. SCD41 CO2 report on EP1 */
#if CONFIG_ENABLE_SCD41
    if (data->scd41.valid) {
        /* CO2 MeasuredValue is represented as volume fraction (0.0 to 1.0) in standard ZCL. */
        /* Concentration (ppm) * 1e-6 = volume fraction */
        float zcl_co2 = data->scd41.co2 * 1e-6f;
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                                     &zcl_co2, true);
    }
#endif

    /* 4. DS18B20 Root-Zone Temperature report on EP2 */
#if CONFIG_ENABLE_DS18B20
    if (data->ds18b20.valid) {
        int16_t zcl_root_temp = (int16_t)(data->ds18b20.temperature * 100.0f);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ROOT_TEMP, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                     &zcl_root_temp, true);
    }
#endif

    /* 5. Winsen ZE03 NH3 Gas Density report on EP3 */
#if CONFIG_ENABLE_ZE03_NH3
    if (data->winsen_nh3.valid) {
        /* Winsen NH3 is reported as volume fraction */
        float zcl_nh3 = data->winsen_nh3.nh3 * 1e-6f;
        esp_zb_zcl_set_attribute_val(ENDPOINT_AMMONIA_GAS, ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
                                     &zcl_nh3, true);
    }
#endif

    /* 6. Custom Agricultural Extension Cluster reports on EP1 */
#if CONFIG_ENABLE_SOIL_MOISTURE || CONFIG_ENABLE_JSN_SR04T || CONFIG_ENABLE_HX711 || CONFIG_ENABLE_BME280
    #if CONFIG_ENABLE_SOIL_MOISTURE
    if (data->soil_moisture.valid) {
        uint16_t zcl_sm = (uint16_t)(data->soil_moisture.vwc * 100.0f);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, CLUSTER_ID_AGRI_EXTENSION,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ID_AGRI_SOIL_MOISTURE,
                                     &zcl_sm, true);
    }
    #endif

    #if CONFIG_ENABLE_BME280
    if (data->bme280.valid) {
        uint16_t zcl_pres = (uint16_t)(data->bme280.pressure * 10.0f);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, CLUSTER_ID_AGRI_EXTENSION,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ID_AGRI_BARO_PRESSURE,
                                     &zcl_pres, true);
    }
    #endif

    #if CONFIG_ENABLE_JSN_SR04T
    if (data->jsn_sr04t.valid) {
        uint16_t zcl_depth = (uint16_t)(data->jsn_sr04t.distance_cm);
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, CLUSTER_ID_AGRI_EXTENSION,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ID_AGRI_SILO_DEPTH,
                                     &zcl_depth, true);
    }
    #endif

    #if CONFIG_ENABLE_HX711
    if (data->hx711.valid) {
        uint32_t zcl_weight = (uint32_t)(data->hx711.weight_kg * 1000.0f); /* report weight in grams */
        esp_zb_zcl_set_attribute_val(ENDPOINT_ENV_PLATFORM, CLUSTER_ID_AGRI_EXTENSION,
                                     ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ATTR_ID_AGRI_WEIGHT_SCALE,
                                     &zcl_weight, true);
    }
    #endif
#endif

    ESP_LOGI(TAG, "All reports queued successfully");
    return ESP_OK;
}
