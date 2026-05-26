/**
 * @file soil_moisture_sensor.c
 * @brief Capacitive Soil Moisture Sensor Driver (ADC-based)
 *
 * Reads a capacitive soil moisture probe via ESP32 ADC1 oneshot mode.
 * The sensor outputs an analog voltage inversely proportional to soil
 * moisture: higher voltage = drier soil, lower voltage = wetter soil.
 *
 * Calibration:
 *   - Dry (air):  ~2500 mV  → VWC = 0%
 *   - Wet (water): ~1100 mV → VWC = 100%
 *   - Linear interpolation between these two calibration points.
 *
 * ADC Configuration:
 *   - ADC_UNIT_1, channel via Kconfig (default channel 1 = GPIO2 on ESP32-H2)
 *   - ADC_ATTEN_DB_12 for full 0–3.3 V input range
 *   - Curve-fitting calibration scheme for accurate mV conversion
 *
 * Capabilities: CAP_SOIL_MOISTURE
 *
 * @author IoT Ecosystem Build
 * @version 1.1.0-universal
 */

#include "sensor_registry.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include <string.h>

#define TAG "SOIL_MOISTURE"

/*=============================================================================
 * KCONFIG DEFAULTS
 *
 * These macros are normally provided by Kconfig. The #ifndef guards allow the
 * driver to compile with sensible defaults when the Kconfig symbols have not
 * been generated yet (e.g., during early development or unit testing).
 *============================================================================*/
#ifndef CONFIG_SENSOR_SOIL_ADC_CHANNEL
#define CONFIG_SENSOR_SOIL_ADC_CHANNEL 1   /* ADC1 channel 1 = GPIO2 on ESP32-H2 */
#endif

/*=============================================================================
 * CALIBRATION CONSTANTS
 *
 * These define the two-point linear calibration curve. They should be tuned
 * per-sensor in production, but these defaults cover the common Capacitive
 * Soil Moisture Sensor v1.2 modules from the market.
 *============================================================================*/
#define SOIL_DRY_VOLTAGE_MV   2500.0f   /* Voltage when sensor is in air (0% VWC) */
#define SOIL_WET_VOLTAGE_MV   1100.0f   /* Voltage when sensor is in water (100% VWC) */

/*=============================================================================
 * MODULE STATE
 *============================================================================*/
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool s_cali_enabled   = false;
static bool s_initialized    = false;

/*=============================================================================
 * INIT
 *============================================================================*/
static esp_err_t soil_moisture_init(void)
{
    ESP_LOGI(TAG, "Soil moisture init: ADC_UNIT_1, CH%d, ATTEN=12dB",
             CONFIG_SENSOR_SOIL_ADC_CHANNEL);

    /* ---- 1. Create ADC1 oneshot unit ---- */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .clk_src  = 0,                    /* Default clock source */
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit create failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ---- 2. Configure the channel ---- */
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,      /* 0–3.3 V full-scale */
    };
    err = adc_oneshot_config_channel(s_adc_handle,
                                     (adc_channel_t)CONFIG_SENSOR_SOIL_ADC_CHANNEL,
                                     &chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return err;
    }

    /* ---- 3. Set up voltage calibration (best-effort) ---- */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = (adc_channel_t)CONFIG_SENSOR_SOIL_ADC_CHANNEL,
        .atten   = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle) == ESP_OK) {
        s_cali_enabled = true;
        ESP_LOGI(TAG, "ADC curve-fitting calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable – using raw approximation");
        s_cali_enabled = false;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Soil moisture sensor initialized");
    return ESP_OK;
}

/*=============================================================================
 * READ
 *============================================================================*/
static esp_err_t soil_moisture_read(sensor_data_t *data)
{
    if (!s_initialized || data == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* ---- 1. Sample the ADC ---- */
    int raw_val = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle,
                                      (adc_channel_t)CONFIG_SENSOR_SOIL_ADC_CHANNEL,
                                      &raw_val);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* ---- 2. Convert raw count → millivolts ---- */
    int voltage_mv = 0;
    if (s_cali_enabled) {
        adc_cali_raw_to_voltage(s_cali_handle, raw_val, &voltage_mv);
    } else {
        /* Fallback: simple linear mapping (12-bit ADC, 3300 mV full-scale) */
        voltage_mv = (raw_val * 3300) / 4095;
    }

    /* ---- 3. Linear interpolation → Volumetric Water Content (VWC %) ---- */
    float vwc = 100.0f * (SOIL_DRY_VOLTAGE_MV - (float)voltage_mv)
                        / (SOIL_DRY_VOLTAGE_MV - SOIL_WET_VOLTAGE_MV);

    /* Clamp to valid range */
    if (vwc > 100.0f) vwc = 100.0f;
    if (vwc < 0.0f)   vwc = 0.0f;

    /* ---- 4. Populate the output struct ---- */
    memset(data, 0, sizeof(sensor_data_t));
    data->soil_moisture.value.f32    = vwc;
    data->soil_moisture.valid        = true;
    data->soil_moisture.timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    ESP_LOGD(TAG, "Soil moisture: raw=%d, voltage=%dmV, VWC=%.1f%%",
             raw_val, voltage_mv, vwc);
    return ESP_OK;
}

/*=============================================================================
 * SLEEP / WAKEUP
 *
 * The ADC peripheral is stateless between reads in oneshot mode, so there is
 * no explicit low-power state to manage.  Returning ESP_OK keeps the
 * framework happy.
 *============================================================================*/
static esp_err_t soil_moisture_sleep(void)  { return ESP_OK; }
static esp_err_t soil_moisture_wakeup(void) { return ESP_OK; }

/*=============================================================================
 * DEINIT
 *============================================================================*/
static esp_err_t soil_moisture_deinit(void)
{
    if (s_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(s_cali_handle);
        s_cali_handle  = NULL;
        s_cali_enabled = false;
    }
    if (s_adc_handle) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
    }
    s_initialized = false;
    ESP_LOGI(TAG, "Soil moisture sensor de-initialized");
    return ESP_OK;
}

/*=============================================================================
 * METADATA
 *============================================================================*/
static const sensor_info_t s_info = {
    .name               = "Capacitive Soil Moisture",
    .model              = "CapSoilMoisture",
    .capabilities       = CAP_SOIL_MOISTURE,
    .min_interval_ms    = 2000,
    .default_interval_ms = 10000,
    .supports_sleep     = true,
};

static const sensor_info_t *soil_moisture_get_info(void) { return &s_info; }

/*=============================================================================
 * OPERATIONS TABLE & EXPORTED SYMBOL
 *============================================================================*/
static const sensor_ops_t soil_moisture_ops = {
    .init     = soil_moisture_init,
    .read     = soil_moisture_read,
    .sleep    = soil_moisture_sleep,
    .wakeup   = soil_moisture_wakeup,
    .get_info = soil_moisture_get_info,
    .deinit   = soil_moisture_deinit,
};

const sensor_ops_t soil_moisture_sensor_ops = soil_moisture_ops;
