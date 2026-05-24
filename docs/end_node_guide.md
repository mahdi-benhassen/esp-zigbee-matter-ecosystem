# Universal End Node Architecture Guide

The universal end node codebase is designed with a plugin architecture, enabling one firmware image to support multiple sensor hardware options. The specific sensor driver and pins are selected at build time using the ESP-IDF Kconfig (`menuconfig`) utility.

---

## Universal Sensor Registry (`sensor_universal`)

The registry abstracts sensor interactions using a common interface declared in [sensor_registry.h](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/end_node/components/sensor_universal/include/sensor_registry.h) and implemented in [sensor_registry.c](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/end_node/components/sensor_universal/sensor_registry.c).

### Operation Interface (`sensor_ops_t`)
Each sensor driver implements a function pointer table containing:
- **`init`**: Configures the I2C interface, GPIO lines, and registers the sensor.
- **`read`**: Reads physical metrics (Temperature, Humidity, etc.) from the device.
- **`sleep`**: Puts the sensor hardware into its lowest power sleep state.
- **`wakeup`**: Wakes the sensor hardware to prepare for new readings.
- **`get_info`**: Retrieves capabilities, precision margins, and device name metadata.

---

## Dynamic ZCL Cluster Registration (`zcl_clusters`)

Zigbee Cluster Library (ZCL) clusters must match the physical capabilities of the selected sensor. Rather than hardcoding Zigbee endpoints and clusters, [zcl_cluster_config.c](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/end_node/components/zcl_clusters/zcl_cluster_config.c) auto-generates the ZCL attributes based on the sensor capabilities registered at runtime:
- **Temperature Measurement Cluster (0x0402)**: Registered if the sensor has temperature capabilities.
- **Humidity Measurement Cluster (0x0405)**: Registered if the sensor has humidity capabilities.
- **Basic & Identify Clusters**: Default housekeeping clusters registered for all devices.

---

## Included Sensor Drivers (`sensor_drivers`)

1. **Sensirion SHT4x (I2C)**: High precision industrial sensor. Uses dynamic CRC checks.
2. **Sensirion SHT30 (I2C)**: Standard digital temperature and humidity sensor.
3. **Aosong AHT20 (I2C)**: Popular calibration-free budget sensor.
4. **DHT22 / AM2302 (1-Wire)**: Bit-banged timing-sensitive one-wire protocol.
5. **Internal Temp Sensor**: Reads the ESP32 chip's on-die silicon temperature.
6. **Stub Sensor**: A simulated sensor performing a random-walk algorithm, useful for testing network reliability and cluster reporting without hardware attachments.

---

## Low Power Management (Sleepy End Device)

To achieve multi-year battery life, the firmware implements aggressive power management:
- **Tickless Idle**: Configures FreeRTOS to enter a tickless sleep state (`CONFIG_FREERTOS_USE_TICKLESS_IDLE=y`).
- **Zigbee Polling**: Puts the ZBOSS radio stack into sleep between reporting intervals, waking up briefly to poll the coordinator for parent messages.
- **Hardware Power Down**: Powers down the internal flash and unused peripherals during the idle phase.
