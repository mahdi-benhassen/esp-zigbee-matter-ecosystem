# ESP-Zigbee + Matter IoT Ecosystem

## v1.2.0 - Dual-SoC Gateway + Sleepy End Device Node

A comprehensive smart agricultural IoT ecosystem built on the Espressif ESP32 platform using the official `esp-zigbee-sdk` (v2.x ZBOSS stack) and ESP-IDF v5.x.

---

## Detailed Documentation

For in-depth explanations of the system components, refer to the following guides:
- [Dual-SoC Gateway Architecture Guide](docs/gateway_guide.md): Details the host/RCP layout, state machines, and registry.
- [Sleepy End Device Architecture Guide](docs/end_node_guide.md): Details the transient linear pipeline, power gating, low-level bit-banged drivers, and ZBOSS multi-endpoint/custom clusters.
- [CI/CD Pipeline & Build Matrix Guide](docs/cicd_guide.md): Details the automated GitHub Actions compiler runs and release mechanisms.

---

## What's New in v1.2.0

### Sleepy End Device (SED) Node
- **Strict Linear Pipeline**: Boot -> stabilization delay -> trigger conversions -> read sensors -> power gate off -> join network -> report ZCL -> sleep. 
- **Power Gating**: Designated Sensor Power Control Pin (`GPIO_NUM_4`) drives sensor VCC rails, preventing parasitic leak current during sleep.
- **Generic Modular Compilation**: Enable/disable specific sensors (`BME280`, `BH1750`, `SCD41`, `DS18B20`, `ZE03-NH3`, `JSN-SR04T`, `HX711`, `Soil Moisture`) at compile time via preprocessor macros in `sensor_hub.h`.
- **Multi-Endpoint Layout**: Prevents attribute collision by distributing sensors to distinct endpoints (Endpoint 1 for environmental sensors, Endpoint 2 for DS18B20 root temp, Endpoint 3 for Winsen NH3 gas).
- **Custom Agricultural Cluster**: Encapsulates Soil Moisture VWC%, Barometric Pressure, Silo Depth, and Scale Weight inside a proprietary cluster (`0xFF01`) on Endpoint 1.
- **RTC Retention cache**: Stores BME280 calibration parameters in RTC Slow RAM to bypass I2C read cycles on wake.
- **Deep Sleep target**: Achieves a ~7µA target sleep current.

---

## Project Structure

```
esp-zigbee-matter-ecosystem/
|-- CMakeLists.txt                           # Workspace Root
|-- README.md                                # This document
|
|-- gateway/                                 # DUAL-SOC GATEWAY
|   |-- CMakeLists.txt                       # ESP32-S3 target
|   |-- sdkconfig.defaults
|   |-- partitions.csv                       # 8MB flash
|   |-- main/
|   |   |-- gateway_main.c
|   |   |-- Kconfig.projbuild
|   |   |-- idf_component.yml
|
|-- end_node/                                # SLEEPY END NODE (ESP32-H2)
|   |-- CMakeLists.txt                       # ESP32-H2 target
|   |-- sdkconfig.defaults                   # SED defaults
|   |-- partitions.csv                       # 4MB flash
|   |-- main/
|   |   |-- CMakeLists.txt                   # Compiles main.c, sensor_hub.c, zigbee_config.c
|   |   |-- main.c                           # app_main, FSM signals, deep sleep pipeline
|   |   |-- sensor_hub.h                     # Pinout, macro config, data structures
|   |   |-- sensor_hub.c                     # Power gating & low-level drivers
|   |   |-- zigbee_config.h                  # Endpoint & custom cluster layout
|   |   |-- zigbee_config.c                  # ZBOSS registration and reports
```

---

## Sleepy End Node Sensor Pinout

| Sensor | Interface | ESP32-H2 Pins | Details |
|--------|-----------|---------------|---------|
| VCC Rail | Power Gate | GPIO4 | Drives PMOS or Load Switch |
| BME280 / BH1750 / SCD41 | I2C | SDA=GPIO6, SCL=GPIO7 | Shared I2C bus |
| DS18B20 | 1-Wire | GPIO5 | Bit-banged timing |
| Winsen ZE03-NH3 | UART1 | TX=GPIO24, RX=GPIO23 | 9600 bps Q&A mode |
| JSN-SR04T | GPIO | Trig=GPIO0, Echo=GPIO1 | Microsecond pulse timing |
| HX711 | 2-Wire Serial | SCK=GPIO10, DOUT=GPIO11 | Synchronous weight readings |
| Soil Moisture v1.2 | ADC1 | Channel 1 (GPIO2) | 12-bit SAR ADC one-shot |

---

## Building and Flashing

To build the Sleepy End Node firmware:
```bash
cd end_node
idf.py set-target esp32h2
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

---

## CI/CD Build Matrix (GitHub Actions)

Upon push or pull request to the `main` branch, the GitHub Actions runner compiles the firmware using an Espressif ESP-IDF Docker container (`v5.5.4` target).

---

## License

Provided as-is for educational and development purposes.
