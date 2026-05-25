# Hardware Configuration Guide

This document details the hardware pin assignments for both the Dual-SoC Gateway and the Universal End Node within the ESP-Zigbee-Matter Ecosystem. It also provides instructions on how to select and enable specific sensors on the End Node.

---

## 1. Gateway Pin Assignment

The Gateway operates in a Dual-SoC architecture: an ESP32-S3 acts as the Wi-Fi host, and an ESP32-H2 acts as the Zigbee/Thread Radio Co-Processor (RCP) connected via UART (Spinel protocol).

### Gateway Host (ESP32-S3 CoreS3)
If using the M5Stack CoreS3 Grove PORT C interface, the pins are configured as follows:
- **UART TX (S3 -> H2):** `GPIO17`
- **UART RX (S3 <- H2):** `GPIO18`
- **Hardware Reset for RCP:** `GPIO9` (Connected to the EN pin of the ESP32-H2 to allow the host to hard-reset the RCP if the connection hangs).

### Gateway RCP (ESP32-H2 Unit)
The OpenThread RCP firmware running on the ESP32-H2 uses the following pins for UART communication with the host:
- **UART RX (H2 <- S3):** `GPIO23`
- **UART TX (H2 -> S3):** `GPIO24`

---

## 2. End Node Pin Assignment

The Universal End Node is built on the ESP32-H2 (e.g., ESP32-H2-DevKitM-1). The pin assignments have been carefully mapped to avoid conflicts with internal flash lines and boot strapping pins.

| Peripheral Interface         | Pins / Ports                          | Description                               |
| :---                         | :---                                  | :---                                      |
| **Sensor Power Gate**        | `GPIO4`                               | Drives the PMOS/VCC rail to power sensors |
| **I2C Bus (SDA / SCL)**      | `GPIO6` / `GPIO7`                     | Shared bus for BME280, BH1750, SCD41      |
| **One-Wire Bus**             | `GPIO5`                               | For DS18B20 Temperature Sensor            |
| **Soil Moisture ADC**        | `ADC1_CH1` (`GPIO2`)                  | Analog input for soil moisture reading    |
| **JSN-SR04T (Ultrasonic)**   | Trig: `GPIO0`, Echo: `GPIO1`          | Distance/Level measurement                |
| **Winsen ZE03-NH3 (UART)**   | TX: `GPIO24`, RX: `GPIO23` (Port 1)   | Gas concentration measurement             |
| **HX711 (Load Cell)**        | SCK: `GPIO10`, DOUT: `GPIO11`         | Weight measurement                        |

> **Note:** Do NOT connect external sensors to GPIO8, GPIO9, GPIO15, GPIO16, GPIO17, GPIO18, GPIO19, GPIO20, or GPIO21. These are dedicated to boot strapping and internal SiP SPI Flash. Using them will cause the ESP32-H2 to crash or fail to boot.

---

## 3. How to Select and Enable Sensors in the Node

The Universal End Node uses a highly modular architecture. Sensors can be enabled or disabled via configuration flags in the source code.

### Step 1: Enable Hardware Modules in `sensor_hub.h`
Open `end_node/main/sensor_hub.h` and locate the **SENSOR ENABLE CONFIGURATION** block. Set the macro for the sensor(s) you wish to include to `1` (enabled) or `0` (disabled):

```c
#define CONFIG_ENABLE_BME280        1
#define CONFIG_ENABLE_BH1750        1
#define CONFIG_ENABLE_SCD41         1
#define CONFIG_ENABLE_SOIL_MOISTURE 1
#define CONFIG_ENABLE_DS18B20       1
#define CONFIG_ENABLE_ZE03_NH3      1
#define CONFIG_ENABLE_JSN_SR04T     1
#define CONFIG_ENABLE_HX711         1
```

### Step 2: Configure via Menuconfig (Optional)
Additional basic sensors (like the SHT30, AHT20, or DHT22) and the data reporting intervals can be configured via the ESP-IDF `menuconfig` tool.

1. In your terminal, navigate to the `end_node` directory.
2. Run `idf.py menuconfig`.
3. Go to **Universal End Node Configuration** -> **Sensor Selection**.
4. Use the spacebar to check/uncheck the relevant sensors.
5. Save your changes and exit.

### Step 3: Rebuild and Flash
Once the desired sensors are enabled in both the header file and `menuconfig` (if applicable), clean and rebuild your project:

```bash
idf.py fullclean
idf.py build flash monitor
```

The node will automatically expose the corresponding Zigbee ZCL clusters (e.g., Temperature, Humidity, Illuminance, Carbon Dioxide, Analog Input) based on the sensors you enabled.
