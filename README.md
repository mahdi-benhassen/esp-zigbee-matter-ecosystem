# ESP-Zigbee + Matter IoT Ecosystem

## v1.1.0 - Dual-SoC Gateway + Universal Sensor Node

A comprehensive smart home IoT ecosystem built on Espressif ESP32 platform using
the official `esp-zigbee-sdk` (v2.x ZBOSS stack) and ESP-IDF v5.x.

## Detailed Documentation

For in-depth explanations of the system components, refer to the following guides:
- [Dual-SoC Gateway Architecture Guide](docs/gateway_guide.md): Details the host/RCP layout, state machines, and registry.
- [Universal End Node Architecture Guide](docs/end_node_guide.md): Details the plugin driver architecture, dynamic ZCL registration, and sleepy end device low power configurations.
- [CI/CD Pipeline & Build Matrix Guide](docs/cicd_guide.md): Details the automated GitHub Actions compiler runs and release mechanisms.

---

## What's New in v1.1.0

### Dual-SoC Gateway
- **ESP32-S3** as Wi-Fi Host + Application processor
- **ESP32-H2** as 802.15.4 Radio Co-Processor (RCP) via UART
- Full RCP UART diagnostics and recovery
- Dedicated `rcp_uart` component with state machine

### Universal Sensor Node
- **One codebase** for ALL sensor types
- Sensor selected at **build time via menuconfig** - no code changes
- ZCL clusters **auto-generated** from sensor capabilities
- 6 sensor drivers included: SHT30, SHT4x, AHT20, DHT22, Internal, Stub

---

## Project Structure

```
esp-zigbee-matter-ecosystem/
|-- CMakeLists.txt                           # Root
|-- README.md
|
|-- gateway/                                 # DUAL-SOC GATEWAY
|   |-- CMakeLists.txt                       # ESP32-S3 target
|   |-- sdkconfig.defaults                   # Dual-SoC defaults
|   |-- partitions.csv                       # 8MB flash
|   |
|   |-- main/
|   |   |-- CMakeLists.txt
|   |   |-- gateway_main.c                   # Entry + Wi-Fi + FSM
|   |   |-- Kconfig.projbuild               # Wi-Fi, UART RCP config
|   |   |-- idf_component.yml
|   |
|   |-- components/
|   |   |-- rcp_uart/                        # ESP32-H2 RCP UART interface
|   |   |   |-- include/rcp_uart.h           # RCP state machine, diagnostics
|   |   |   |-- rcp_uart.c                   # UART driver, reset, heartbeat
|   |   |   |-- CMakeLists.txt
|   |   |
|   |   |-- network_fsm/                     # NETWORK FSM (15 states, 35 events)
|   |       |-- include/network_fsm.h        # States, events, node registry
|   |       |-- network_fsm.c                # Full FSM + Dual-SoC RCP states
|   |       |-- CMakeLists.txt
|   |   |
|   |   |-- web_server/                      # [Stage 2] HTTP/WebSocket
|   |   |-- matter_bridge/                   # [Stage 2] Matter bridge
|
|-- end_node/                                # UNIVERSAL SENSOR NODE
|   |-- CMakeLists.txt                       # ESP32-H2 target
|   |-- sdkconfig.defaults                   # SED defaults
|   |-- partitions.csv                       # 4MB flash
|   |
|   |-- main/
|   |   |-- CMakeLists.txt
|   |   |-- end_node_main.c                  # Universal FSM (sensor-agnostic)
|   |   |-- Kconfig.projbuild               # SENSOR SELECTION menu
|   |   |-- idf_component.yml
|   |
|   |-- components/
|   |   |-- sensor_universal/                # SENSOR PLUGIN ARCHITECTURE
|   |   |   |-- include/sensor_registry.h    # Cap flags, ops interface, registry
|   |   |   |-- sensor_registry.c            # Init, read_all, sleep/wakeup
|   |   |   |-- CMakeLists.txt
|   |   |
|   |   |-- sensor_drivers/                  # ALL SENSOR DRIVERS
|   |   |   |-- CMakeLists.txt               # Conditional compilation
|   |   |   |
|   |   |   |-- stub/                        # Simulation (fallback)
|   |   |   |   |-- stub_sensor.c            # Random walk T+H
|   |   |   |
|   |   |   |-- internal/                    # ESP32 on-die temp
|   |   |   |   |-- internal_sensor.c        # Internal temp sensor
|   |   |   |
|   |   |   |-- sht30/                       # Sensirion SHT30 (I2C)
|   |   |   |   |-- sht30_sensor.c           # CRC8, high precision
|   |   |   |
|   |   |   |-- sht4x/                       # Sensirion SHT40/SHT41 (I2C)
|   |   |   |   |-- sht4x_sensor.c           # Best accuracy
|   |   |   |
|   |   |   |-- aht20/                       # Aosong AHT20 (I2C)
|   |   |   |   |-- aht20_sensor.c           # Budget option
|   |   |   |
|   |   |   |-- dht22/                       # DHT22/AM2302 (1-wire)
|   |   |   |   |-- dht22_sensor.c           # Bit-banged protocol
|   |   |
|   |   |-- zcl_clusters/                    # DYNAMIC ZCL REGISTRATION
|   |       |-- include/zcl_cluster_config.h # Auto-cluster API
|   |       |-- zcl_cluster_config.c         # Cluster creation from caps
|   |       |-- CMakeLists.txt
```

---

## Hardware: Dual-SoC Gateway

### ESP32-S3 (Host) <-> ESP32-H2 (RCP) Wiring

| Signal | ESP32-S3 | ESP32-H2 | Note |
|--------|----------|----------|------|
| UART TX | GPIO5 | GPIO8 (RX) | S3 -> H2 |
| UART RX | GPIO4 | GPIO9 (TX) | S3 <- H2 |
| 3.3V | 3.3V | 3.3V | Power |
| GND | GND | GND | Ground |

### Flash RCP Firmware (ESP32-H2)
```bash
cd $IDF_PATH/examples/openthread/ot_rcp
idf.py set-target esp32h2 flash
```

---

## Building

### 1. Gateway (ESP32-S3 + ESP32-H2)

```bash
cd gateway
idf.py set-target esp32s3
idf.py menuconfig   # Set Wi-Fi SSID/password, verify UART pins
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. End Node - Select Sensor via Menuconfig

```bash
cd end_node
idf.py set-target esp32h2

# CRITICAL: Select your sensor here!
idf.py menuconfig
#   -> Component config -> Universal End Node Configuration
#      -> Sensor Selection -> [X] Sensirion SHT40/SHT41 (I2C)
#      -> I2C Configuration: SDA=GPIO6, SCL=GPIO7
#      -> Power Mode: Sleepy End Device (SED)
#      -> Sleep Duration: 30 seconds

idf.py build
idf.py -p /dev/ttyUSB1 flash monitor
```

### Build Examples for Different Sensors

```bash
# Temperature + Humidity with SHT41 (I2C)
cd end_node && idf.py menuconfig  # Select SHT4X

# Temperature + Humidity with DHT22 (1-wire)
cd end_node && idf.py menuconfig  # Select DHT22, set GPIO pin

# Temperature only (internal sensor)
cd end_node && idf.py menuconfig  # Select Internal

# No sensor - simulation for testing
cd end_node && idf.py menuconfig  # Select Stub/Simulation
```

---

## Sensor Driver Matrix

| Sensor | Interface | Temp | Humidity | Pressure | Best For |
|--------|-----------|------|----------|----------|----------|
| SHT30 | I2C | +/- 0.2C | +/- 2% RH | - | General purpose |
| SHT4x | I2C | +/- 0.1C | +/- 1.8% RH | - | **Best accuracy** |
| AHT20 | I2C | +/- 0.3C | +/- 2% RH | - | Budget builds |
| DHT22 | 1-wire GPIO | +/- 0.5C | +/- 2-5% RH | - | Basic setups |
| Internal | On-chip | +/- 1C | - | - | Cost zero (die temp) |
| Stub | None | Simulated | Simulated | - | Development |

### Adding a New Sensor (3 steps)

1. **Create driver file**: `sensor_drivers/<name>/<name>_sensor.c`
2. **Implement ops**: init, read, sleep, wakeup, get_info, deinit
3. **Add Kconfig**: menu option in `main/Kconfig.projbuild`

The sensor_registry auto-discovers and auto-registers ZCL clusters.

---

## FSM Architecture

### Gateway FSM (15 states)
```
INIT -> HW_INIT -> RCP_INIT -> RCP_READY -> ZB_INIT -> NET_FORMATION
                                                          |
                              NET_RECOVER <---------------+
                                  |
                    NODE_JOINING -> NODE_OPERATIONAL -> NET_OPERATIONAL
                                                          |
                     NODE_REJOIN <- NODE_LEAVING <-------+
                                                          |
                                    NET_FAILED -> NET_SHUTDOWN
```

### End Node FSM (11 states)
```
INIT -> HW_INIT -> SENSORS_INIT -> ZB_INIT -> NETWORK_SCAN
                                                  |
                    REJOINING <-------------------+
                        |
                    JOINING -> JOINED -> OPERATIONAL [-> SLEEP -> wake]
                                                  |
                                    LEAVING -> SHUTDOWN
```

---

## CI/CD Build Matrix (GitHub Actions)

```yaml
# Build all sensor variants in parallel
strategy:
  matrix:
    sensor: [stub, internal, sht30, sht4x, aht20, dht22]
steps:
  - name: Configure sensor
    run: |
      cd end_node
      idf.py set-target esp32h2
      # Set CONFIG_SENSOR_${{ matrix.sensor }}=y in sdkconfig
  - name: Build
    run: idf.py build
```

---

## Stage Roadmap

| Stage | Component | Status |
|-------|-----------|--------|
| Stage 1 | Dual-SoC Gateway FSM + RCP UART | **Complete** |
| Stage 1 | Universal Sensor Node + Auto ZCL | **Complete** |
| Stage 2 | HTTP Server + REST API | Pending |
| Stage 2 | Matter-Zigbee Bridge | Pending |
| Stage 3 | OTA Updates for both targets | Pending |
| Stage 3 | Advanced ZCL Reporting (configurable intervals) | Pending |
| Stage 4 | Web Dashboard (mesh visualization) | Pending |

---

## License

Provided as-is for educational and development purposes.
