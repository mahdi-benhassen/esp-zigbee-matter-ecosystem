# Dual-SoC Gateway Architecture Guide

The gateway is built on a Dual-SoC hardware architecture, partitioning the application logic/Wi-Fi stack from the IEEE 802.15.4 low-power radio operations.

## Hardware Partitioning

- **ESP32-S3 (Host Application Processor)**:
  - Connects to Wi-Fi.
  - Runs the main Zigbee host stack (ZBOSS host mode) and Matter application stack.
  - Manages the network state machine, NVS configuration storage, and node registry.
  - Connects to the RCP via UART.
- **ESP32-H2 (Radio Co-Processor - RCP)**:
  - Runs the physical and MAC layers of the 802.15.4 stack (`ot_rcp` or `zboss_rcp` firmware).
  - Acts as a dumb radio transceiver, forwarding packets between the air and the S3 host via UART.

---

## RCP UART Driver Component (`rcp_uart`)

The gateway communicates with the ESP32-H2 RCP using a custom UART framing layer configured via [rcp_uart.h](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/gateway/main_soc/components/rcp_uart/include/rcp_uart.h) and [rcp_uart.c](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/gateway/main_soc/components/rcp_uart/rcp_uart.c).

### Features
- **State Tracking**: Monitors UART link state (Uninitialized, Ready, Error).
- **Heartbeat & Keepalive**: Periodically pings the RCP to verify hardware health.
- **Hardware Reset Line**: Automatically toggles the GPIO reset pin of the ESP32-H2 if communications fail, attempting automatic recovery.

---

## Network Finite State Machine (`network_fsm`)

The network FSM handles initialization, network formation, node joining, and recovery. It is defined in [network_fsm.h](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/gateway/main_soc/components/network_fsm/include/network_fsm.h) and implemented in [network_fsm.c](file:///c:/Users/MAHDI/Desktop/Agent_AI/esp-thread-br/esp-zigbee-matter-ecosystem/gateway/main_soc/components/network_fsm/network_fsm.c).

### Core States
1. **`NET_STATE_INIT`**: Initial state of the state machine.
2. **`NET_STATE_HW_INIT`**: Initializes ESP32-S3 peripherals (GPIO, UART, NVS).
3. **`NET_STATE_RCP_INIT`**: Handshakes with the ESP32-H2 co-processor.
4. **`NET_STATE_RCP_READY`**: RCP link is confirmed active.
5. **`NET_STATE_ZB_INIT`**: Configures ZBOSS Zigbee stack parameters.
6. **`NET_STATE_NET_FORMATION`**: Forms the Zigbee PAN Coordinator network on the designated channel.
7. **`NET_STATE_NET_READY`**: Network is formed and accepting node joins.
8. **`NET_STATE_NODE_JOINING`**: Processing a join handshake from an end device.
9. **`NET_STATE_NET_RECOVER`**: Triggered when RCP connection is lost; resets the co-processor and attempts recovery.

### Active Signals Handled
- **`ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE`**: Detects new node announcement.
- **`ESP_ZB_ZDO_SIGNAL_LEAVE_INDICATION`**: Detects when a node leaves the network, capturing the leaving node's EUI64 and short address to update the node registry.

---

## Node Registry

The gateway tracks connected devices in a local in-memory registry, which is persisted to NVS (Non-Volatile Storage) on change:
- **`node_info_t`**: Stores device EUI64, short 16-bit address, device endpoints, clusters, link quality indicators (LQI), received signal strength (RSSI), and sleep capability.
- **Auto-Commissioning**: When a node joins, it announces its capabilities which are dynamically mapped to active profiles.
