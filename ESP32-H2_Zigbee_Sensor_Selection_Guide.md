
# ESP32-H2 Zigbee Node — Sensor Selection Guide
## Hardware-Aware Selection for Battery-Powered Agricultural/Poultry Nodes

---

## ESP32-H2 Hardware Constraints Summary

| Resource | Specification | Implication |
|----------|-------------|-------------|
| **GPIOs** | 19 usable (GPIO0-5, 8-14, 22-27) | Limited to ~3-4 sensors per node |
| **ADC Channels** | 5 (GPIO1-5) | Max 5 analog sensors |
| **I2C Buses** | 2 (any GPIO via matrix) | Best for multi-sensor chains |
| **SPI** | 1 (FSPI: GPIO0/4/5) | For SD cards, displays |
| **UART** | 2 (GPIO23/24 = UART0, any for UART1) | For RS-485, GPS, LoRa |
| **Deep Sleep Current** | ~15μA | Critical for battery life |
| **Active TX Current** | ~74mA | Short bursts only |
| **Average Current** | ~333μA (10min sleep, 3s active) | 3500mAh battery = ~437 days |
| **LP GPIOs** | GPIO8-14, 22 (RTC wake capable) | For interrupt-driven sensors |

**Safe GPIOs (no strapping/USB conflicts):** GPIO0, 1, 4, 5, 10, 11, 12, 22 citeweb_search:7#1

**Strapping pins (avoid or use carefully):** GPIO2, 3, 8, 9, 25 citeweb_search:7#5

---

## Recommended I2C Pin Mapping for ESP32-H2

```
I2C Bus 0 (Primary Sensors):
  SDA = GPIO12 (safe, LP-capable)
  SCL = GPIO22 (safe, LP-capable)

I2C Bus 1 (Secondary/Expansion):
  SDA = GPIO10 (safe, LP-capable)
  SCL = GPIO11 (safe, LP-capable)

ADC Channels (Analog Sensors):
  ADC1_CH0 = GPIO1
  ADC1_CH1 = GPIO2 (strapping - use with care)
  ADC1_CH2 = GPIO3 (strapping - use with care)
  ADC1_CH3 = GPIO4
  ADC1_CH4 = GPIO5

1-Wire (Temperature chain):
  Data = GPIO0 (safe, also FSPIQ)

Status LED:
  RGB = GPIO8 (on-board, strapping but safe by design)
```

---

## 1. Climate Sensors (Air Temperature + Humidity)

### PRIMARY CHOICE: Sensirion SHT4xI Series (I2C)

| Part Number | Accuracy | Sleep Current | Active Current | Price | Why Choose |
|-------------|----------|---------------|----------------|-------|------------|
| **SHT45I-AD1B** | ±1.0% RH, ±0.1°C | 0.4μA | 0.6mA | $6-10 | **Best accuracy for premium nodes** |
| **SHT41I-AD1B** | ±1.8% RH, ±0.2°C | 0.4μA | 0.6mA | $4-6 | **Best price/performance balance** |
| **SHT40I-AD1B** | ±1.8% RH, ±0.2°C | 0.4μA | 0.6mA | $3-5 | **Cost-optimized high volume** |

**Key Advantages for ESP32-H2:**
- **Ultra-low power**: 0.4μA sleep — negligible impact on 333μA average budget
- **3.3V native**: Direct connection, no level shifting
- **I2C fast mode**: Up to 1MHz, compatible with ESP32-H2 I2C matrix
- **Wettable flank ("I" suffix)**: Reliable soldering for outdoor nodes
- **NIST traceable**: Calibration certificates for commercial deployments
- **Forced mode**: Sensor sleeps between readings, ESP32-H2 controls timing citeweb_search:7#2

**Wiring:**
```
SHT4xI          ESP32-H2
VDD (1)  ---->  3.3V
SDA (2)  ---->  GPIO12 (SDA)
GND (3)  ---->  GND
SCL (4)  ---->  GPIO22 (SCL)
```

**Code Example (ESP-IDF):**
```c
#include "sht4x.h"

sht4x_dev_t sht_sensor;
sht4x_init_desc(&sht_sensor, 0, SHT4X_I2C_ADDR_1, GPIO12, GPIO22);
sht4x_init(&sht_sensor);

// Read in forced mode (sensor sleeps after reading)
float temperature, humidity;
sht4x_measure(&sht_sensor, &temperature, &humidity);
```

---

### ALTERNATIVE: SHT30-DIS (Budget Nodes)

| Part Number | Accuracy | Sleep Current | Price | Use Case |
|-------------|----------|---------------|-------|----------|
| **SHT30-DIS** | ±2.0% RH, ±0.2°C | 0.2μA | $2-4 | High-volume deployment, cost-sensitive |

---

### AVOID FOR BATTERY NODES:
- **DHT22/AM2302**: Single-wire protocol, 2.5mA average, poor low-power performance
- **SHT20**: Older generation, higher drift, no wettable flank

---

## 2. CO₂ Sensors (Greenhouse / Poultry Air Quality)

### PRIMARY CHOICE: Sensirion SCD4x Series (I2C)

| Part Number | Range | Accuracy | Active Current | Sleep Current | Price | Why Choose |
|-------------|-------|----------|----------------|---------------|-------|------------|
| **SCD41** | 400-5000 ppm | ±40 ppm + 5% | 15mA (meas) | 0.2mA | $18-30 | **Extended range for poultry houses** |
| **SCD40** | 400-2000 ppm | ±40 ppm + 5% | 15mA (meas) | 0.2mA | $15-25 | **Standard greenhouse enrichment** |

**Key Advantages for ESP32-H2:**
- **Single I2C bus**: Shares SDA/SCL with SHT4xI (same I2C bus)
- **Low-power periodic measurement**: SCD41 supports single-shot mode
- **3.3V native**: No level shifting
- **Integrated RH/T compensation**: Uses internal RH/T sensor, or can use external SHT4xI for better accuracy
- **Auto-calibration**: ASC algorithm reduces drift

**Power Budget Impact:**
- Single-shot measurement: 15mA for ~5s every 30s → 2.5mA average
- With 10-minute sleep cycle: ~0.1mA average contribution
- **Total node average: ~1.5mA** (still viable for 6+ months on 18650)

**Wiring (Shared I2C Bus):**
```
SCD41           ESP32-H2
VDD      ---->  3.3V
SDA      ---->  GPIO12 (SDA)  [shared with SHT4xI]
GND      ---->  GND
SCL      ---->  GPIO22 (SCL)  [shared with SHT4xI]
```

**I2C Address Conflict Resolution:**
- SHT4xI: 0x44 (fixed)
- SCD4x: 0x62 (fixed)
- **No conflict! Both can coexist on same bus**

---

### ALTERNATIVE: Amphenol Telaire T6703 (I2C/UART)

| Part Number | Range | Accuracy | Interface | Price | Notes |
|-------------|-------|----------|-----------|-------|-------|
| **T6703-5K** | 0-5000 ppm | ±75 ppm + 5% | I2C/UART | $40-60 | **Industrial grade, NDIR** |
| **T6703** | 0-2000 ppm | ±75 ppm + 5% | I2C/UART | $35-50 | Standard indoor |

**⚠️ WARNING for Poultry:** T6703 uses NDIR optics — NH₃, Cl₂, NOx, or Ozone can corrode the optical chamber. **Not recommended for poultry houses with high ammonia.** Use SCD41 instead. citeweb_search:7#0

---

## 3. Ammonia (NH₃) Sensors — Poultry Critical

### PRIMARY CHOICE: Winsen ZE03-NH3 (UART)

| Part Number | Range | Accuracy | Interface | Active Current | Price | Why Choose |
|-------------|-------|----------|-----------|----------------|-------|------------|
| **ZE03-NH3** | 0-100 ppm | ±5% FS | UART/Analog | ~10mA (heater cycled) | $30-50 | **Best balance for battery nodes** |

**Key Advantages for ESP32-H2:**
- **UART interface**: Uses ESP32-H2 UART1 (GPIO10/11 or any GPIO)
- **Cycled heater**: Can power down between readings (unlike MQ-137)
- **Pre-calibrated**: Factory calibrated, minimal field calibration
- **Compact**: 20mm diameter, fits in node enclosure

**Power Management Strategy:**
```c
// Power ZE03 via GPIO-controlled MOSFET
#define ZE03_POWER_GPIO  GPIO11

void read_nh3(void) {
    gpio_set_level(ZE03_POWER_GPIO, 1);  // Power on
    vTaskDelay(pdMS_TO_TICKS(30000));     // 30s warm-up

    // Read via UART
    uint16_t nh3_ppb = ze03_read();

    gpio_set_level(ZE03_POWER_GPIO, 0);  // Power off
}
```
- **Average current**: 10mA × 30s / 600s = 0.5mA
- **Battery impact**: Minimal with 10-minute cycle

**Wiring:**
```
ZE03-NH3        ESP32-H2
VCC      ---->  GPIO11 (via MOSFET)  [Power control]
TXD      ---->  GPIO10 (RX)            [UART1]
RXD      ---->  GPIO12 (TX)            [UART1]
GND      ---->  GND
```

---

### BUDGET ALTERNATIVE: Winsen MQ-137 (Analog)

| Part Number | Range | Accuracy | Interface | Active Current | Price |
|-------------|-------|----------|-----------|----------------|-------|
| **MQ-137** | 10-300 ppm | ±10% FS | Analog (0-5V) | **150mA continuous** | $15-25 |

**⚠️ CRITICAL: MQ-137 is NOT for battery nodes**
- Heater draws **150mA continuous** — drains 3500mAh battery in ~23 hours
- **Only use for mains-powered nodes** or with external 5V supply
- If used: Power via external 5V, read analog via voltage divider to GPIO1 (ADC1_CH0)

---

### PREMIUM ALTERNATIVE: Membrapor NH3/C-200 (4-20mA)

| Part Number | Range | Accuracy | Interface | Price | Use Case |
|-------------|-------|----------|-----------|-------|----------|
| **NH3/C-200** | 0-200 ppm | ±2% FS | 4-20mA | $200-350 | **Premium poultry houses, research** |

**Requires**: External 4-20mA loop resistor (250Ω) + ADC reading on GPIO1

---

## 4. Soil Moisture + Temperature

### PRIMARY CHOICE: DS18B20 Chain (1-Wire) + DIY Capacitive

| Part Number | Parameter | Range | Accuracy | Interface | Sleep Current | Price |
|-------------|-----------|-------|----------|-----------|---------------|-------|
| **DS18B20** | Temperature | -55 to +125°C | ±0.5°C | 1-Wire | 750nA | $2-4 |
| **DS18B20-PAR** | Temperature (parasite) | -55 to +125°C | ±0.5°C | 1-Wire | 750nA | $2-4 |
| **Capacitive Soil (DIY)** | VWC | 0-100% | ±5% | Analog | 0μA | $3-5 |

**Key Advantages for ESP32-H2:**
- **1-Wire protocol**: Single GPIO (GPIO0) supports unlimited sensors in chain
- **Parasite power**: DS18B20-PAR uses data line for power — no separate VDD wire
- **Ultra-low sleep**: 750nA — negligible power budget impact
- **Long cable runs**: 1-Wire supports 100m+ with proper pull-up

**Wiring:**
```
DS18B20 Chain   ESP32-H2
DQ (1)   ---->  GPIO0 (1-Wire data, 4.7kΩ pull-up to 3.3V)
VDD (2)  ---->  3.3V (or parasite mode: connect to GND)
GND (3)  ---->  GND

Capacitive Soil ESP32-H2
VCC      ---->  3.3V
SIGNAL   ---->  GPIO1 (ADC1_CH0)
GND      ---->  GND
```

**Code Example:**
```c
#include "owb.h"
#include "ds18b20.h"

// Initialize 1-Wire bus on GPIO0
owb_rmt_driver_info rmt_driver_info;
OneWireBus *owb = owb_rmt_initialize(&rmt_driver_info, GPIO0, 
                                       RMT_CHANNEL_1, RMT_CHANNEL_0);
owb_use_crc(owb, true);

// Discover devices
OneWireBus_ROMCode rom_codes[MAX_DEVICES];
unsigned num_devices = owb_search_rom(owb, rom_codes, MAX_DEVICES);

// Read temperatures
DS18B20_Info *devices[MAX_DEVICES];
for (int i = 0; i < num_devices; i++) {
    devices[i] = ds18b20_malloc();
    ds18b20_init(devices[i], owb, rom_codes[i]);
    ds18b20_use_crc(devices[i], true);
    ds18b20_set_resolution(devices[i], DS18B20_RESOLUTION_12_BIT);
}

ds18b20_convert_all(owb);
ds18b20_wait_for_conversion(devices[0]);

float temps[MAX_DEVICES];
for (int i = 0; i < num_devices; i++) {
    ds18b20_read_temp(devices[i], &temps[i]);
}
```

---

### PREMIUM ALTERNATIVE: METER TEROS 12 (SDI-12)

| Part Number | Parameters | Range | Accuracy | Interface | Price |
|-------------|------------|-------|----------|-----------|-------|
| **TEROS 12** | VWC, EC, T | VWC: 0-0.70 m³/m³, EC: 0-20,000 μS/cm, T: -40 to +60°C | VWC: ±0.03, EC: ±5%, T: ±0.5°C | SDI-12 | $350-450 |

**Challenge for ESP32-H2:** SDI-12 requires bit-banged GPIO or translator chip
**Solution**: Use SDI-12 to UART translator (e.g., SDI12-USB) or bit-bang on GPIO10

---

## 5. Soil pH / EC (Precision Agriculture)

### PRIMARY CHOICE: Atlas Scientific EZO-pH + EZO-EC (I2C)

| Part Number | Parameter | Range | Accuracy | Interface | Active Current | Price |
|-------------|-----------|-------|----------|-----------|----------------|-------|
| **EZO-pH** | pH | 0-14 | ±0.002 | I2C/UART | ~15mA | $80-120 |
| **EZO-EC** | EC | 0-500,000 μS/cm | ±2% | I2C/UART | ~15mA | $80-120 |

**Key Advantages for ESP32-H2:**
- **Isolated I2C**: No ground loop issues with soil probes
- **3.3V compatible**: Direct connection
- **Built-in calibration**: Store calibration in EZO chip, not ESP32-H2
- **Temperature compensation**: Automatic when paired with temperature sensor

**Wiring (Shared I2C Bus with SHT4xI + SCD41):**
```
EZO-pH/EC       ESP32-H2
VCC      ---->  3.3V
SDA      ---->  GPIO12 (SDA)  [shared bus]
GND      ---->  GND
SCL      ---->  GPIO22 (SCL)  [shared bus]
```

**I2C Address Configuration:**
- EZO circuits default to 0x63 (pH) and 0x64 (EC)
- **No conflict with SHT4xI (0x44) or SCD4x (0x62)**

---

## 6. Light / PAR Sensors

### PRIMARY CHOICE: BH1750FVI / VEML7700 (I2C)

| Part Number | Range | Accuracy | Interface | Sleep Current | Price |
|-------------|-------|----------|-----------|---------------|-------|
| **VEML7700** | 0.004-83,000 lux | ±10% | I2C | 0.5μA | $2-3 |
| **BH1750FVI** | 1-65,535 lux | ±20% | I2C | 1μA | $1-2 |

**Wiring (Shared I2C Bus):**
```
VEML7700        ESP32-H2
VCC      ---->  3.3V
SDA      ---->  GPIO12 (SDA)  [shared bus]
GND      ---->  GND
SCL      ---->  GPIO22 (SCL)  [shared bus]
```

**I2C Address:** VEML7700 = 0x10, BH1750 = 0x23 (configurable) — **no conflicts**

---

### PREMIUM: Apogee SQ-420 (Analog/USB)

| Part Number | Range | Accuracy | Interface | Price |
|-------------|-------|----------|-----------|-------|
| **SQ-420** | 0-4000 μmol/m²/s | ±5% | Analog (mV) / USB | $300-400 |

**For ESP32-H2**: Use analog output (0-2.5V) → GPIO1 (ADC1_CH0) with voltage divider if needed

---

## 7. Water Level / Flow (Irrigation)

### PRIMARY CHOICE: YF-S201 (Pulse) + MS5837-30BA (I2C)

| Part Number | Parameter | Range | Accuracy | Interface | Price |
|-------------|-----------|-------|----------|-----------|-------|
| **YF-S201** | Flow rate | 1-30 L/min | ±10% | Pulse (freq) | $5-10 |
| **MS5837-30BA** | Pressure/Depth | 0-30 bar | ±0.2% | I2C | $15-25 |

**YF-S201 Wiring:**
```
YF-S201         ESP32-H2
Red (VCC)---->  5V (external, not ESP32-H2 3.3V)
Yellow     ---->  GPIO10 (Pulse counter input)
Black      ---->  GND
```

**MS5837-30BA Wiring (Shared I2C):**
```
MS5837          ESP32-H2
VCC      ---->  3.3V
SDA      ---->  GPIO12 (SDA)  [shared bus]
GND      ---->  GND
SCL      ---->  GPIO22 (SCL)  [shared bus]
```

---

## 8. Multi-Gas / VOC Proxy (Budget Poultry)

### PRIMARY CHOICE: ScioSense ENS160 + HDC1080 (I2C)

| Part Number | Parameters | Range | Accuracy | Interface | Sleep Current | Price |
|-------------|------------|-------|----------|-----------|---------------|-------|
| **ENS160** | eCO₂, TVOC, AQI | eCO₂: 400-65,000 ppm | ±15% | I2C | 0.5μA | $4-6 |
| **HDC1080** | T, RH | -40 to +125°C | ±0.2°C, ±2% RH | I2C | 0.1μA | $2-3 |

**Advantages:**
- **Ultra-low power**: Combined <1μA sleep
- **Single I2C bus**: Both on same bus (ENS160 = 0x53, HDC1080 = 0x40)
- **AI-compensated**: ENS160 uses humidity/temp from HDC1080 for compensation
- **Compact**: Both in 3×3mm DFN — fits small node PCBs

---

## 9. Complete Node Configurations

### Configuration A: Greenhouse Climate Node (Battery, 6+ months)

```
Sensors:
  1. SHT41I-AD1B      (I2C, 0x44)  — Air T/RH
  2. SCD41            (I2C, 0x62)  — CO₂
  3. VEML7700         (I2C, 0x10)  — Light
  4. DS18B20×3        (1-Wire)     — Soil/Canopy T

GPIO Mapping:
  GPIO12  → I2C_SDA (shared)
  GPIO22  → I2C_SCL (shared)
  GPIO0   → 1-Wire DQ (4.7kΩ pull-up)
  GPIO8   → RGB LED (status)
  GPIO1   → ADC (spare)

Power Budget:
  Deep sleep:     15μA
  SHT41I:         0.01μA (avg)
  SCD41:          100μA (avg, 10min cycle)
  VEML7700:       0.01μA (avg)
  DS18B20×3:      0.05μA (avg)
  ESP32-H2 active: 333μA (avg, 10min cycle)
  ─────────────────────────────
  Total average:   ~450μA
  Battery life:    ~320 days (3500mAh)
```

---

### Configuration B: Poultry House Node (Battery, 3+ months)

```
Sensors:
  1. SHT41I-AD1B      (I2C, 0x44)  — Air T/RH
  2. SCD41            (I2C, 0x62)  — CO₂
  3. ZE03-NH3         (UART1)      — Ammonia (cycled power)
  4. INMP441          (I2S)        — Sound level (stress indicator)

GPIO Mapping:
  GPIO12  → I2C_SDA (shared)
  GPIO22  → I2C_SCL (shared)
  GPIO10  → UART1_RX (ZE03 TX)
  GPIO11  → UART1_TX (ZE03 RX)
  GPIO11  → ZE03_POWER (MOSFET control)
  GPIO13  → I2S_WS (INMP441)
  GPIO14  → I2S_SD (INMP441)
  GPIO8   → RGB LED (status)

Power Budget:
  Deep sleep:      15μA
  SHT41I:          0.01μA (avg)
  SCD41:           100μA (avg)
  ZE03-NH3:        500μA (avg, 30s on/10min cycle)
  INMP441:         0.5μA (sleep)
  ESP32-H2 active: 333μA (avg)
  ─────────────────────────────
  Total average:   ~950μA
  Battery life:    ~150 days (3500mAh)
```

---

### Configuration C: Soil Monitoring Node (Battery, 6+ months)

```
Sensors:
  1. SHT40I-AD1B      (I2C, 0x44)  — Ambient T/RH (enclosure)
  2. DS18B20×5        (1-Wire)     — Soil temperature profile
  3. Capacitive VWC×2 (Analog)     — Soil moisture
  4. EZO-EC           (I2C, 0x64)  — Soil EC

GPIO Mapping:
  GPIO12  → I2C_SDA (shared)
  GPIO22  → I2C_SCL (shared)
  GPIO0   → 1-Wire DQ
  GPIO1   → ADC1_CH0 (Cap soil #1)
  GPIO4   → ADC1_CH3 (Cap soil #2)
  GPIO8   → RGB LED

Power Budget:
  Deep sleep:      15μA
  SHT40I:          0.01μA (avg)
  DS18B20×5:       0.08μA (avg)
  Capacitive:      0μA (passive)
  EZO-EC:          50μA (avg, 10min cycle)
  ESP32-H2 active: 333μA (avg)
  ─────────────────────────────
  Total average:   ~400μA
  Battery life:    ~360 days (3500mAh)
```

---

## 10. I2C Bus Address Map (Conflict-Free)

| Address | Device | Bus | Node Type |
|---------|--------|-----|-----------|
| 0x10 | VEML7700 (light) | I2C0 | Greenhouse |
| 0x23 | BH1750FVI (light) | I2C0 | Greenhouse |
| 0x40 | HDC1080 (T/RH) | I2C0 | Poultry (budget) |
| 0x44 | SHT4xI (T/RH) | I2C0 | All |
| 0x53 | ENS160 (VOC) | I2C0 | Poultry (budget) |
| 0x62 | SCD4x (CO₂) | I2C0 | Greenhouse/Poultry |
| 0x63 | EZO-pH | I2C0 | Soil |
| 0x64 | EZO-EC | I2C0 | Soil |
| 0x76 | BME280/BMP280 | I2C0 | General |
| 0x77 | BME680/BME688 | I2C0 | General |

**All addresses are unique — no conflicts on shared bus**

---

## 11. Power Management Best Practices

### GPIO-Controlled Sensor Power

```c
// Use GPIO to cut power to high-draw sensors
#define SENSOR_POWER_GPIO  GPIO11

void sensors_power_on(void) {
    gpio_set_direction(SENSOR_POWER_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(SENSOR_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));  // Stabilization
}

void sensors_power_off(void) {
    gpio_set_level(SENSOR_POWER_GPIO, 0);
    gpio_set_direction(SENSOR_POWER_GPIO, GPIO_MODE_INPUT);  // Hi-Z
}
```

### Deep Sleep Wake Strategy

```c
// Configure RTC GPIO wake (for interrupt sensors)
esp_sleep_enable_gpio_wakeup();
gpio_wakeup_enable(GPIO10, GPIO_INTR_LOW_LEVEL);  // Rain sensor

// Or timer wake for periodic sampling
esp_sleep_enable_timer_wakeup(10 * 60 * 1000000ULL);  // 10 minutes

// Enter deep sleep
esp_deep_sleep_start();
```

---

## 12. Procurement & Datasheet Links

| Manufacturer | Part Number | Datasheet | Supplier |
|-------------|-------------|-----------|----------|
| Sensirion | SHT41I-AD1B | [sensirion.com](https://sensirion.com) | Digi-Key, Mouser |
| Sensirion | SCD41 | [sensirion.com](https://sensirion.com) | Digi-Key, Mouser |
| Winsen | ZE03-NH3 | [winsen-sensor.com](https://winsen-sensor.com) | AliExpress, Direct |
| Maxim | DS18B20 | [maximintegrated.com](https://maximintegrated.com) | Digi-Key, Mouser |
| Vishay | VEML7700 | [vishay.com](https://vishay.com) | Digi-Key, Mouser |
| Atlas Scientific | EZO-pH | [atlas-scientific.com](https://atlas-scientific.com) | Direct |
| TE | MS5837-30BA | [te.com](https://te.com) | Digi-Key |
| ScioSense | ENS160 | [sciosense.com](https://sciosense.com) | Digi-Key |

---

*Document Version: 1.0 — ESP32-H2 Zigbee Node Optimized*
*Last Updated: 2026-05-26*
*Compatible with ESP-Zigbee-SDK, ESP-IDF v5.4+*
