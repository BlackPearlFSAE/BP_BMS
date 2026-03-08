# BMS Architecture & Signal Agreement

**Project:** BP16 FSAE BMS — KMUTT BlackPearlRacing Team
**Vehicle:** BP16 (FSAE 2025 and onward)

> For the complete signal table (CAN IDs, byte layout, scaling factors), see the
> [BMS Signals Agreement Spreadsheet](https://docs.google.com/spreadsheets/d/1TbN9bK0u2_f_2LeJMQZu6MZCuvkwzeiOJ2B9JoZcbQw/edit?gid=226357247#gid=226357247).
> This document covers architecture rationale, IC selection, and logic implementation.

---

## 1. System Overview

The accumulator pack uses **LG M50LT (LG34) 21700 Li-ion cells** arranged as:

- 10 cells in series per module → one BMU per module
- 8 modules in series → 80S total pack
- Nominal pack voltage: ~296 V (3.7 V × 80)
- Max pack voltage: ~336 V (4.2 V × 80)
- Min pack voltage: ~256 V (3.2 V × 80)

Three firmware targets run on separate MCUs:

| Target | MCU | Role |
|--------|-----|------|
| BMU (×7) | ESP32-C3 | Cell monitoring slave — reads LTC6811, reports via CAN |
| BCU (×1) | ESP32-S3 | Master aggregator — AMS_OK logic, charger control |
| Datalogger | ESP32-S3 | SD card + WiFi streaming (out of scope here) |

```
BMU 1-7 (ESP32-C3)          BCU (ESP32-S3)
  │                            │
  ├─ LTC6811 via SPI           ├─ CAN RX from BMU 1-7
  │  └─ 10 cells/module        ├─ Local LTC6811 (Module 8)
  ├─ 2× NTC thermistors        ├─ Aggregate every 500 ms
  ├─ CAN TX every 1000 ms      ├─ AMS_OK GPIO output
  │  (5 messages/module)       └─ OBC charger control (CAN)
  └─ CAN RX (BCU config)
```

---

## 2. Why Distributed BMS Architecture

A **distributed** (master-slave) BMS places a small monitoring node on each battery module instead of running all cell-tap wiring to one central board. The reasons this was chosen:

### 2.1 Wiring length and noise
An 80S series string physically spans the full length of the accumulator box. Routing individual cell-tap wires from every cell to a single centralized board would require dozens of long low-level voltage wires running inside a high-voltage enclosure. Long tap wires pick up switching noise from the inverter and motor controller and add significant wiring weight. A distributed BMU sits directly on each module — cell-tap traces are millimeters long on the PCB, and only the two-wire CAN bus needs to run the length of the car.

### 2.2 Identical, reusable hardware
All 7 BMU boards are identical hardware. Module number is assigned at flash time. Adding or removing modules requires only changing `MODULE_NUM` in firmware and reflashing the BCU. This simplifies both manufacturing and spare management.

### 2.3 Fault isolation
If a BMU's MCU or power supply fails, only that module is affected. The BCU detects the missing module within one timeout period (1500 ms) and pulls AMS_OK low. The remaining modules continue running independently and do not drop off the bus.

### 2.4 Cell-level fault granularity
Each BMU maintains a 10-bit fault bitmap for each fault type — one bit per cell. This means the BCU knows not only *that* an overvoltage exists, but *which exact cell* is faulting, across all 80 cells. A centralized design with one large AFE ICs would produce the same granularity, but at the cost of a more complex single-point-of-failure board.

### 2.5 FSAE EV rules compliance
FSAE EV Technical Regulations require per-cell voltage monitoring and a hardware AMS fault output (AMS_OK/AIR interlock). The distributed topology naturally provides per-cell monitoring at every module, with the BCU consolidating all data into a single AMS_OK signal that interfaces with the shutdown circuit.

---

## 3. Why LTC6811 for the BMU

The Analog Devices (formerly Linear Technology) **LTC6811-1** multi-cell battery stack monitor was selected for the following reasons:

### 3.1 Cell count fit
The LTC6811 measures up to **12 cells in series** per IC. With 10 cells per module, one IC per BMU is sufficient — no cascading or additional ICs needed per node.

### 3.2 Measurement resolution
The LTC6811 ADC provides **0.1 mV resolution** (raw code × 0.0001 V per LSB). The passive balancing threshold in this project is 10 mV above average — the IC resolution is well within that margin.

```cpp
// src/bmu.cpp
cellvoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001f;  // 0.1 mV per LSB
```

### 3.3 Built-in passive balancing control
The LTC6811 CFG register contains **DCC (Discharge Control Cell) bits** — one per cell. Writing a DCC bit enables a built-in discharge path through an external shunt resistor. No separate balancing IC or MOSFET driver is needed.

```cpp
// src/bmu.cpp — configure DCC bits for cells to balance
bms_ic[0].config.tx_data[4] = dischargeBits & 0xFF;           // DCC[7:0] — Cells 1-8
bms_ic[0].config.tx_data[5] = (bms_ic[0].config.tx_data[5] & 0xFC)
                             | ((dischargeBits >> 8) & 0x03); // DCC[9:8] — Cells 9-10
LTC6811_wrcfg(TOTAL_IC, bms_ic);
```

### 3.4 isoSPI daisy-chaining
The LTC6811 supports isoSPI — an isolated, differential serial interface that can daisy-chain multiple ICs across isolation barriers. Although this project uses one IC per BMU (not chained within a module), the interface is compatible with future multi-IC-per-module expansion.

### 3.5 Official Linduino driver
Analog Devices provides the **Linduino** Arduino-compatible driver (`lib/LTC6811/`, `lib/LTC681x/`, `lib/LT_SPI/`), which handles all register initialization, CRC computation, and ADC sequencing. This reduces firmware risk on the cell-measurement layer.

### 3.6 SPI hardware compatibility
The ESP32-C3 SPI peripheral supports Mode 3 (CPOL=1, CPHA=1) required by the LTC6811. The SPI clock is set to approximately 1.25 MHz (clock divider 16), within the LTC6811 maximum.

```cpp
// src/bmu.cpp
SPI.begin(4, 5, 6, 7);  // SCK=4, MISO=5, MOSI=6, CS=7
SPI.setDataMode(SPI_MODE3);
SPI.setClockDivider(SPI_CLOCK_DIV16);  // ~1.25 MHz
```

---

## 4. Fault Detection Logic

Fault detection is split across two layers: per-module (BMU) and system-level (BCU).

### 4.1 BMU — threshold calculation

Thresholds are recomputed whenever the BCU broadcasts updated parameters:

```cpp
// src/bmu.cpp — updateThresholds()
OV_WARNING_THRESHOLD  = 0.95f * VmaxCell;    // 3.99 V  (default Vmax = 4.2 V)
OV_CRITICAL_THRESHOLD = VmaxCell;            // 4.20 V
LV_WARNING_THRESHOLD  = VminCell + 0.2f;     // 3.40 V  (default Vmin = 3.2 V)
LV_CRITICAL_THRESHOLD = VminCell;            // 3.20 V
TEMP_WARNING_THRESHOLD  = 0.8f * TempMaxCell;  // 48 °C  (default Tmax = 60 °C)
TEMP_CRITICAL_THRESHOLD = 0.9f * TempMaxCell;  // 54 °C
DV_WARNING_THRESHOLD  = dVmax;               // 0.20 V  (default DVMAX = 0.2 V)
DV_CRITICAL_THRESHOLD = dVmax * 1.5f;        // 0.30 V
```

### 4.2 BMU — per-cell fault flag generation

Each fault type is stored as a **10-bit bitmask** (bit 9 = Cell 1, bit 0 = Cell 10).

```cpp
// src/bmu.cpp — updateFaultFlags()
void updateFaultFlags() {
  myBMU.OVERVOLTAGE_WARNING    = 0;
  myBMU.OVERVOLTAGE_CRITICAL   = 0;
  myBMU.LOWVOLTAGE_WARNING     = 0;
  myBMU.LOWVOLTAGE_CRITICAL    = 0;
  myBMU.OVERTEMP_WARNING       = 0;
  myBMU.OVERTEMP_CRITICAL      = 0;
  myBMU.OVERDIV_VOLTAGE_WARNING  = 0;
  myBMU.OVERDIV_VOLTAGE_CRITICAL = 0;

  float avgV = calculateAvgVoltage();

  for (int i = 0; i < NUM_CELLS; i++) {
    uint16_t cellBit = (1 << (9 - i));   // MSB-first in CAN frame

    // Over voltage
    if (cellvoltages[i] >= OV_CRITICAL_THRESHOLD)
      myBMU.OVERVOLTAGE_CRITICAL |= cellBit;
    else if (cellvoltages[i] > OV_WARNING_THRESHOLD)
      myBMU.OVERVOLTAGE_WARNING  |= cellBit;

    // Low voltage
    if (cellvoltages[i] <= LV_CRITICAL_THRESHOLD)
      myBMU.LOWVOLTAGE_CRITICAL  |= cellBit;
    else if (cellvoltages[i] < LV_WARNING_THRESHOLD)
      myBMU.LOWVOLTAGE_WARNING   |= cellBit;

    // Cell imbalance (delta from module average)
    float cellDV = fabs(cellvoltages[i] - avgV);
    if (cellDV >= DV_CRITICAL_THRESHOLD)
      myBMU.OVERDIV_VOLTAGE_CRITICAL |= cellBit;
    else if (cellDV >= DV_WARNING_THRESHOLD)
      myBMU.OVERDIV_VOLTAGE_WARNING  |= cellBit;
  }

  // Temperature: only 2 sensors per module — flag all 10 cells together
  float maxTemp = max(currentTemp1, currentTemp2);
  if (maxTemp >= TEMP_CRITICAL_THRESHOLD)
    myBMU.OVERTEMP_CRITICAL = 0x3FF;   // all 10 cells
  else if (maxTemp >= TEMP_WARNING_THRESHOLD)
    myBMU.OVERTEMP_WARNING  = 0x3FF;
}
```

### 4.3 BCU — system-level AMS_OK determination

The BCU aggregates fault flags from all modules with a bitwise OR, then evaluates AMS_OK:

```cpp
// src/bcu.cpp — runs every 500 ms after aggregation

// Aggregate: OR fault flags from all 7 (or 8) modules
for (int j = 0; j < MODULE_NUM; j++) {
  AMS_Package.OVERVOLT_CRITICAL  |= BMU_Package[j].OVERVOLTAGE_CRITICAL;
  AMS_Package.LOWVOLT_CRITICAL   |= BMU_Package[j].LOWVOLTAGE_CRITICAL;
  AMS_Package.OVERTEMP_CRITICAL  |= BMU_Package[j].OVERTEMP_CRITICAL;
  AMS_Package.OVERDIV_CRITICAL   |= BMU_Package[j].OVERDIV_VOLTAGE_CRITICAL;
  // (warnings aggregated similarly)
}

// AMS_OK: any critical fault from any cell → shutdown
bool ACCUMULATOR_Fault = (AMS_Package.OVERVOLT_CRITICAL  > 0)
                       || (AMS_Package.LOWVOLT_CRITICAL   > 0)
                       || (AMS_Package.OVERTEMP_CRITICAL  > 0)
                       || (AMS_Package.OVERDIV_CRITICAL   > 0);
AMS_OK = !ACCUMULATOR_Fault;

// If charger is plugged in, also gate on OBC status
if (CHARGER_PLUGGED) {
  uint16_t OBCFault = OBC_Package.OBCstatusbit;  // non-zero = OBC fault
  AMS_OK = !(ACCUMULATOR_Fault || OBCFault);
}

// Hardware output — feeds AIR interlock / shutdown circuit
digitalWrite(AMS_OUT, AMS_OK ? HIGH : LOW);
```

**Fault summary table:**

| Fault | Warning threshold | Critical threshold | AMS_OK effect |
|-------|-------------------|--------------------|---------------|
| Overvoltage | 3.99 V (0.95 × Vmax) | 4.20 V | Critical → AMS LOW |
| Undervoltage | 3.40 V (Vmin + 0.2) | 3.20 V | Critical → AMS LOW |
| Overtemperature | 48 °C (0.8 × Tmax) | 54 °C (0.9 × Tmax) | Critical → AMS LOW |
| Cell imbalance (ΔV) | 0.20 V | 0.30 V | Critical → AMS LOW + stops charging |
| BMU disconnected | — | 1500 ms no CAN frame | → AMS LOW |
| OBC fault | — | OBCstatusbit ≠ 0 | → AMS LOW (charging mode) |

---

## 5. OBC and Charging Shutdown Board Co-operation

### 5.1 Charger detection

The BCU reads `OBCIN` (GPIO 14) to detect whether a charger is physically connected. When `CHARGER_PLUGGED = true`, the BCU:
- Enables the OBC CAN transmission timer (500 ms period)
- Processes incoming OBC status frames
- Includes `OBCFault` in the AMS_OK gate

### 5.2 BCU → OBC control frame

The BCU sends a charger control message to CAN ID `0x1806E5F4` (OBC address, SAE J1939 style) every 500 ms while the charger is plugged in:

```cpp
// src/bcu.cpp — packBCU_toOBCmsg()
void packBCU_toOBCmsg(twai_message_t *BCUsent,
                      bool AMS_OK, bool ReadytoCharge,
                      bool OverDivCritical_Yes, bool Voltage_is_Full) {
  BCUsent->identifier      = OBC_ADD;  // 0x1806E5F4
  BCUsent->data_length_code = 8;

  bool allowCharge = AMS_OK
                  && ReadytoCharge
                  && !OverDivCritical_Yes
                  && !Voltage_is_Full;

  if (allowCharge) {
    BCUsent->data[0] = 0x18;  // Target voltage high byte  (240.0 V → 0x0960 × 0.1)
    BCUsent->data[1] = 0x00;  // Target voltage low byte
    BCUsent->data[2] = 0x00;  // Target current high byte
    BCUsent->data[3] = 0x64;  // Target current low byte   (10.0 A → 100 × 0.1)
    BCUsent->data[4] = 0x00;  // Control byte: charger OPERATE
  } else {
    BCUsent->data[0] = 0x00;
    BCUsent->data[1] = 0x00;
    BCUsent->data[2] = 0x00;
    BCUsent->data[3] = 0x00;
    BCUsent->data[4] = 0x01;  // Control byte: charger SHUTDOWN
  }
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;
}
```

**Charging is allowed only when ALL four conditions are simultaneously true:**

| Condition | Source |
|-----------|--------|
| `AMS_OK` | No critical voltage/temp fault on any cell |
| `ReadytoCharge` | Pack internal charge-ready state |
| `!OverDivCritical` | No critical cell imbalance (ΔV < 0.30 V) |
| `!Voltage_is_Full` | Pack voltage < 95% of max (~319 V) |

Any one condition failing sends `data[4] = 0x01` which commands the OBC to shut down.

### 5.3 OBC → BCU status frame

The OBC replies on its own CAN ID with a status byte. The BCU decodes this in `processReceived_OBCmsg()`:

```cpp
// src/bcu.cpp
void processReceived_OBCmsg(twai_message_t *receivedframe) {
  if (receivedframe->identifier != OBC_ADD) return;
  OBC_Package.OBCVolt      = mergeHLbyte(receivedframe->data[0], receivedframe->data[1]);
  OBC_Package.OBCAmp       = mergeHLbyte(receivedframe->data[2], receivedframe->data[3]);
  OBC_Package.OBCstatusbit = receivedframe->data[4];
  // OBCstatusbit: bit 4=Timeout, bit 3=NoBattery, bit 2=AC_Reversed,
  //               bit 1=Overheat, bit 0=HW_Fault
  OBC_Package.OBC_OK = (OBC_Package.OBCstatusbit == 0);
}
```

If `OBCstatusbit` is non-zero during a charge session, the BCU immediately incorporates that fault into the AMS_OK gate (see Section 4.3), pulling AMS_OK low and simultaneously commanding `data[4] = 0x01` on the next OBC frame — a double-redundant charging shutdown.

### 5.4 Charging shutdown sequence (summary)

```
Cell overvoltage / overtemp / imbalance
  OR pack full (≥95% Vmax)
  OR OBC reports a fault
       │
       ▼
BCU sets AMS_OK = false
       │
       ├─ GPIO AMS_OUT → LOW  (hardware AIR interlock opens)
       └─ CAN frame to OBC: data[4] = 0x01 (software shutdown)
```

---

## 6. Project Usage

### 6.1 Prerequisites

- [PlatformIO Core](https://docs.platformio.org/en/latest/core/installation/) (CLI) **or** VSCode with the PlatformIO extension
- All libraries are bundled under `lib/` — no additional package installation required

### 6.2 Build environments

`platformio.ini` defines three build targets sharing common compiler flags:

| Environment | MCU board | Purpose |
|-------------|-----------|---------|
| `bmu` | ESP32-C3-DevKitM-1 | Battery Monitoring Unit firmware |
| `bcu` | ESP32-S3-DevKitC-1 | Battery Control Unit firmware |
| `energy_meter` | ESP32 DevKit | RS485 energy meter (experimental) |

Each environment excludes the source files that belong to other targets via `build_src_filter`.

### 6.3 Build and flash

```bash
# Build only
pio run -e bmu
pio run -e bcu

# Build and upload
pio run -e bmu -t upload
pio run -e bcu -t upload

# Open serial monitor (115200 baud)
pio device monitor

# Build, upload, and monitor in one step
pio run -e bmu -t upload && pio device monitor
```

### 6.4 Project structure

```
FSAEbms_1/
├── src/
│   ├── bmu.cpp             # BMU firmware — cell reading, fault detection, CAN TX
│   ├── bcu.cpp             # BCU firmware — aggregation, AMS_OK, OBC control
│   ├── bms_helper.cpp      # CAN ID encoding, byte utilities
│   └── ntstermistor.cpp    # NTC thermistor → temperature conversion
│
├── include/
│   ├── bms_helper.h        # CAN ID format, helper function declarations
│   └── ntstermistor.h      # Thermistor interface
│
├── lib/
│   ├── ams_data_util/      # BMUdata, AMSdata, OBCdata structs; config constants
│   ├── LTC6811/            # LTC6811 cell monitor IC driver (Analog Devices)
│   ├── LTC681x/            # LTC681x register definitions (base class)
│   ├── LT_SPI/             # SPI layer for LTC chips
│   ├── Linduino/           # Arduino compatibility shims for Linduino library
│   ├── CAN32_util/         # ESP32 TWAI (CAN) send/receive wrapper
│   ├── SD32_util/          # SD card CSV logging
│   └── RS485-master/       # RS485 UART utility (energy meter use)
│
├── doc/
│   ├── bms_architecture.md          # This file
│   ├── bcu_issues_analysis.md       # Known BCU bugs and proposed fixes
│   ├── teleplot_debug_implementation.md
│   └── BP16B Signals Agreement.*    # Full CAN signal table (also on Google Sheets)
│
├── platformio.ini          # Build configuration
└── README.md               # Quick-start overview
```

### 6.5 Key configuration constants

All constants are in `lib/ams_data_util/ams_data_util.h`:

| Constant | Default | Description |
|----------|---------|-------------|
| `CELL_NUM` | 10 | Cells per module |
| `MODULE_NUM` | 7 | Number of BMU slaves |
| `VMAX_CELL` | 4.2 V | LG34 cell maximum |
| `VMIN_CELL` | 3.2 V | LG34 cell minimum |
| `DVMAX` | 0.2 V | Max allowed cell imbalance |
| `TEMP_MAX_CELL` | 60 °C | Cell temperature limit |
| `BMS_COMMUNICATE_TIME` | 1000 ms | BMU CAN TX interval |
| `OBC_COMMUNICATE_TIME` | 500 ms | BCU→OBC CAN TX interval |
| `DISCONNENCTION_TIMEOUT` | 1500 ms | BMU loss detection window |

### 6.6 Hardware pin assignments

**BMU (ESP32-C3):**

| Signal | GPIO |
|--------|------|
| SPI SCK | 4 |
| SPI MISO | 5 |
| SPI MOSI | 6 |
| LTC6811 CS | 7 |
| NTC Sensor 1 | 0 (ADC) |
| NTC Sensor 2 | 2 (ADC) |
| AMS_OUT | 3 |
| CAN TX | 20 |
| CAN RX | 21 |

**BCU (ESP32-S3):**

| Signal | GPIO |
|--------|------|
| CAN TX | 48 |
| CAN RX | 47 |
| AMS_OK output | 21 |
| Charger detect (OBCIN) | 14 |

### 6.7 CAN bus overview

- Bitrate: **250 kbps** (standard vehicle CAN)
- Frame type: **Extended 29-bit CAN ID**
- ID format: `0x18 [Priority:4] [SourceAddr:4] 00 [MsgNum:4]`
  - Priority `0x2` = module data (cell voltages, temperatures, balancing)
  - Priority `0x1` = fault codes (OV/LV/OT/ΔV bitmaps)
- BCU broadcast: `0x18000000` (config, balancing enable, AMS_OK)
- OBC control: `0x1806E5F4` (charger operate/shutdown)

For the complete byte-level signal layout and scaling factors, refer to the
[BP16B Signals Agreement spreadsheet](https://docs.google.com/spreadsheets/d/1TbN9bK0u2_f_2LeJMQZu6MZCuvkwzeiOJ2B9JoZcbQw/edit?gid=226357247#gid=226357247)
or the local copy at `doc/BP16B Signals Agreement.xlsx`.
