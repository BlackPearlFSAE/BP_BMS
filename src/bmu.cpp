/* BMU - Battery Monitoring Unit
 * Handles cell voltage monitoring, temperature sensing, and fault detection
 * Communicates with BCU via CAN bus
 *
 * Architecture:
 * - LTC6811: Cell voltage reading and passive balancing
 * - NTC Thermistor: Temperature sensing (2 sensors per module)
 * - CAN TX: Periodic cell data (1000ms) and fault codes (1300ms)
 * - CAN RX: BCU config updates (runtime parameter changes)
 */

/************************* Includes ***************************/
#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <driver/twai.h>

#include "LTC6811.h"
#include "LTC681x.h"
#include "Linduino.h"
#include "CAN32_util.h"
#include "ams_common_conf.h"
#include "helper.h"
#include "ntstermistor.h"

/************************* Pin Definitions ***************************/
#define CAN_TX_PIN       20
#define CAN_RX_PIN       21
#define CS_PIN           7
#define TEMP_SENSOR1_PIN A0
#define TEMP_SENSOR2_PIN A1

/************************* Hardware Configuration ***************************/
#define TOTAL_IC    1
#define NUM_CELLS   10
#define BALANCE_THRESHOLD 0.01f  // 10mV above average triggers balancing

/************************* Global Variables ***************************/

// CAN messages
twai_message_t tx_message;
twai_message_t rx_message;
bool canbusready = false;

// LTC6811 data
cell_asic bms_ic[TOTAL_IC];
float cellvoltages[NUM_CELLS];

// BMU MSG 1 data
bool BMU_ReadyToCharge = false;              // Byte 0: Module ready to charge
uint16_t BalancingDischarge_Cells = 0x0000;  // Byte 1-2: 10-bit cell balancing status
uint8_t DV_raw = 0;                          // Byte 3: Delta V (factor 0.1V)

// BMU MSG 4 & 5 Fault flags (10-bit per fault type, each bit = 1 cell)
uint16_t OVERVOLTAGE_WARNING = 0x0000;
uint16_t OVERVOLTAGE_CRITICAL = 0x0000;
uint16_t LOWVOLTAGE_WARNING = 0x0000;
uint16_t LOWVOLTAGE_CRITICAL = 0x0000;
uint16_t OVERTEMP_WARNING = 0x0000;
uint16_t OVERTEMP_CRITICAL = 0x0000;
uint16_t OVERDIV_WARNING = 0x0000;
uint16_t OVERDIV_CRITICAL = 0x0000;

// BMU parameters (configurable from BCU at runtime)
int transimission_time = BMS_COMMUNICATE_TIME;
float VmaxCell = VMAX_CELL;
float VminCell = VMIN_CELL;
float TempMaxCell = TEMP_MAX_CELL;
float dVmax = DVMAX;

// Calculated fault thresholds (updated by updateThresholds())
float OV_WARNING_THRESHOLD;     // 0.95 * VmaxCell
float OV_CRITICAL_THRESHOLD;    // VmaxCell
float LV_WARNING_THRESHOLD;     // VminCell + 0.2
float LV_CRITICAL_THRESHOLD;    // VminCell
float TEMP_WARNING_THRESHOLD;   // 0.8 * TempMaxCell
float TEMP_CRITICAL_THRESHOLD;  // 0.9 * TempMaxCell
float DV_WARNING_THRESHOLD;     // dVmax
float DV_CRITICAL_THRESHOLD;    // dVmax * 1.5

// Temperature readings
float currentTemp1 = 25.0f;
float currentTemp2 = 25.0f;

// Balancing status array (index 0 = Cell 1)
static bool balancingStatus[NUM_CELLS];

// Timing
unsigned long prevMillis = 0;
unsigned long prevFaultMillis = 0;
unsigned long lastCANHealthCheck = 0;
unsigned long intervalMillis = 1000;        // Cell data transmission interval
unsigned long faultIntervalMillis = 1300;   // Fault code transmission interval
unsigned long canHealthCheckInterval = 5000; // CAN health check interval

/************************* Function Declarations ***************************/

// LTC6811 functions
void readAllCells();
float getTemp(int pin, int print);

// CAN functions
bool reinitCAN();
void checkCANHealth();
void processBCUConfigMsg(twai_message_t* msg);

// Message packing
void packBMU_MSG1_OperationStatus(twai_message_t* msg, uint32_t id, uint8_t temp1, uint8_t temp2);
void packBMU_MSG2_CellsLowSeries(twai_message_t* msg, uint32_t id);
void packBMU_MSG3_CellsHighSeries(twai_message_t* msg, uint32_t id);
void packBMU_MSG4_FaultCode1(twai_message_t* msg, uint32_t id);
void packBMU_MSG5_FaultCode2(twai_message_t* msg, uint32_t id);
uint8_t encode_temp(float temp_c);

// Fault detection and balancing
void updateThresholds();
float calculateAvgVoltage();
float calculateDV();
void updateFaultFlags();
bool isCellBalanced(uint8_t cellIndex);
bool* balanceCells(float vmaxCell, float vminCell, float tempMaxCell, float dvMax);

// Debug
void debugConfig();

/************************* Setup ***************************/
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { }
  delay(1000);

  // SPI setup for LTC6811
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(4, 5, 6, 7);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  Serial.println("SPI Initialized");

  // LTC6811 initialization
  LTC6811_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC, bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC, bms_ic);

  // Initialize fault thresholds from default config
  updateThresholds();

  // CAN bus initialization
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  canbusready = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, t_config);

  Serial.println("BMU initialized");
}

/************************* Main Loop ***************************/
void loop() {
  uint32_t SESSION_TIME = millis();
  int ModuleNum = 5;  // TODO: Set from EEPROM or DIP switch

  /*==================== Sensor Reading ====================*/

  // Read temperatures
  currentTemp1 = getTemp(TEMP_SENSOR1_PIN, 0);
  currentTemp2 = getTemp(TEMP_SENSOR2_PIN, 0);
  uint8_t temp1_encoded = encode_temp(currentTemp1);
  uint8_t temp2_encoded = encode_temp(currentTemp2);

  // Read cell voltages from LTC6811
  readAllCells();

  /*==================== CAN RX ====================*/

  // Process incoming CAN messages (BCU config updates)
  if (canbusready) {
    while (CAN32_receiveCAN(&rx_message) == ESP_OK) {
      processBCUConfigMsg(&rx_message);
    }
  }

  /*==================== Data Processing ====================*/

  // Calculate delta voltage and update DV_raw for CAN message
  float dv = calculateDV();
  DV_raw = (uint8_t)(dv / 0.1f);  // Factor 0.1V per spec
  if (DV_raw > 10) DV_raw = 10;   // Clamp to 0-1V range

  // Update fault flags based on current readings
  updateFaultFlags();

  // Cell balancing when charging (pass config parameters)
  if (BMU_ReadyToCharge) {
    bool* balancing = balanceCells(VmaxCell, VminCell, TempMaxCell, dVmax);
  }

  /*==================== CAN TX: Cell Data (1000ms) ====================*/

  if (SESSION_TIME - prevMillis >= intervalMillis) {
    // BMU MSG 1: Operation Status (Priority=2, Msg=1)
    uint32_t bmu_id_msg1 = createExtendedCANID(2, ModuleNum, 1);
    packBMU_MSG1_OperationStatus(&tx_message, bmu_id_msg1, temp1_encoded, temp2_encoded);
    if (CAN32_sendCAN(&tx_message, canbusready) != ESP_OK) {
      Serial.println("CAN TX failed: MSG1 Operation Status");
    }

    // BMU MSG 2: Cell 1-8 (Priority=2, Msg=2)
    uint32_t bmu_id_msg2 = createExtendedCANID(2, ModuleNum, 2);
    packBMU_MSG2_CellsLowSeries(&tx_message, bmu_id_msg2);
    if (CAN32_sendCAN(&tx_message, canbusready) != ESP_OK) {
      Serial.println("CAN TX failed: MSG2 Cells 1-8");
    }

    // BMU MSG 3: Cell 9-10 (Priority=2, Msg=3)
    uint32_t bmu_id_msg3 = createExtendedCANID(2, ModuleNum, 3);
    packBMU_MSG3_CellsHighSeries(&tx_message, bmu_id_msg3);
    if (CAN32_sendCAN(&tx_message, canbusready) != ESP_OK) {
      Serial.println("CAN TX failed: MSG3 Cells 9-10");
    }
    prevMillis = SESSION_TIME;
  }

  /*==================== CAN TX: Fault Codes (1300ms) ====================*/

  if (SESSION_TIME - prevFaultMillis >= faultIntervalMillis) {
    // BMU MSG 4: Fault Code 1 - OV/LV (Priority=1, Msg=1)
    uint32_t bmu_id_fault1 = createExtendedCANID(1, ModuleNum, 1);
    packBMU_MSG4_FaultCode1(&tx_message, bmu_id_fault1);
    if (CAN32_sendCAN(&tx_message, canbusready) != ESP_OK) {
      Serial.println("CAN TX failed: MSG4 Fault Code 1");
    }

    // BMU MSG 5: Fault Code 2 - Temp/DV (Priority=1, Msg=2)
    uint32_t bmu_id_fault2 = createExtendedCANID(1, ModuleNum, 2);
    packBMU_MSG5_FaultCode2(&tx_message, bmu_id_fault2);
    if (CAN32_sendCAN(&tx_message, canbusready) != ESP_OK) {
      Serial.println("CAN TX failed: MSG5 Fault Code 2");
    }
    prevFaultMillis = SESSION_TIME;
  }

  /*==================== CAN Health Check (5000ms) ====================*/

  if (SESSION_TIME - lastCANHealthCheck >= canHealthCheckInterval) {
    checkCANHealth();
    lastCANHealthCheck = SESSION_TIME;
  }
}

/************************* LTC6811 Functions ***************************/

void readAllCells() {
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);
  delay(10);
  int8_t error = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);

  for (int i = 0; i < NUM_CELLS; i++) {
    cellvoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001;
  }
}

float getTemp(int pin, int print) {
  float temp = resistance_to_celsius(read_ntc_resistance(pin));

  if (print == 1) {
    Serial.print("tempC pin ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.println(temp);
  }

  return temp;
}

/************************* CAN Functions ***************************/

bool reinitCAN() {
  Serial.println("--- CAN Bus Reinitialization ---");

  twai_stop();
  twai_driver_uninstall();
  delay(100);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  canbusready = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, t_config);

  if (canbusready) {
    Serial.println("CAN bus reinitialized successfully");
  } else {
    Serial.println("CAN bus reinitialization FAILED");
  }

  return canbusready;
}

void checkCANHealth() {
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);

  if (status_info.state == TWAI_STATE_BUS_OFF ||
      status_info.state == TWAI_STATE_STOPPED) {
    Serial.println("[CAN] Bus error detected - attempting recovery...");
    reinitCAN();
  }
  else if (status_info.tx_error_counter > 127 || status_info.rx_error_counter > 127) {
    Serial.printf("[CAN] High error count (TX:%d, RX:%d) - reinitializing...\n",
                  status_info.tx_error_counter, status_info.rx_error_counter);
    reinitCAN();
  }
}

void processBCUConfigMsg(twai_message_t* msg) {
  if (msg->identifier != BCU_ADD) return;

  bool BMUUpdateFlag = msg->data[6];
  if (!BMUUpdateFlag) return;

  transimission_time = msg->data[0];
  BMU_ReadyToCharge = msg->data[1];
  VmaxCell     = msg->data[2] * 0.1;
  VminCell     = msg->data[3] * 0.1;
  TempMaxCell  = msg->data[4];
  dVmax        = msg->data[5] * 0.1;

  updateThresholds();
}

/************************* CAN Message Packing ***************************/

uint8_t encode_temp(float temp_c) {
  if (temp_c < -40.0f) temp_c = -40.0f;
  if (temp_c > 87.5f) temp_c = 87.5f;

  float scaled = (temp_c + 40.0f) * 2.0f;
  return (uint8_t)(scaled + 0.5f);
}

// BMU MSG 1: Module Operation & Voltage Status
// Byte 0: BMU Enter Charging Mode (0=NO, 1=YES)
// Byte 1-2: Cells in Balancing (10-bit, MSB first)
// Byte 3: Averaged Cell Voltage Difference DV (factor 0.1V)
// Byte 4: Temp Sensor 1 (offset -40, factor 0.5C)
// Byte 5: Temp Sensor 2 (offset -40, factor 0.5C)
// Byte 6-7: Reserved
void packBMU_MSG1_OperationStatus(twai_message_t* msg, uint32_t id, uint8_t temp1, uint8_t temp2) {
  msg->identifier = id;
  msg->extd = 1;
  msg->rtr = 0;
  msg->data_length_code = 8;

  msg->data[0] = BMU_ReadyToCharge ? 1 : 0;
  msg->data[1] = (BalancingDischarge_Cells >> 8) & 0xFF;
  msg->data[2] = BalancingDischarge_Cells & 0xFF;
  msg->data[3] = DV_raw;
  msg->data[4] = temp1;
  msg->data[5] = temp2;
  msg->data[6] = 0x00;
  msg->data[7] = 0x00;
}

// BMU MSG 2: Cell Monitoring LOW SERIES
// Byte 0-7: Cell 1-8 voltages (factor 0.02V)
void packBMU_MSG2_CellsLowSeries(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;
  msg->rtr = 0;
  msg->data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    msg->data[i] = (uint8_t)(cellvoltages[i] / 0.02);
  }
}

// BMU MSG 3: Cell Monitoring HIGH SERIES
// Byte 0-1: Cell 9-10 voltages (factor 0.02V)
// Byte 2-7: Reserved
void packBMU_MSG3_CellsHighSeries(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;
  msg->rtr = 0;
  msg->data_length_code = 8;

  msg->data[0] = (uint8_t)(cellvoltages[8] / 0.02);
  msg->data[1] = (uint8_t)(cellvoltages[9] / 0.02);
  msg->data[2] = 0x00;
  msg->data[3] = 0x00;
  msg->data[4] = 0x00;
  msg->data[5] = 0x00;
  msg->data[6] = 0x00;
  msg->data[7] = 0x00;
}

// BMU MSG 4: Module Fault Code 1
// Byte 0-1: Over Voltage Warning (10-bit, MSB first)
// Byte 2-3: Over Voltage Critical (10-bit, MSB first)
// Byte 4-5: Low Voltage Warning (10-bit, MSB first)
// Byte 6-7: Low Voltage Critical (10-bit, MSB first)
void packBMU_MSG4_FaultCode1(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;
  msg->rtr = 0;
  msg->data_length_code = 8;

  msg->data[0] = (OVERVOLTAGE_WARNING >> 8) & 0xFF;
  msg->data[1] = OVERVOLTAGE_WARNING & 0xFF;
  msg->data[2] = (OVERVOLTAGE_CRITICAL >> 8) & 0xFF;
  msg->data[3] = OVERVOLTAGE_CRITICAL & 0xFF;
  msg->data[4] = (LOWVOLTAGE_WARNING >> 8) & 0xFF;
  msg->data[5] = LOWVOLTAGE_WARNING & 0xFF;
  msg->data[6] = (LOWVOLTAGE_CRITICAL >> 8) & 0xFF;
  msg->data[7] = LOWVOLTAGE_CRITICAL & 0xFF;
}

// BMU MSG 5: Module Fault Code 2
// Byte 0-1: Over Temp Warning (10-bit, MSB first)
// Byte 2-3: Over Temp Critical (10-bit, MSB first)
// Byte 4-5: Over Div Voltage Warning (10-bit, MSB first)
// Byte 6-7: Over Div Voltage Critical (10-bit, MSB first)
void packBMU_MSG5_FaultCode2(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;
  msg->rtr = 0;
  msg->data_length_code = 8;

  msg->data[0] = (OVERTEMP_WARNING >> 8) & 0xFF;
  msg->data[1] = OVERTEMP_WARNING & 0xFF;
  msg->data[2] = (OVERTEMP_CRITICAL >> 8) & 0xFF;
  msg->data[3] = OVERTEMP_CRITICAL & 0xFF;
  msg->data[4] = (OVERDIV_WARNING >> 8) & 0xFF;
  msg->data[5] = OVERDIV_WARNING & 0xFF;
  msg->data[6] = (OVERDIV_CRITICAL >> 8) & 0xFF;
  msg->data[7] = OVERDIV_CRITICAL & 0xFF;
}

/************************* Fault Detection & Thresholds ***************************/

void updateThresholds() {
  OV_WARNING_THRESHOLD = 0.95f * VmaxCell;
  OV_CRITICAL_THRESHOLD = VmaxCell;
  LV_WARNING_THRESHOLD = VminCell + 0.2f;
  LV_CRITICAL_THRESHOLD = VminCell;
  TEMP_WARNING_THRESHOLD = 0.8f * TempMaxCell;
  TEMP_CRITICAL_THRESHOLD = 0.9f * TempMaxCell;
  DV_WARNING_THRESHOLD = dVmax;
  DV_CRITICAL_THRESHOLD = dVmax * 1.5f;

  Serial.println("Thresholds updated:");
  Serial.printf("  OV: warn=%.2fV, crit=%.2fV\n", OV_WARNING_THRESHOLD, OV_CRITICAL_THRESHOLD);
  Serial.printf("  LV: warn=%.2fV, crit=%.2fV\n", LV_WARNING_THRESHOLD, LV_CRITICAL_THRESHOLD);
  Serial.printf("  Temp: warn=%.1fC, crit=%.1fC\n", TEMP_WARNING_THRESHOLD, TEMP_CRITICAL_THRESHOLD);
  Serial.printf("  DV: warn=%.2fV, crit=%.2fV\n", DV_WARNING_THRESHOLD, DV_CRITICAL_THRESHOLD);
}

float calculateAvgVoltage() {
  float sum = 0;
  for (int i = 0; i < NUM_CELLS; i++) {
    sum += cellvoltages[i];
  }
  return sum / NUM_CELLS;
}

float calculateDV() {
  float avgV = calculateAvgVoltage();
  float maxDV = 0;

  for (int i = 0; i < NUM_CELLS; i++) {
    float deviation = fabs(cellvoltages[i] - avgV);
    if (deviation > maxDV) {
      maxDV = deviation;
    }
  }
  return maxDV;
}

void updateFaultFlags() {
  // Reset all fault flags
  OVERVOLTAGE_WARNING = 0;
  OVERVOLTAGE_CRITICAL = 0;
  LOWVOLTAGE_WARNING = 0;
  LOWVOLTAGE_CRITICAL = 0;
  OVERTEMP_WARNING = 0;
  OVERTEMP_CRITICAL = 0;
  OVERDIV_WARNING = 0;
  OVERDIV_CRITICAL = 0;

  float avgV = calculateAvgVoltage();

  // Check each cell for voltage faults (10-bit representation, MSB = Cell 1)
  for (int i = 0; i < NUM_CELLS; i++) {
    uint16_t cellBit = (1 << (9 - i));

    // Over voltage check
    if (cellvoltages[i] >= OV_CRITICAL_THRESHOLD) {
      OVERVOLTAGE_CRITICAL |= cellBit;
    } else if (cellvoltages[i] > OV_WARNING_THRESHOLD) {
      OVERVOLTAGE_WARNING |= cellBit;
    }

    // Low voltage check
    if (cellvoltages[i] <= LV_CRITICAL_THRESHOLD) {
      LOWVOLTAGE_CRITICAL |= cellBit;
    } else if (cellvoltages[i] < LV_WARNING_THRESHOLD) {
      LOWVOLTAGE_WARNING |= cellBit;
    }

    // Delta voltage check (per-cell deviation from average)
    float cellDV = fabs(cellvoltages[i] - avgV);
    if (cellDV >= DV_CRITICAL_THRESHOLD) {
      OVERDIV_CRITICAL |= cellBit;
    } else if (cellDV >= DV_WARNING_THRESHOLD) {
      OVERDIV_WARNING |= cellBit;
    }
  }

  // Temperature check (applies to all cells since only 2 sensors for module)
  float maxTemp = (currentTemp1 > currentTemp2) ? currentTemp1 : currentTemp2;

  if (maxTemp >= TEMP_CRITICAL_THRESHOLD) {
    OVERTEMP_CRITICAL = 0x3FF;
  } else if (maxTemp >= TEMP_WARNING_THRESHOLD) {
    OVERTEMP_WARNING = 0x3FF;
  }
}

/************************* Cell Balancing ***************************/

bool isCellBalanced(uint8_t cellIndex) {
  if (cellIndex >= NUM_CELLS) return false;
  return balancingStatus[cellIndex];
}

// Passive cell balancing using LTC6811 discharge resistors
// Returns boolean array where true = cell is being balanced (index 0 = Cell 1)
bool* balanceCells(float vmaxCell, float vminCell, float tempMaxCell, float dvMax) {
  // Initialize all cells as not balancing
  for (int i = 0; i < NUM_CELLS; i++) {
    balancingStatus[i] = false;
  }

  if (!BMU_ReadyToCharge) {
    // Clear LTC6811 discharge when not charging
    bms_ic[0].config.tx_data[4] = 0;
    bms_ic[0].config.tx_data[5] &= 0xFC;
    LTC6811_wrcfg(TOTAL_IC, bms_ic);

    BalancingDischarge_Cells = toUint16FromBitarrayMSB(balancingStatus);
    return balancingStatus;
  }

  float avgV = calculateAvgVoltage();

  // Identify cells for balancing
  for (int i = 0; i < NUM_CELLS; i++) {
    float cellDV = fabs(cellvoltages[i] - avgV);

    // Only balance if:
    // 1. Cell voltage is above average + threshold
    // 2. Cell is not already at or below minimum voltage
    // 3. Cell voltage deviation from avg is within safe range
    if (cellvoltages[i] > (avgV + BALANCE_THRESHOLD) &&
        cellvoltages[i] > vminCell &&
        cellDV < (dvMax * 1.5f)) {
      balancingStatus[i] = true;
    }
  }

  // Convert boolean array to uint16_t for CAN message (MSB-first format)
  BalancingDischarge_Cells = toUint16FromBitarrayMSB(balancingStatus);

  // Configure LTC6811 discharge (DCC bits are LSB-first: bit 0 = Cell 1)
  uint16_t dischargeBits = toUint16FromBitarrayLSB(balancingStatus);
  bms_ic[0].config.tx_data[4] = dischargeBits & 0xFF;
  bms_ic[0].config.tx_data[5] = (bms_ic[0].config.tx_data[5] & 0xFC) |
                                 ((dischargeBits >> 8) & 0x03);
  LTC6811_wrcfg(TOTAL_IC, bms_ic);

  if (dischargeBits != 0) {
    Serial.printf("Balancing: 0x%03X (avg=%.3fV, CAN=0x%03X)\n",
                  dischargeBits, avgV, BalancingDischarge_Cells);
  }

  return balancingStatus;
}

/************************* Debug Functions ***************************/

void debugConfig() {
  Serial.println("=== BMU Runtime Config ===");
  Serial.printf("SyncTime: %dms\n", transimission_time);
  Serial.printf("ChargerPlugged: %s\n", BMU_ReadyToCharge ? "YES" : "NO");
  Serial.printf("VmaxCell: %.1fV\n", VmaxCell);
  Serial.printf("VminCell: %.1fV\n", VminCell);
  Serial.printf("TempMax: %.0fC\n", TempMaxCell);
  Serial.printf("DVmax: %.1fV\n", dVmax);
}
