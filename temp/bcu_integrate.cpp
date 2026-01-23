/* BCU - Battery Control Unit
 * Handles CAN communication with BMU modules and OBC charger
 * Determines AMS_OK status based on fault conditions
 *
 * Architecture:
 * - CAN RX: Lightweight message processing (store data only)
 * - Real-time: BMU connection monitoring every loop
 * - Periodic (500ms): AMS data aggregation
 * - AMS_OK: Fault evaluation and output control
 */

/************************* Includes ***************************/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <driver/gpio.h>
#include <driver/twai.h>
#include <Arduino.h>
#include <SPI.h>
#include <EEPROM.h>

#include "CAN32_util.h"
#include "LTC6811.h"
#include "LTC681x.h"
#include "Linduino.h"
#include "ams_common_conf.h"
#include "helper.h"
#include "ntstermistor.h"

/************************* Pin Definitions ***************************/
#define CAN_TX_PIN       20
#define CAN_RX_PIN       21
// #define OBCIN         9  // Charger plug detect input
#define AMS_OUT       3  // AMS OK output signal

// BCU local LTC6811 pins
#define CS_PIN           7
#define TEMP_SENSOR1_PIN 0
#define TEMP_SENSOR2_PIN 1

/************************* LTC6811 Configuration ***************************/
#define TOTAL_IC    1
#define NUM_CELLS   CELL_NUM

/************************* Global Variables ***************************/

// Total Module 1 BCU master the rest are BMU slave
#define BMU_NUM (MODULE_NUM - 1)
#define BCU_LOCAL_INDEX (MODULE_NUM - 1)  // BCU's own cells stored at last index

// CAN messages
twai_message_t sendMsg;
twai_message_t receivedMsg;
twai_message_t chargerMsg;
bool canbusready = false;

// Timing
unsigned long Sustained_Communicate_Time = 0;
unsigned long lastModuleResponse[BMU_NUM];
uint32_t shutdown_timer = 0;
uint32_t debug_timer = 0;
uint32_t aggregation_timer = 0;

// Hardware timers
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// Data structures (MODULE_NUM to include BCU's local cells at last index)
BMUdata BMU_Package[MODULE_NUM];
AMSdata AMS_Package;
OBCdata OBC_Package;

// Aliases for cleaner access
bool &AMS_OK = AMS_Package.AMS_OK;
float &ACCUM_MAXVOLTAGE = AMS_Package.ACCUM_MAXVOLTAGE;
float &ACCUM_MINVOLTAGE = AMS_Package.ACCUM_MINVOLTAGE;

// Flags
volatile bool CAN_SEND_FLG1 = false;
volatile bool CAN_SEND_FLG2 = false;
bool CHARGER_PLUGGED = false;
bool CAN_TIMEOUT_FLG = false;
bool ACCUM_ReadytoCharge = false;
bool ACCUM_OverDivWarn = false;
bool ACCUM_OverDivCritical = false;
bool OVER_TEMP_WARN = false;
bool OVER_TEMP_CRIT = false;
bool LOW_VOLT_CRIT = false;
bool LOW_VOLT_WARN = false;
bool OVER_VOLT_CRIT = false;
bool ACCUM_FULL = false;
bool ACCUM_LOW = false;

// BMU parameters (configurable)
int transimission_time = BMS_COMMUNICATE_TIME;
float VmaxCell = VMAX_CELL;
float VminCell = VMIN_CELL;
int TempMaxCell = TEMP_MAX_CELL;
float dVmax = DVMAX;
bool BMUUpdateFlag = false;

// BCU local LTC6811 data
cell_asic bms_ic[TOTAL_IC];
float cellvoltages[NUM_CELLS];
float currentTemp1 = 25.0f;
float currentTemp2 = 25.0f;

// Fault thresholds (calculated from config)
float OV_WARNING_THRESHOLD;
float OV_CRITICAL_THRESHOLD;
float LV_WARNING_THRESHOLD;
float LV_CRITICAL_THRESHOLD;
float TEMP_WARNING_THRESHOLD;
float TEMP_CRITICAL_THRESHOLD;
float DV_WARNING_THRESHOLD;
float DV_CRITICAL_THRESHOLD;

/************************* Function Declarations ***************************/
void packBCU_toAMSmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool is_charger_plugged);
void packBCU_toOBCmsg(twai_message_t *BCUsent, bool AMS_OK, bool ReadytoCharge, bool OverDivCritical_Yes, bool Voltage_is_Full);
void processReceived_BMUmsg(twai_message_t *receivedframe, BMUdata *BMU_Package);
void processReceived_OBCmsg(twai_message_t *receivedframe);
void packing_AMSstruct(int moduleIndex);
bool isModuleActive(int moduleIndex);
bool checkModuleDisconnect(BMUdata *BMU_Package);
void resetAllStruct();
void updateLocalBMU(BMUdata *localModule);

// BCU local cell monitoring helpers
void initLocalLTC6811();
void updateThresholds();
void readLocalCells(BMUdata *localModule);
float getLocalTemp(int pin);
void updateLocalFaultFlags(BMUdata *localModule);
float calculateLocalAvgVoltage();
float calculateLocalDV();
uint8_t encode_temp(float temp_c);

// Debug functions (can be disabled in production)
void debugAMSstate();
void debugBMUMod(int moduleNum);
void debugOBCmsg();

/************************* Timer ISRs ***************************/
void IRAM_ATTR onTimer_dischargeMode() {
  CAN_SEND_FLG1 = true;
}

void IRAM_ATTR onTimer_chargeMode() {
  CAN_SEND_FLG2 = true;
}

/************************* Setup ***************************/
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Disable brownout detector
  Serial.begin(115200);

  // GPIO setup
  // pinMode(OBCIN, INPUT_PULLDOWN);
  pinMode(AMS_OUT, OUTPUT);
  digitalWrite(AMS_OUT, LOW);  // Start with AMS_OK = false

  // Initialize module response timestamps (0 = never received)
  for (int i = 0; i < BMU_NUM; i++) {
    lastModuleResponse[i] = 0;
  }

  // Initialize BCU local LTC6811
  initLocalLTC6811();
  updateThresholds();

  // CAN bus init
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  canbusready = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, STANDARD_BIT_RATE, filter_config);
  sendMsg.extd = receivedMsg.extd = chargerMsg.extd = true;

  // Timer 1: BCU broadcast to BMUs (1000ms)
  My_timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer1, &onTimer_dischargeMode, true);
  timerAlarmWrite(My_timer1, BMS_COMMUNICATE_TIME * 1000, true);
  timerAlarmEnable(My_timer1);

  // Timer 2: BCU broadcast to OBC (500ms, only when charging)
  My_timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer2, &onTimer_chargeMode, true);
  timerAlarmWrite(My_timer2, OBC_COMMUNICATE_TIME * 1000, true);
  timerAlarmEnable(My_timer2);

  // Initialize timing references
  Sustained_Communicate_Time = millis();
  aggregation_timer = millis();
  debug_timer = millis();

  Serial.println("BCU initialized");
}

/************************* Main Loop ***************************/
void loop() {
  unsigned long SESSION_TIME = millis();
  // CHARGER_PLUGGED = digitalRead(OBCIN);
  CHARGER_PLUGGED = false;

  /*==================== CAN TX ====================*/

  // Broadcast to BMUs (every 1000ms)
  if (CAN_SEND_FLG1) {
    CAN_SEND_FLG1 = false;
    packBCU_toAMSmsg(&sendMsg, BMS_COMMUNICATE_TIME, CHARGER_PLUGGED);
    CAN32_sendCAN(&sendMsg, canbusready);
  }

  // Broadcast to OBC (every 500ms, only when charging)
  if (CAN_SEND_FLG2 && CHARGER_PLUGGED) {
    CAN_SEND_FLG2 = false;
    packBCU_toOBCmsg(&chargerMsg, AMS_OK, ACCUM_ReadytoCharge, ACCUM_OverDivCritical, ACCUM_FULL);
    CAN32_sendCAN(&chargerMsg, canbusready);
  }

  /*==================== CAN RX ====================*/

  // Process incoming messages (lightweight - store data only)
  if (CAN32_receiveCAN(&receivedMsg, canbusready) == ESP_OK) {
    processReceived_BMUmsg(&receivedMsg, BMU_Package);
    if (CHARGER_PLUGGED) {
      processReceived_OBCmsg(&receivedMsg);
    }
    CAN_TIMEOUT_FLG = false;
    Sustained_Communicate_Time = millis();
  }
  // Complete communication loss check
  else if (SESSION_TIME - Sustained_Communicate_Time >= DISCONNENCTION_TIMEOUT) {
    digitalWrite(AMS_OUT, LOW);
    if (SESSION_TIME - shutdown_timer >= 500) {
      Serial.println("NO_BYTE_RECV");
      shutdown_timer = millis();
    }
    if (!CAN_TIMEOUT_FLG) resetAllStruct();
    CAN_TIMEOUT_FLG = true;
    return;
  }

  /*==================== BCU Local Cell Update ====================*/

  updateLocalBMU(&BMU_Package[BCU_LOCAL_INDEX]);

  /*==================== Real-time Connection Check ====================*/

  // Loop 1: Set BMUconnected (CAN-based BMUs only, BCU local is always connected)
  for (int i = 0; i < BMU_NUM; i++) {
    BMU_Package[i].BMUconnected = isModuleActive(i);
  }
  if(checkModuleDisconnect(BMU_Package)){
    digitalWrite(AMS_OUT, LOW);
    if (SESSION_TIME - debug_timer >= 500) {
      Serial.println("BMU_DISCONNECTED:");
      for (int i = 0; i < BMU_NUM; i++) {
        if(!BMU_Package[i].BMUconnected) Serial.printf("  Module %d\n", i + 1); 
      }
      debug_timer = millis();
    }
    return;
  }
  // // Check each BMU connection status
  // bool allConnected = true;
  // for (int i = 0; i < BMU_NUM; i++) {
  //   BMU_Package[i].BMUconnected = isModuleActive(i);
  //   if (!BMU_Package[i].BMUconnected) allConnected = false;
  // }

  // // Handle BMU disconnection
  // if (!allConnected) {
  //   digitalWrite(AMS_OUT, LOW);
  //   if (SESSION_TIME - debug_timer >= 500) {
  //     Serial.println("BMU_DISCONNECTED:");
  //     for (int i = 0; i < BMU_NUM; i++) {
  //       if (!BMU_Package[i].BMUconnected) Serial.printf("  Module %d\n", i + 1); 
  //     }
  //     debug_timer = millis();
  //   }
  //   return;
  // }

  /*==================== Periodic Aggregation (500ms) ====================*/

  if (SESSION_TIME - aggregation_timer >= 500) {
    AMS_Package = AMSdata(); // Reset AMS package for fresh aggregation

    for (int j = 0; j < MODULE_NUM; j++) {
      if (BMU_Package[j].BMUconnected) {
        
        // Calculate module voltage (sum of cells)
        BMU_Package[j].V_MODULE = 0;
        for (int k = 0; k < CELL_NUM; k++)
          BMU_Package[j].V_MODULE += BMU_Package[j].V_CELL[k];
        
          // Pack into AMS aggregate
        packing_AMSstruct(j);
      }
    }
    aggregation_timer = SESSION_TIME;
  }

  /*==================== AMS_OK Determination ====================*/

  // Voltage fault flags
  ACCUM_FULL = (AMS_Package.ACCUM_VOLTAGE >= 0.95f * ACCUM_MAXVOLTAGE);
  ACCUM_LOW = (AMS_Package.ACCUM_VOLTAGE >= 1.12f * ACCUM_MINVOLTAGE);
  OVER_VOLT_CRIT = (AMS_Package.OVERVOLT_CRITICAL > 0);
  LOW_VOLT_CRIT = (AMS_Package.LOWVOLT_CRITICAL > 0);
  LOW_VOLT_WARN = (AMS_Package.LOWVOLT_WARNING > 0);
  LOW_VOLT_WARN = (AMS_Package.ACCUM_VOLTAGE <= 1.10f * ACCUM_MINVOLTAGE);

  // Temperature fault flags
  OVER_TEMP_CRIT = (AMS_Package.OVERTEMP_CRITICAL > 0);
  OVER_TEMP_WARN = (AMS_Package.OVERTEMP_WARNING > 0);

  // Differential voltage fault flags
  ACCUM_OverDivCritical = (AMS_Package.OVERDIV_CRITICAL > 0);
  ACCUM_OverDivWarn = (AMS_Package.OVERDIV_WARNING > 0);

  // AMS_OK logic: any critical fault triggers shutdown
  bool ACCUMULATOR_Fault = OVER_VOLT_CRIT || LOW_VOLT_CRIT || OVER_TEMP_CRIT || ACCUM_OverDivCritical;
  AMS_OK = !ACCUMULATOR_Fault;

  // Charging mode fault handling
  if (CHARGER_PLUGGED) {
    uint16_t OBCFault = OBC_Package.OBCstatusbit;
    ACCUM_ReadytoCharge = (AMS_Package.ACCUM_CHG_READY > 0);
    AMS_OK = !(ACCUMULATOR_Fault || OBCFault || !ACCUM_ReadytoCharge);
  }

  /*==================== Debug Output (500ms) ====================*/

  if (SESSION_TIME - debug_timer >= 500) {
    // for (int j = 0; j < BMU_NUM; j++) debugBMUMod(j);
    debugAMSstate();
    debug_timer = millis();
  }

  /*==================== Output Control ====================*/

  digitalWrite(AMS_OUT, AMS_OK ? HIGH : LOW);
}

/************************* CAN Message Packing ***************************/

void packBCU_toAMSmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool is_charger_plugged) {
  uint8_t* transmission_time = splitHLbyte(bcu_transimission_time);
  BCUsent->identifier = BCU_ADD;
  BCUsent->data_length_code = 8;
  BCUsent->data[0] = transmission_time[0];
  BCUsent->data[1] = transmission_time[1];
  BCUsent->data[2] = is_charger_plugged ? 1 : 0;
  BCUsent->data[3] = (uint8_t)(VmaxCell / 0.1f);
  BCUsent->data[4] = (uint8_t)(VminCell / 0.1f);
  BCUsent->data[5] = (uint8_t)TempMaxCell;
  BCUsent->data[6] = (uint8_t)(dVmax / 0.1f);
  BCUsent->data[7] = BMUUpdateFlag;
}

void packBCU_toOBCmsg(twai_message_t *BCUsent, bool AMS_OK, bool ReadytoCharge, bool OverDivCritical_Yes, bool Voltage_is_Full) {
  BCUsent->identifier = OBC_ADD;
  BCUsent->data_length_code = 8;
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;

  // Charge if OK, ready, no critical overdiv, and not full
  if (AMS_OK && ReadytoCharge && !OverDivCritical_Yes && !Voltage_is_Full) {
    BCUsent->data[0] = 0x18;  // Voltage high byte (240.0V)
    BCUsent->data[1] = 0x00;  // Voltage low byte
    BCUsent->data[2] = 0x00;  // Current high byte
    BCUsent->data[3] = 0x64;  // Current low byte (10.0A)
    BCUsent->data[4] = 0x00;  // Control: charger operate
  } else {
    BCUsent->data[0] = 0x00;
    BCUsent->data[1] = 0x00;
    BCUsent->data[2] = 0x00;
    BCUsent->data[3] = 0x00;
    BCUsent->data[4] = 0x01;  // Control: charger shutdown
  }
}

/************************* CAN Message Processing ***************************/

void processReceived_BMUmsg(twai_message_t *receivedframe, BMUdata *BMU_Package) {
  extCANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, receivedframe->identifier);

  int i = decodedCANID.SRC - 1;
  if (i < 0 || i >= BMU_NUM) return;

  // Update timestamp and ID
  lastModuleResponse[i] = millis();
  BMU_Package[i].BMU_ID = receivedframe->identifier;

  // Priority 0x02: BMU module & cell data
  if (decodedCANID.PRIORITY == 0x02) {
    switch (decodedCANID.MSG_NUM) {
      case 1:  // Operation status
        BMU_Package[i].BMUreadytoCharge = receivedframe->data[0];
        BMU_Package[i].BalancingDischarge_Cells = mergeHLbyte(receivedframe->data[1], receivedframe->data[2]);
        BMU_Package[i].DV = receivedframe->data[3];
        BMU_Package[i].TEMP_SENSE[0] = receivedframe->data[4];
        BMU_Package[i].TEMP_SENSE[1] = receivedframe->data[5];
        break;

      case 2:  // Cell voltages C1-C8
        for (int j = 0; j < 8; j++) {
          BMU_Package[i].V_CELL[j] = receivedframe->data[j];
        }
        break;

      case 3:  // Cell voltages C9-C10
        for (int j = 8; j < CELL_NUM; j++) {
          BMU_Package[i].V_CELL[j] = receivedframe->data[j - 8];
        }
        break;
    }
  }
  // Priority 0x01: Fault codes
  else if (decodedCANID.PRIORITY == 0x01) {
    switch (decodedCANID.MSG_NUM) {
      case 1:
        BMU_Package[i].OVERVOLTAGE_WARNING = mergeHLbyte(receivedframe->data[0], receivedframe->data[1]);
        BMU_Package[i].OVERVOLTAGE_CRITICAL = mergeHLbyte(receivedframe->data[2], receivedframe->data[3]);
        BMU_Package[i].LOWVOLTAGE_WARNING = mergeHLbyte(receivedframe->data[4], receivedframe->data[5]);
        BMU_Package[i].LOWVOLTAGE_CRITICAL = mergeHLbyte(receivedframe->data[6], receivedframe->data[7]);
        break;

      case 2:
        BMU_Package[i].OVERTEMP_WARNING = mergeHLbyte(receivedframe->data[0], receivedframe->data[1]);
        BMU_Package[i].OVERTEMP_CRITICAL = mergeHLbyte(receivedframe->data[2], receivedframe->data[3]);
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = mergeHLbyte(receivedframe->data[4], receivedframe->data[5]);
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = mergeHLbyte(receivedframe->data[6], receivedframe->data[7]);
        break;
    }
  }
}

void processReceived_OBCmsg(twai_message_t *receivedframe) {
  if (receivedframe->identifier != 0x18FF50E5) return;

  OBC_Package.OBCstatusbit = receivedframe->data[4];
  OBC_Package.OBCVolt = mergeHLbyte(receivedframe->data[0], receivedframe->data[1]);
  OBC_Package.OBCAmp = mergeHLbyte(receivedframe->data[2], receivedframe->data[3]);
}

/************************* AMS Helper Functions ***************************/

void packing_AMSstruct(int moduleIndex) {
  int k = moduleIndex;

  // Accumulate voltage (0.02V per bit × sum of cells)
  AMS_Package.ACCUM_VOLTAGE += (float)(BMU_Package[k].V_MODULE) * 0.02f;

  // OR together fault flags
  AMS_Package.OVERTEMP_WARNING |= BMU_Package[k].OVERTEMP_WARNING;
  AMS_Package.OVERTEMP_CRITICAL |= BMU_Package[k].OVERTEMP_CRITICAL;
  AMS_Package.OVERDIV_WARNING |= BMU_Package[k].OVERDIV_VOLTAGE_WARNING;
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[k].OVERDIV_VOLTAGE_CRITICAL;

  // AND together charge ready (all must be ready)
  AMS_Package.ACCUM_CHG_READY &= BMU_Package[k].BMUreadytoCharge;
}

void resetAllStruct() {
  for (int i = 0; i < BMU_NUM; i++) {
    BMU_Package[i] = BMUdata();
    lastModuleResponse[i] = 0;
  }
  AMS_Package = AMSdata();
  OBC_Package = OBCdata();
}

bool isModuleActive(int moduleIndex) {
  unsigned long lastResponse = lastModuleResponse[moduleIndex];
  if (lastResponse == 0) return false;  // Never received a response
  return (millis() - lastResponse) <= DISCONNENCTION_TIMEOUT;
}

bool checkModuleDisconnect(BMUdata *BMU_Package) {
  for (short i = 0; i < BMU_NUM; i++) {
    if (!BMU_Package[i].BMUconnected) return true;
  }
  return false;
}

/************************* BCU Local Cell Monitoring ***************************/

void initLocalLTC6811() {
  // SPI setup for LTC6811
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(4, 5, 6, 7);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);

  // LTC6811 initialization
  LTC6811_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC, bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC, bms_ic);

  Serial.println("BCU local LTC6811 initialized");
}

void updateThresholds() {
  OV_WARNING_THRESHOLD = 0.95f * VmaxCell;
  OV_CRITICAL_THRESHOLD = VmaxCell;
  LV_WARNING_THRESHOLD = VminCell + 0.2f;
  LV_CRITICAL_THRESHOLD = VminCell;
  TEMP_WARNING_THRESHOLD = 0.8f * TempMaxCell;
  TEMP_CRITICAL_THRESHOLD = 0.9f * TempMaxCell;
  DV_WARNING_THRESHOLD = dVmax;
  DV_CRITICAL_THRESHOLD = dVmax * 1.5f;
}

void updateLocalBMU(BMUdata *localModule) {
  // BCU's local module is always connected (no CAN needed)
  localModule->BMUconnected = true;

  // Read temperatures
  currentTemp1 = getLocalTemp(TEMP_SENSOR1_PIN);
  currentTemp2 = getLocalTemp(TEMP_SENSOR2_PIN);
  localModule->TEMP_SENSE[0] = encode_temp(currentTemp1);
  localModule->TEMP_SENSE[1] = encode_temp(currentTemp2);

  // Read cell voltages from LTC6811
  readLocalCells(localModule);

  // Calculate delta voltage
  float dv = calculateLocalDV();
  localModule->DV = (uint8_t)(dv / 0.1f);
  if (localModule->DV > 10) localModule->DV = 10;  // Clamp to 0-1V range

  // Update fault flags
  updateLocalFaultFlags(localModule);
}

void readLocalCells(BMUdata *localModule) {
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);
  delay(10);
  LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);

  for (int i = 0; i < NUM_CELLS; i++) {
    cellvoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001f;
    localModule->V_CELL[i] = (uint8_t)(cellvoltages[i] / 0.02f);  // CAN format (0.02V factor)
  }
}

float getLocalTemp(int pin) {
  return resistance_to_celsius(read_ntc_resistance(pin));
}

uint8_t encode_temp(float temp_c) {
  if (temp_c < -40.0f) temp_c = -40.0f;
  if (temp_c > 87.5f) temp_c = 87.5f;
  float scaled = (temp_c + 40.0f) * 2.0f;
  return (uint8_t)(scaled + 0.5f);
}

float calculateLocalAvgVoltage() {
  float sum = 0;
  for (int i = 0; i < NUM_CELLS; i++) {
    sum += cellvoltages[i];
  }
  return sum / NUM_CELLS;
}

float calculateLocalDV() {
  float avgV = calculateLocalAvgVoltage();
  float maxDV = 0;
  for (int i = 0; i < NUM_CELLS; i++) {
    float deviation = fabs(cellvoltages[i] - avgV);
    if (deviation > maxDV) {
      maxDV = deviation;
    }
  }
  return maxDV;
}

void updateLocalFaultFlags(BMUdata *localModule) {
  // Reset all fault flags
  localModule->OVERVOLTAGE_WARNING = 0;
  localModule->OVERVOLTAGE_CRITICAL = 0;
  localModule->LOWVOLTAGE_WARNING = 0;
  localModule->LOWVOLTAGE_CRITICAL = 0;
  localModule->OVERTEMP_WARNING = 0;
  localModule->OVERTEMP_CRITICAL = 0;
  localModule->OVERDIV_VOLTAGE_WARNING = 0;
  localModule->OVERDIV_VOLTAGE_CRITICAL = 0;

  float avgV = calculateLocalAvgVoltage();

  // Check each cell for voltage faults (10-bit representation, MSB = Cell 1)
  for (int i = 0; i < NUM_CELLS; i++) {
    uint16_t cellBit = (1 << (9 - i));

    // Over voltage check
    if (cellvoltages[i] >= OV_CRITICAL_THRESHOLD) {
      localModule->OVERVOLTAGE_CRITICAL |= cellBit;
    } else if (cellvoltages[i] > OV_WARNING_THRESHOLD) {
      localModule->OVERVOLTAGE_WARNING |= cellBit;
    }

    // Low voltage check
    if (cellvoltages[i] <= LV_CRITICAL_THRESHOLD) {
      localModule->LOWVOLTAGE_CRITICAL |= cellBit;
    } else if (cellvoltages[i] < LV_WARNING_THRESHOLD) {
      localModule->LOWVOLTAGE_WARNING |= cellBit;
    }

    // Delta voltage check (per-cell deviation from average)
    float cellDV = fabs(cellvoltages[i] - avgV);
    if (cellDV >= DV_CRITICAL_THRESHOLD) {
      localModule->OVERDIV_VOLTAGE_CRITICAL |= cellBit;
    } else if (cellDV >= DV_WARNING_THRESHOLD) {
      localModule->OVERDIV_VOLTAGE_WARNING |= cellBit;
    }
  }

  // Temperature check (applies to all cells since only 2 sensors for module)
  float maxTemp = (currentTemp1 > currentTemp2) ? currentTemp1 : currentTemp2;

  if (maxTemp >= TEMP_CRITICAL_THRESHOLD) {
    localModule->OVERTEMP_CRITICAL = 0x3FF;
  } else if (maxTemp >= TEMP_WARNING_THRESHOLD) {
    localModule->OVERTEMP_WARNING = 0x3FF;
  }
}

/************************* Debug Functions ***************************/

void debugAMSstate() {
  Serial.printf("AMS_OK: %d\n", AMS_OK);
  Serial.printf("AMS_VOLT: %.2f Low: %d Full: %d \n", AMS_Package.ACCUM_VOLTAGE,ACCUM_LOW,ACCUM_FULL);
  Serial.printf("AMS_MAX: %.2f \n", AMS_Package.ACCUM_MAXVOLTAGE);
  Serial.printf("AMS_MIN: %.2f\n", AMS_Package.ACCUM_MINVOLTAGE);
  Serial.printf("OV_CRIT: %d\n", OVER_VOLT_CRIT);
  Serial.printf("LV_CRIT: %d\n", LOW_VOLT_CRIT);
  Serial.printf("LV_WARN: %d\n", LOW_VOLT_WARN);
  Serial.printf("OT_CRIT: %d\n", OVER_TEMP_CRIT);
  Serial.printf("OT_WARN: %d\n", OVER_TEMP_WARN);
  Serial.printf("DV_CRIT: %d\n", ACCUM_OverDivCritical);
  Serial.printf("DV_WARN: %d\n", ACCUM_OverDivWarn);
}

void debugBMUMod(int moduleNum) {
  Serial.printf("=== BMU %d (ID: %X) ===\n", moduleNum + 1, BMU_Package[moduleNum].BMU_ID);
  Serial.printf("V_MODULE: %.2fV\n", BMU_Package[moduleNum].V_MODULE * 0.02f);
  Serial.print("V_CELL: ");
  for (int i = 0; i < CELL_NUM; i++) {
    Serial.printf("%.2f ", BMU_Package[moduleNum].V_CELL[i] * 0.02f);
  }
  Serial.println("V");

  Serial.printf("DV: %.2fV\n", BMU_Package[moduleNum].DV * 0.2f);
  Serial.printf("TEMP: %.1fv, %.1fv\n",
    BMU_Package[moduleNum].TEMP_SENSE[0] * 0.0125f + 2,
    BMU_Package[moduleNum].TEMP_SENSE[1] * 0.0125f + 2);
  Serial.printf("Ready to Charge: %d, Connected: %d\n",
    BMU_Package[moduleNum].BMUreadytoCharge,
    BMU_Package[moduleNum].BMUconnected);

  Serial.printf("Faults - OV:%X/%X LV:%X/%X OT:%X/%X DV:%X/%X\n",
    BMU_Package[moduleNum].OVERVOLTAGE_WARNING,
    BMU_Package[moduleNum].OVERVOLTAGE_CRITICAL,
    BMU_Package[moduleNum].LOWVOLTAGE_WARNING,
    BMU_Package[moduleNum].LOWVOLTAGE_CRITICAL,
    BMU_Package[moduleNum].OVERTEMP_WARNING,
    BMU_Package[moduleNum].OVERTEMP_CRITICAL,
    BMU_Package[moduleNum].OVERDIV_VOLTAGE_WARNING,
    BMU_Package[moduleNum].OVERDIV_VOLTAGE_CRITICAL);
  Serial.println();
}

void debugOBCmsg() {
  Serial.printf("OBC: %dV, %dA, Status: %02X\n",
    OBC_Package.OBCVolt, OBC_Package.OBCAmp, OBC_Package.OBCstatusbit);

  bool *obcstatbitarray = toBitarrayLSB(OBC_Package.OBCstatusbit);
  if (obcstatbitarray[0]) Serial.println("  HW Fault");
  if (obcstatbitarray[1]) Serial.println("  Overheat");
  if (obcstatbitarray[2]) Serial.println("  AC Reversed");
  if (obcstatbitarray[3]) Serial.println("  No Battery");
  if (obcstatbitarray[4]) Serial.println("  Comm Timeout");
}
