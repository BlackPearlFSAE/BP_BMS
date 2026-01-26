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
#include "ams_data_util.h"
#include "bms_helper.h"

/************************* Pin Definitions ***************************/
#define CAN_TX_PIN   48
#define CAN_RX_PIN   47
#define OBCIN        14  // Charger plug detect input
#define AMS_OUT      21  // AMS OK output signal

/************************* Global Variables ***************************/
// CAN messages
twai_message_t sendMsg;
twai_message_t receivedMsg;
twai_message_t chargerMsg;
bool canbusready = false;

// Timing
unsigned long Sustained_Communicate_Time = 0;
unsigned long lastModuleResponse[MODULE_NUM];
uint32_t shutdown_timer = 0;
uint32_t debug_timer = 0;
uint32_t aggregation_timer = 0;

// Hardware timers
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// Data structures
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
bool ACCUM_ReadytoCharge = false;
bool ACCUM_OverDivWarn = false;
bool ACCUM_OverDivCritical = false;
bool OVER_TEMP_WARN = false;
bool OVER_TEMP_CRIT = false;
bool LOW_VOLT_CRIT = false;
bool LOW_VOLT_WARN = false;
bool OVER_VOLT_CRIT = false;
bool OVER_VOLT_WARN = false;
bool ACCUM_FULL = false;
bool ACCUM_LOW = false;

// BMU parameters (configurable)
int transimission_time = BMS_COMMUNICATE_TIME;
float VmaxCell = VMAX_CELL;
float VminCell = VMIN_CELL;
int TempMaxCell = TEMP_MAX_CELL;
float dVmax = DVMAX;
bool BMUUpdateFlag = false;

/************************* Function Declarations ***************************/
void packBCU_toBMUmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool is_charger_plugged);
void packBCU_toOBCmsg(twai_message_t *BCUsent, bool AMS_OK, bool ReadytoCharge, bool OverDivCritical_Yes, bool Voltage_is_Full);
void processReceived_BMUmsg(twai_message_t *receivedframe, BMUdata *BMU_Package);
void processReceived_OBCmsg(twai_message_t *receivedframe);
void packing_AMSstruct(int moduleIndex);
bool isModuleActive(int moduleIndex);
bool checkModuleDisconnect(BMUdata *BMU_Package);
void resetAllStruct();

bool checkModuleDisconnect(BMUdata *BMU_Package);


/************************* Timer ISRs ***************************/
void IRAM_ATTR onTimer_dischargeMode() {
  CAN_SEND_FLG1 = true;
}

void IRAM_ATTR onTimer_chargeMode() {
  CAN_SEND_FLG2 = true;
}

/************************* Setup *******************************/



void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // Disable brownout detector
  Serial.begin(115200);

  // GPIO setup
  pinMode(OBCIN, INPUT_PULLDOWN);
  pinMode(AMS_OUT, OUTPUT);
  digitalWrite(AMS_OUT, LOW);  // Start with AMS_OK = false

  // Initialize module response timestamps (0 = never received)
  for (int i = 0; i < MODULE_NUM; i++) {
    lastModuleResponse[i] = 0;
  }

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
#define DEBUG_MODE 1 // Mode 1 = Regular, Mode 2 = Teleplot
void loop() {
  unsigned long SESSION_TIME = millis();
  // CHARGER_PLUGGED = digitalRead(OBCIN);
  CHARGER_PLUGGED = false;

  #if DEBUG_MODE == 1
  if (SESSION_TIME - debug_timer >= 500) {
    for (int j = 0; j < MODULE_NUM; j++) debugBMUModule(&BMU_Package[j],j);
    // debugAMSstate(&AMS_Package);
    debug_timer = millis();
  }
  #endif 
  #if DEBUG_MODE == 2
  if (SESSION_TIME - teleplot_timer >= 500) {
    // Teleplot: AMS aggregate data
    teleplotAMSstate(&AMS_Package);
    // Teleplot: All module voltages overview
    teleplotAllModules(BMU_Package, MODULE_NUM);
    // Teleplot: BCU local cell voltages (raw float)
    teleplotLocalCells(cellvoltages, NUM_CELLS, "BCU");
    // Teleplot: BCU local temperatures
    Serial.printf(">BCU_T1:%.1f|BCU_T2:%.1f\n", currentTemp1, currentTemp2);
    // Teleplot: System status flags
    Serial.printf(">ChgPlugged:%d|AMS_OK:%d\n", CHARGER_PLUGGED ? 1 : 0, AMS_OK ? 1 : 0);
    Serial.printf(">AccumFull:%d|AccumLow:%d\n", ACCUM_FULL ? 1 : 0, ACCUM_LOW ? 1 : 0);

    teleplot_timer = millis();
  }
  #endif

  /*==================== CAN TX ====================*/

  // Broadcast to BMUs (every 1000ms)
  if (CAN_SEND_FLG1) {
    CAN_SEND_FLG1 = false;
    packBCU_toBMUmsg(&sendMsg, BMS_COMMUNICATE_TIME, CHARGER_PLUGGED);
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
    Sustained_Communicate_Time = millis();
  }
  // Complete communication loss check
  else if (SESSION_TIME - Sustained_Communicate_Time >= DISCONNENCTION_TIMEOUT) {
    digitalWrite(AMS_OUT, LOW);
    if (SESSION_TIME - shutdown_timer >= 500) {
      Serial.println("NO_BYTE_RECV");
      shutdown_timer = millis();
    }
    resetAllStruct();
    return;
  }

  // Check each BMU connection status
  bool allConnected = true;
  for (int i = 0; i < MODULE_NUM; i++) {
    BMU_Package[i].BMUconnected = isModuleActive(i);
    if (!BMU_Package[i].BMUconnected) allConnected = false;
  }

  // Handle BMU disconnection
  if (!allConnected) {
    digitalWrite(AMS_OUT, LOW);
    
    if (SESSION_TIME - debug_timer >= 500) {
      Serial.println("BMU_DISCONNECTED:");
      for (int i = 0; i < MODULE_NUM; i++) {
        if (!BMU_Package[i].BMUconnected) Serial.printf("  Module %d\n", i + 1); 
      }
      debug_timer = millis();
    }
    AMS_Package = AMSdata();
    OBC_Package = OBCdata();
    return;
  }

  /*==================== Periodic Aggregation (500ms) ====================*/

  if (SESSION_TIME - aggregation_timer >= 500) {
    // Reset AMS package for fresh aggregation
    AMS_Package = AMSdata();

    for (int j = 0; j < MODULE_NUM; j++) {
      if (BMU_Package[j].BMUconnected) {
        // Calculate module voltage (sum of cells)
        BMU_Package[j].V_MODULE = 0;
        for (int k = 0; k < CELL_NUM; k++) {
          BMU_Package[j].V_MODULE += BMU_Package[j].V_CELL[k];
        }
        // Pack into AMS aggregate
        packing_AMSstruct(j);
      }
    }
    aggregation_timer = SESSION_TIME;
  }

  /*==================== Real-time Connection Check ====================*/

  
  

  /*==================== AMS_OK Determination ====================*/

  // Voltage fault flags
  ACCUM_FULL = (AMS_Package.ACCUM_VOLTAGE >= 0.95f * ACCUM_MAXVOLTAGE);
  ACCUM_LOW = (AMS_Package.ACCUM_VOLTAGE >= 1.12f * ACCUM_MINVOLTAGE);
  OVER_VOLT_CRIT = (AMS_Package.OVERVOLT_CRITICAL > 0);
  OVER_VOLT_WARN = (AMS_Package.OVERVOLT_WARNING > 0);
  LOW_VOLT_CRIT = (AMS_Package.LOWVOLT_CRITICAL > 0);
  LOW_VOLT_WARN = (AMS_Package.LOWVOLT_WARNING > 0);

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

  /*==================== Output Control ====================*/

  digitalWrite(AMS_OUT, AMS_OK ? HIGH : LOW);
}

/************************* CAN Message Packing ***************************/

void packBCU_toBMUmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool is_charger_plugged) {
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
  if (i < 0 || i >= MODULE_NUM) return;

  // Update timestamp and ID
  lastModuleResponse[i] = millis();
  BMU_Package[i].BMU_ID = receivedframe->identifier;

  // Priority 0x02: BMU module & cell data
  if (decodedCANID.PRIORITY == 0x02) {
    switch (decodedCANID.MSG_NUM) {
      case 1:  // Operation status
        BMU_Package[i].BMUneedBalance = receivedframe->data[0];
        BMU_Package[i].BalancingDischarge_Cells = mergeHLbyte(receivedframe->data[1], receivedframe->data[2]);
        BMU_Package[i].DV = receivedframe->data[3];
        BMU_Package[i].TEMP_SENSE[0] = mergeHLbyte(receivedframe->data[4],
                                                    receivedframe->data[5]);
        BMU_Package[i].TEMP_SENSE[1] = mergeHLbyte(receivedframe->data[7],
                                                    receivedframe->data[8]);
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
  AMS_Package.OVERVOLT_WARNING |= BMU_Package[k].OVERVOLTAGE_WARNING;
  AMS_Package.OVERVOLT_CRITICAL |= BMU_Package[k].OVERVOLTAGE_CRITICAL;
  AMS_Package.LOWVOLT_WARNING |= BMU_Package[k].LOWVOLTAGE_WARNING;
  AMS_Package.LOWVOLT_CRITICAL |= BMU_Package[k].LOWVOLTAGE_CRITICAL;

  AMS_Package.OVERTEMP_WARNING |= BMU_Package[k].OVERTEMP_WARNING;
  AMS_Package.OVERTEMP_CRITICAL |= BMU_Package[k].OVERTEMP_CRITICAL;
  AMS_Package.OVERDIV_WARNING |= BMU_Package[k].OVERDIV_VOLTAGE_WARNING;
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[k].OVERDIV_VOLTAGE_CRITICAL;

  // AND together charge ready (all must be ready)
  AMS_Package.ACCUM_CHG_READY &= BMU_Package[k].BMUneedBalance;
}

void resetAllStruct() {
  for (int i = 0; i < MODULE_NUM; i++) {
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
  for (short i = 0; i < MODULE_NUM; i++) {
    if (!BMU_Package[i].BMUconnected) return true;
  }
  return false;
}


