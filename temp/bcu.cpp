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
#include "ams_config.h"
#include "helper.h"

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
bool CAN_TIMEOUT_FLG = false;
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
void mockBMUData();
void packBCU_toBMUmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool is_charger_plugged);
void packBCU_toOBCmsg(twai_message_t *BCUsent, bool AMS_OK, bool ReadytoCharge, bool OverDivCritical_Yes, bool Voltage_is_Full);
void processReceived_BMUmsg(twai_message_t *receivedframe, BMUdata *BMU_Package);
void processReceived_OBCmsg(twai_message_t *receivedframe);
void packing_AMSstruct(int moduleIndex);
bool isModuleActive(int moduleIndex);
bool checkModuleDisconnect(BMUdata *BMU_Package);
void resetAllStruct();

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
void loop() {
  unsigned long SESSION_TIME = millis();
  // CHARGER_PLUGGED = digitalRead(OBCIN);
  CHARGER_PLUGGED = false;

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

  /*==================== Real-time Connection Check ====================*/

  // Loop 1: Set BMUconnected
  // for (int i = 0; i < MODULE_NUM; i++) {
  //   BMU_Package[i].BMUconnected = isModuleActive(i);
  // }
  // if(checkModuleDisconnect(BMU_Package)){
  //   digitalWrite(AMS_OUT, LOW);
  //   if (SESSION_TIME - debug_timer >= 500) {
  //     Serial.println("BMU_DISCONNECTED:");
  //     for (int i = 0; i < MODULE_NUM; i++) {
  //       if(!BMU_Package[i].BMUconnected) Serial.printf("  Module %d\n", i + 1); 
  //     }
  //     debug_timer = millis();
  //   }
  //   return;
  // }
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

  /*==================== Debug Output (500ms) ====================*/

  if (SESSION_TIME - debug_timer >= 500) {
    // for (int j = 0; j < MODULE_NUM; j++) debugBMUMod(j);
    debugAMSstate();
    debug_timer = millis();
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

/************************* Debug Functions ***************************/

void debugAMSstate() {
  Serial.printf("AMS_OK: %d\n", AMS_OK);
  Serial.printf("AMS_VOLT: %.2f Low: %d Full: %d \n", AMS_Package.ACCUM_VOLTAGE,ACCUM_LOW,ACCUM_FULL);
  Serial.printf("AMS_MAX: %.2f \n", AMS_Package.ACCUM_MAXVOLTAGE);
  Serial.printf("AMS_MIN: %.2f\n", AMS_Package.ACCUM_MINVOLTAGE);
  Serial.printf("OV_WARN: %d\n", AMS_Package.OVERVOLT_WARNING);
  Serial.printf("OV_CRIT: %d\n", OVER_VOLT_CRIT);
  Serial.printf("LV_WARN: %d\n", LOW_VOLT_WARN);
  Serial.printf("LV_CRIT: %d\n", LOW_VOLT_CRIT);
  Serial.printf("OT_WARN: %d\n", OVER_TEMP_WARN);
  Serial.printf("OT_CRIT: %d\n", OVER_TEMP_CRIT);
  Serial.printf("DV_WARN: %d\n", ACCUM_OverDivWarn);
  Serial.printf("DV_CRIT: %d\n", ACCUM_OverDivCritical);
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
    BMU_Package[moduleNum].BMUneedBalance,
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

void mockBMUData() {
    // Half identical (good modules)
    for (int i = 0; i < MODULE_NUM / 2; i++) {
        BMU_Package[i].BMU_ID = 0x18200001 + (i << 16);
        BMU_Package[i].BMUconnected = true;
        BMU_Package[i].BMUneedBalance = 1;
        BMU_Package[i].DV = 5;  // 0.5V diff (5 * 0.1)
        BMU_Package[i].TEMP_SENSE[0] = 0xC8;  // ~25C
        BMU_Package[i].TEMP_SENSE[1] = 0xC8;
        
        // All cells at 3.7V (185 * 0.02 = 3.7V)
        for (int j = 0; j < CELL_NUM; j++) {
            BMU_Package[i].V_CELL[j] = 185;
        }
        
        // No faults
        BMU_Package[i].OVERVOLTAGE_WARNING = 0x0000;
        BMU_Package[i].OVERVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].LOWVOLTAGE_WARNING = 0x0000;
        BMU_Package[i].LOWVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].OVERTEMP_WARNING = 0x0000;
        BMU_Package[i].OVERTEMP_CRITICAL = 0x0000;
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = 0x0000;
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].BalancingDischarge_Cells = 0x0000;
        
        lastModuleResponse[i] = millis();
    }
    
    // Half mixed bag (faulty modules)
    for (int i = MODULE_NUM / 2; i < MODULE_NUM; i++) {
        BMU_Package[i].BMU_ID = 0x18200001 + (i << 16);
        BMU_Package[i].BMUconnected = true;
        BMU_Package[i].BMUneedBalance = 0;  // Not ready
        BMU_Package[i].DV = 15;  // 1.5V diff - high deviation
        BMU_Package[i].TEMP_SENSE[0] = 0xFA;  // Hot ~60C
        BMU_Package[i].TEMP_SENSE[1] = 0xD0;  // Warm ~40C
        
        // Cell deviation: some high, some low
        BMU_Package[i].V_CELL[0] = 210;  // 4.2V (high)
        BMU_Package[i].V_CELL[1] = 205;  // 4.1V
        BMU_Package[i].V_CELL[2] = 160;  // 3.2V (low)
        BMU_Package[i].V_CELL[3] = 185;  // 3.7V
        BMU_Package[i].V_CELL[4] = 190;  // 3.8V
        BMU_Package[i].V_CELL[5] = 155;  // 3.1V (low)
        BMU_Package[i].V_CELL[6] = 200;  // 4.0V
        BMU_Package[i].V_CELL[7] = 185;  // 3.7V
        BMU_Package[i].V_CELL[8] = 195;  // 3.9V
        BMU_Package[i].V_CELL[9] = 175;  // 3.5V
        
        // Faults: Cell 1 OV warning, Cell 3 & 6 LV warning
        BMU_Package[i].OVERVOLTAGE_WARNING = 0x0200;   // 0b0000000010 << 1 = Cell 1
        BMU_Package[i].OVERVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].LOWVOLTAGE_WARNING = 0x0090;    // 0b0010010000 = Cell 3 & 6
        BMU_Package[i].LOWVOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].OVERTEMP_WARNING = 0x0200;      // Cell 1 area hot
        BMU_Package[i].OVERTEMP_CRITICAL = 0x0000;
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = 0x0094;  // Cells with deviation
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = 0x0000;
        BMU_Package[i].BalancingDischarge_Cells = 0x0201; // Balancing Cell 1 & 10
        
        lastModuleResponse[i] = millis();
    }
    
    // Make last module disconnected (timeout)
    if (MODULE_NUM > 1) {
        lastModuleResponse[MODULE_NUM - 1] = millis() - DISCONNENCTION_TIMEOUT - 100;
        BMU_Package[MODULE_NUM - 1].BMUconnected = false;
    }
}