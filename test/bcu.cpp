/* BCU - Battery Control Unit
 * Handles CAN communication with BMU modules and OBC charger
 * Determines AMS_OK status based on fault conditions
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

/************************* Pin Definitions ***************************/
#define CAN_TX_PIN 20
#define CAN_RX_PIN 21
#define OBCIN        9   // Charger plug detect input
#define AMS_OUT      2  // AMS OK output signal

/************************* Global Variables ***************************/
// CAN messages
twai_message_t sendMsg;
twai_message_t receivedMsg;
twai_message_t chargerMsg;
bool canbusready = false;

// Timing
unsigned long reference_time = 0;
unsigned long Sustained_Communicate_Time = 0;
unsigned long shutdown_timer = 0;
unsigned long lastModuleResponse[BMU_NUM];
unsigned long logCount = 0;
unsigned long lastlogtime = 0;

// Hardware timers
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// Data structures
BMUdata BMU_Package[BMU_NUM];
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

// Default BMU parameters (Can change Later)

int transimission_time = BMS_COMMUNICATE_TIME;
float VmaxCell = VMAX_CELL;
float VminCell = VMIN_CELL;
int TempMaxCell = TEMP_MAX_CELL;
float dVmax = DVMAX;
bool BMUUpdateFlag = 0;

/************************* Function Declarations ***************************/
void packBCU_toAMSmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool &is_charger_plugged);
void packBCU_toOBCmsg(twai_message_t *BCUsent, bool &BMS_OK, bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full);
void processReceived_OBCmsg(twai_message_t *receivedframe);
void processReceived_BMUmsg(twai_message_t *receivedframe, BMUdata *BMS_ROSPackage);
void debugAMSstate();
void debugBMUMod(int moduleNum);
void debugOBCmsg();
bool checkModuleDisconnect(BMUdata *BMU_Package);
void resetAllStruct();
bool isModuleActive(int moduleIndex);
void resetModuleData(int moduleIndex);
void packing_AMSstruct(int moduleIndex);
void dynamicModulereset(BMUdata *BMU_Package);

/************************* Timer ISRs ***************************/
void IRAM_ATTR onTimer_dischargeMode() {
  CAN_SEND_FLG1 = true;
}
void IRAM_ATTR onTimer_chargeMode() {
  CAN_SEND_FLG2 = true;
}

/************************* Setup ***************************/
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);

  // GPIO setup
  pinMode(OBCIN, INPUT_PULLDOWN);
  pinMode(AMS_OUT, OUTPUT);

  // CAN bus init
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  canbusready = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, STANDARD_BIT_RATE, filter_config);
  sendMsg.extd = receivedMsg.extd = chargerMsg.extd = true;

  // Timer 1: Discharge mode (1000ms)
  My_timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(My_timer1, &onTimer_dischargeMode, true);
  timerAlarmWrite(My_timer1, BMS_COMMUNICATE_TIME * 1000, true);
  timerAlarmEnable(My_timer1);

  // Timer 2: Charge mode (500ms)
  My_timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer2, &onTimer_chargeMode, true);
  timerAlarmWrite(My_timer2, OBC_COMMUNICATE_TIME * 1000, true);
  timerAlarmEnable(My_timer2);

  Serial.println("BCU__initialized__");
}

/************************* Main Loop ***************************/
void loop() {
  unsigned long SESSION_TIME = millis();
  CHARGER_PLUGGED = false; // Will Use digitalRead(OBCIN) for actual charger detection later

  // CAN TX: BCU broadcast to BMUs
  if (CAN_SEND_FLG1) {
    CAN_SEND_FLG1 = false;
    packBCU_toAMSmsg(&sendMsg, BMS_COMMUNICATE_TIME, CHARGER_PLUGGED);
    CAN32_sendCAN(&sendMsg, canbusready);
  }

  // CAN TX: BCU to OBC (only when charging)
  if (CAN_SEND_FLG2 && CHARGER_PLUGGED) {
    CAN_SEND_FLG2 = false;
    packBCU_toOBCmsg(&chargerMsg, AMS_OK, ACCUM_ReadytoCharge, ACCUM_OverDivCritical, ACCUM_FULL);
    CAN32_sendCAN(&chargerMsg, canbusready);
  }

  // CAN RX processing - drain all pending messages from queue
  while (CAN32_receiveCAN(&receivedMsg, canbusready) == ESP_OK) {
    processReceived_BMUmsg(&receivedMsg, BMU_Package);
    if (CHARGER_PLUGGED) processReceived_OBCmsg(&receivedMsg);
    CAN_TIMEOUT_FLG = false;
    Sustained_Communicate_Time = millis();
  }

  // Update BMU connection status and aggregate to AMS
  AMS_Package = AMSdata();
  for (int j = 0; j < BMU_NUM; j++) {
    if (isModuleActive(j)) {
      BMU_Package[j].BMUconnected = true;
      packing_AMSstruct(j);
    } else {
      BMU_Package[j].BMUconnected = false;
    }
  }

  // CAN bus timeout - complete disconnect
  if (SESSION_TIME - Sustained_Communicate_Time >= DISCONNENCTION_TIMEOUT) {
    digitalWrite(AMS_OUT, LOW);
    Serial.println("NO_BYTE_RECV");
    delay(500);
    if (!CAN_TIMEOUT_FLG) resetAllStruct();
    CAN_TIMEOUT_FLG = true;
    return;
  }
  debugBMUMod(1);
  // Check for individual BMU disconnection
  if (!checkModuleDisconnect(BMU_Package)) {
    digitalWrite(AMS_OUT, LOW);
    Serial.println("THE_FOLLOWING_BMU_ARE_DISCONNECTED -- Please connect before operate:");
    for (int i = 0; i < BMU_NUM; i++) {
      if (!BMU_Package[i].BMUconnected)
        Serial.printf("BMU Module no.%d \n\n", i + 1);
    }
    delay(500);
    return;
  }

  /*=============== AMS_OK Determination ===============*/

  // Voltage fault flags
  OVER_VOLT_CRIT = (AMS_Package.ACCUM_VOLTAGE >= ACCUM_MAXVOLTAGE);
  ACCUM_FULL = (AMS_Package.ACCUM_VOLTAGE >= 0.9 * ACCUM_MAXVOLTAGE);
  LOW_VOLT_CRIT = (AMS_Package.ACCUM_VOLTAGE <= ACCUM_MINVOLTAGE);
  LOW_VOLT_WARN = (AMS_Package.ACCUM_VOLTAGE <= 1.10 * ACCUM_MINVOLTAGE);

  // Temperature fault flags
  OVER_TEMP_CRIT = (AMS_Package.OVERTEMP_CRITICAL > 0);
  OVER_TEMP_WARN = (AMS_Package.OVERTEMP_WARNING > 0);

  // Differential voltage fault flags
  ACCUM_OverDivCritical = (AMS_Package.OVERDIV_CRITICAL > 0);
  ACCUM_OverDivWarn = (AMS_Package.OVERDIV_WARNING > 0);

  // AMS_OK logic: any critical fault triggers shutdown
  bool ACCUMULATOR_Fault = OVER_VOLT_CRIT | LOW_VOLT_CRIT | OVER_TEMP_CRIT | ACCUM_OverDivCritical;
  AMS_OK = !ACCUMULATOR_Fault;

  // Charging mode fault handling
  if (CHARGER_PLUGGED) {
    uint16_t OBCFault = OBC_Package.OBCstatusbit;
    ACCUM_ReadytoCharge = (AMS_Package.ACCUM_CHG_READY > 0);
    AMS_OK = !(ACCUMULATOR_Fault | OBCFault | ACCUM_ReadytoCharge);
  }

  /*=============== Output Control ===============*/
  digitalWrite(AMS_OUT, AMS_OK ? HIGH : LOW);
}

/************************* CAN Message Packing ***************************/
void packBCU_toAMSmsg(twai_message_t *BCUsent, uint16_t bcu_transimission_time, bool &is_charger_plugged) {
  uint8_t* transmission_time = splitHLbyte(bcu_transimission_time);
  BCUsent->identifier = BCU_ADD;
  BCUsent->data_length_code = 8;
  BCUsent->data[0] = transmission_time[0];
  BCUsent->data[1] = transmission_time[0];
  BCUsent->data[2] = is_charger_plugged ? 1 : 0;
  BCUsent->data[3] = (uint8_t) VmaxCell / 0.1;
  BCUsent->data[4] = (uint8_t) VminCell / 0.1;
  BCUsent->data[5] = (byte) TempMaxCell;
  BCUsent->data[6] = (byte) dVmax / 0.1;
  BCUsent->data[7] = BMUUpdateFlag;
}
void packBCU_toOBCmsg(twai_message_t *BCUsent, bool &AMS_OK, bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full) {
  BCUsent->identifier = OBC_ADD;
  BCUsent->data_length_code = 8;
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;

  // Charge if OK, ready, no critical overdiv, and not full
  if ((AMS_OK && ReadytoCharge) || !OverDivCritical_Yes || !Voltage_is_Full) {
    BCUsent->data[0] = 0x18;  // Voltage high byte (240.0V for 6 modules)
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
  dynamicModulereset(BMU_Package);

  extCANIDDecoded decodedCANID;
  decodeExtendedCANID(&decodedCANID, receivedframe->identifier);

  int i = decodedCANID.SRC - 1;
  if (i < 0 || i >= BMU_NUM) return;  // Added lower bound check

  lastModuleResponse[i] = millis();

  // Priority 0x02: BMU module & cell data
  if (decodedCANID.PRIORITY == 0x02) {
    switch (decodedCANID.MSG_NUM) {
      case 1:  // Operation status
        BMU_Package[i].BMUreadytoCharge = receivedframe->data[0];
        BMU_Package[i].BalancingDischarge_Cells = mergeHLbyte(receivedframe->data[1], receivedframe->data[2]);
        BMU_Package[i].V_MODULE = mergeHLbyte(receivedframe->data[3], receivedframe->data[4]);
        BMU_Package[i].DV = receivedframe->data[5];
        BMU_Package[i].TEMP_SENSE[0] = receivedframe->data[6];
        BMU_Package[i].TEMP_SENSE[1] = receivedframe->data[7];
        break;
      case 2:  // Cell voltages C1-C8
        for (short j = 0; j < 8; j++)
          BMU_Package[i].V_CELL[j] = receivedframe->data[j];
        break;
      case 3:  // Cell voltages C9-C10
        for (short j = 8; j < CELL_NUM; j++)
          BMU_Package[i].V_CELL[j] = receivedframe->data[j - 8];
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
bool isModuleActive(int moduleIndex) {
  return (millis() - lastModuleResponse[moduleIndex]) <= DISCONNENCTION_TIMEOUT;
}
void resetModuleData(int moduleIndex) {
  BMU_Package[moduleIndex] = BMUdata();
}
void packing_AMSstruct(int moduleIndex) {
  int &i = moduleIndex;
  AMS_Package.ACCUM_VOLTAGE += static_cast<float>(BMU_Package[i].V_MODULE) * 0.2;
  AMS_Package.OVERTEMP_WARNING |= BMU_Package[i].OVERTEMP_WARNING;
  AMS_Package.OVERTEMP_CRITICAL |= BMU_Package[i].OVERTEMP_CRITICAL;
  AMS_Package.OVERDIV_WARNING |= BMU_Package[i].OVERDIV_VOLTAGE_WARNING;
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL;
  AMS_Package.ACCUM_CHG_READY &= BMU_Package[i].BMUreadytoCharge;
}
void resetAllStruct() {
  for (int i = 0; i < BMU_NUM; i++) {
    resetModuleData(i);
    lastModuleResponse[i] = 0;
  }
  AMS_Package = AMSdata();
  OBC_Package = OBCdata();
}
bool checkModuleDisconnect(BMUdata *BMU_Package) {
  for (short i = 0; i < BMU_NUM; i++) {
    if (!BMU_Package[i].BMUconnected) return false;
  }
  return true;
}
void dynamicModulereset(BMUdata *BMU_Package) {
  for (short i = 0; i < BMU_NUM; i++) {
    if (!BMU_Package[i].BMUconnected) {
      resetModuleData(i);
    }
  }
}

/************************* Debug Functions ***************************/
void debugAMSstate() {
  Serial.printf("AMS_OK: %d\n", AMS_OK);
  Serial.printf("AMS_VOLT: %.2f\n", AMS_Package.ACCUM_VOLTAGE);
  Serial.printf("AMS_MAX: %.2f\n", AMS_Package.ACCUM_MAXVOLTAGE);
  Serial.printf("AMS_MIN: %.2f\n", AMS_Package.ACCUM_MINVOLTAGE);
  Serial.printf("OV_CRIT: %d\n", OVER_VOLT_CRIT);
  Serial.printf("ACCUM_FULL: %d\n", ACCUM_FULL);
  Serial.printf("LV_CRIT: %d\n", LOW_VOLT_CRIT);
  Serial.printf("LV_WARN: %d\n", LOW_VOLT_WARN);
  Serial.printf("OV_TEMP_CRT: %d\n", OVER_TEMP_CRIT);
  Serial.printf("OV_TEMP_WARN: %d\n", OVER_TEMP_WARN);
  Serial.printf("OV_DIV_CRT: %d\n", ACCUM_OverDivCritical);
  Serial.printf("OV_DIV_WARN: %d\n", ACCUM_OverDivWarn);
}


void debugBMUMod(int moduleNum) {
  Serial.printf("BMU ID: %X\n", BMU_Package[moduleNum].BMU_ID);
  Serial.printf("BMU V_MODULE: %.2f\n", BMU_Package[moduleNum].V_MODULE);
    Serial.print("BMU_VCELL[10]: ");
  for (short i = 0; i < CELL_NUM; i++) {
    Serial.print(BMU_Package[moduleNum].V_CELL[i] * 0.02);
    Serial.print("V.  ");
  } Serial.println();

  Serial.printf("BMU DV: %.2f\n", BMU_Package[moduleNum].DV *0.2);
  Serial.print("TEMP[2]: ");
  Serial.print(BMU_Package[moduleNum].TEMP_SENSE[0] * 0.6 + 2);
  Serial.print("C.  ");
  Serial.print(BMU_Package[moduleNum].TEMP_SENSE[1] * 0.6 + 2);
  Serial.println("C.  ");
  Serial.printf("BMU chargeMode: %d\n", BMU_Package[moduleNum].BMUreadytoCharge);
  Serial.printf("BMU connect: %d\n", BMU_Package[moduleNum].BMUconnected);

  Serial.printf("LV_CRIT: %X\n", BMU_Package[moduleNum].LOWVOLTAGE_WARNING);
  Serial.printf("LV_WARN: %X\n", BMU_Package[moduleNum].LOWVOLTAGE_CRITICAL);
  Serial.printf("OV_TEMP_WARN: %X\n", BMU_Package[moduleNum].OVERTEMP_WARNING);
  Serial.printf("OV_TEMP_CRIT: %X\n", BMU_Package[moduleNum].OVERTEMP_CRITICAL);
  Serial.printf("OV_WARN: %X\n", BMU_Package[moduleNum].OVERVOLTAGE_WARNING);
  Serial.printf("OV_CRIT: %X\n", BMU_Package[moduleNum].OVERVOLTAGE_CRITICAL);
  Serial.printf("OV_DIV_WARN: %X\n", BMU_Package[moduleNum].OVERDIV_VOLTAGE_WARNING);
  Serial.printf("OV_DIV_CRIT: %X\n", BMU_Package[moduleNum].OVERDIV_VOLTAGE_CRITICAL);
  Serial.println();
}

void debugOBCmsg() {
  Serial.printf("Voltage from OBC: %dV\n", OBC_Package.OBCVolt);
  Serial.printf("Current from OBC: %dA\n", OBC_Package.OBCAmp);
  Serial.printf("OBC status bit: %d\n", OBC_Package.OBCstatusbit);

  bool *obcstatbitarray = toBitarrayLSB(OBC_Package.OBCstatusbit);
  if (obcstatbitarray[0]) Serial.println("ChargerHW = Faulty");
  if (obcstatbitarray[1]) Serial.println("ChargerTemp = Overheat");
  if (obcstatbitarray[2]) Serial.println("ChargerACplug = Reversed");
  if (obcstatbitarray[3]) Serial.println("Charger detects: NO BATTERY VOLTAGES");
  if (obcstatbitarray[4]) Serial.println("OBC Detect COMMUNICATION Timeout (6s)");
}
