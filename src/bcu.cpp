/* BCU
Minimum Voltage of 2 Module : 3.1*20 = 62V , Nominal Volage 3.6*20 = 72V
Maximum allowable Voltage for 2 module : 83V => 830 => 0x 03 3E
Maximum allowable current for 2 module : 5A => 50 => 0x 00 32
Priority Table, Check Charge -> NO
Read From SDC
Read from BMU
Read from BAMOCAR
RTOS and Push ROS topics
*/
/************************* Includes ***************************/
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <driver/gpio.h>
#include <driver/twai.h>       
#include <Arduino.h>
#include <SPI.h>
#include "driver/twai.h"
#include "LTC6811.h"
#include "LTC681x.h"
#include "Linduino.h"

#include <EEPROM.h>
#include <new>

// File system and ESP32 SPI SD lib
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// utility function
#include <helper.h>
#include <ams_config.h>
#include "CAN32_util.h"

/************************* Define macros *****************************/
#define TWAI_RX_PIN  GPIO_NUM_13 // This will be 
#define TWAI_TX_PIN  GPIO_NUM_14
#define OBCIN GPIO_NUM_9   // Check OK Signal from Charging Shutdown Circuit, indicates that it is charged
// Digital Output 
#define AMS_OUT GPIO_NUM_21 // SL 1 => Check with multimeter

// Macros
#define OBC_SYNC_TIME 500
#define SYNC_TIME 500
/**************** Setup Variables *******************/
twai_message_t sendMessage;
twai_message_t receivedMessage;
twai_message_t J1939msg;

// Software Timer reference (Iniit zero)
unsigned long reference_time = 0; 
unsigned long communication_timer1 = 0;
unsigned long shutdown_timer = 0;
unsigned long lastModuleResponse[BMU_NUM];
unsigned long logCount = 0;
unsigned long lastlogtime = 0; 

// Hardware Timer object
hw_timer_t *My_timer1 = NULL;
hw_timer_t *My_timer2 = NULL;

// BMU data , Accumulator data structure, Sensing data.
BMUdata BMU_Package[BMU_NUM];
AMSdata AMS_Package;
OBCdata OBC_Package;

// Alias names
bool &AMS_OK = AMS_Package.AMS_OK;
float &ACCUM_MAXVOLTAGE = AMS_Package.ACCUM_MAXVOLTAGE; 
float &ACCUM_MINVOLTAGE = AMS_Package.ACCUM_MINVOLTAGE;  

// Flags
volatile bool CAN_SEND_FLG1 = false;
volatile bool CAN_SEND_FLG2 = false;
bool CHARGER_PLUGGED = false;
bool CAN_TIMEOUT_FLG = false;
// Determine BMU Charging Mode
bool ACCUM_ReadytoCharge = false;
bool ACCUM_OverDivWarn = 0;
bool ACCUM_OverDivCritical = 0;
bool OVER_TEMP_WARN = 0;
bool OVER_TEMP_CRIT = 0;
bool LOW_VOLT_CRIT = 0;
bool LOW_VOLT_WARN = 0; // Voltage sag
bool OVER_VOLT_CRIT = 0; 
bool ACCUM_FULL = 0; // Aka OVER_VOLT_WARN

// Default Parameter preparing for BMU to save in its non-volatile memory.
const byte Sync = SYNC_TIME; 
const byte VmaxCell = (byte) (VMAX_CELL / 0.1) ;
const byte VminCell = (byte) (VMIN_CELL / 0.1) ;
const byte TempMaxCell = (TEMP_MAX_CELL);
const byte dVmax = (DVMAX / 0.1); 
const bool BMUUpdateEEPROM = 0; // Flag for BMU to update its EEPROM

// Assign pin for SD Card pins for ESP32S3devkit-c1 board (Green Board)
#define SD_SCK  39
#define SD_MISO 40
#define SD_MOSI 41
#define SD_CS   38

// Data logging configuration
#define LOG_INTERVAL_MS 1000  // How often to write summary data (every 5 seconds)
#define CSV_BMU_HEADER "Time,bmu_id,bmu_volt,bmu_temp1,bmu_temp2,bmu_dv,bmu_connect,bmu_ready_chg,bmu_cell_in_balance,bmu_ov_crti,bmu_ov_warn,bmu_lv_crit,bmu_lv_warn,bmu_ovt_crit,bmu_ovt_warn,bmu_ovd_crit,bmu_ovd_warn,Cell1,Cell2,Cell3,Cell4,Cell5,Cell6,Cell7,Cell8,Cell9,Cell10\n"
#define CSV_AMS_HEADER "Time,ams_ok,ams_volt,ams_ov_crit,ams_ov_wanr,ams_lv_crit,ams_lv_warn,ams_ovt_crit,ams_ovt_warn,ams_ovd_crit,ams_ovd_warn\n"
// #define CSV_AMS_HEADER "Time,accel_ped1,accel_ped2,break_ped1,break_ped2,current_A,bspd_in,imd_in,air+,emr_o,obc_in,temp_light,lv_light,ams_ok,ams_volt,ams_ov_crit,ams_ov_wanr,ams_lv_crit,ams_lv_warn,ams_ovt_crit,ams_ovt_warn,ams_ovd_crit,ams_ovd_warn\n"
// #define CSV_BMU_package "/bmu_log.csv"
#define CSV_AMS_filename "/amsMaster.csv"
#define CSV_BMU_filename "/bmu"

/**************** Local Function Declaration *******************/
// BCU/AMS message packing
void packAMSmsg(twai_message_t *BCUsent, uint16_t Sync_time, bool &is_charger_plugged);
void packBCU_OBCmsg(twai_message_t *BCUsent, bool &BMS_OK, bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full);

// Message processing
void processOBCmsg(twai_message_t *receivedframe);
void processBMUmsg(twai_message_t *receivedframe, BMUdata *BMS_ROSPackage);

// Debug functions
void debugBMUmsg(int Module);
void debugBMUFault(int Module);
void debugOBCmsg();

// Module state management
bool checkModuleDisconnect(BMUdata *BMU_Package);
void resetAllStruct();
bool isModuleActive(int moduleIndex);
void resetModuleData(int moduleIndex);
void packing_AMSstruct(int moduleIndex);
void dynamicModulereset(BMUdata *BMU_Package);

/*******************************************************************
  ==============================Setup==============================
********************************************************************/
void IRAM_ATTR onTimer_dischargeMode() {
  // May or may not be critical section , --- later to be thought out
  CAN_SEND_FLG1 = 1;
}
void IRAM_ATTR onTimer_chargeMode() {
  CAN_SEND_FLG2 = 1;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  // Setup AMS output  
    pinMode(OBCIN,INPUT_PULLDOWN); // Pulldown to prevent false trip
    pinMode(AMS_OUT,OUTPUT);
    // pinMode(LVlight,OUTPUT);
    // pinMode(TEMPlight,OUTPUT);
      
  /* CAN Communication Setup */
  sendMessage.extd = false;
  receivedMessage.extd = false;
  J1939msg.extd = true;
  twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(TWAI_TX_PIN, TWAI_RX_PIN, TWAI_MODE_NORMAL);
  general_config.tx_queue_len = 2800; // worstcase is 152 bit/frame , this should hold about 5 frame
  general_config.rx_queue_len = 2800; // RX queue hold about 8 frame
  twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // May config this later
  // Install the TWAI driver
  if (twai_driver_install(&general_config, &timing_config, &filter_config) == !ESP_OK) {
    Serial.println("TWAI Driver install failed__");
    while(1);
  }
  // Start the TWAI driver
    if (twai_start() == ESP_OK) {
      Serial.println("TWAI Driver installed , started");
      // Reconfigure the alerts to detect the error of frames received, Bus-Off error and RX queue full error
      uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
      if (twai_reconfigure_alerts(alerts_to_enable, NULL) == !ESP_OK) {
        Serial.println("Failed to reconfigure alerts");
        while(1);
      }
    }

  // Setup timer interrupt every 200ms (Discharge Mode timer)
  My_timer1 =  timerBegin(0, 80, true);  // 80 pre
  timerAttachInterrupt(My_timer1, &onTimer_dischargeMode, true);
  timerAlarmWrite(My_timer1, SYNC_TIME * 1000, true);  
  timerAlarmEnable(My_timer1);

  // Setup timer interrupt every 500ms (Charge Mode timer)
  My_timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(My_timer2, &onTimer_chargeMode, true);
  timerAlarmWrite(My_timer2, OBC_SYNC_TIME * 1000, true);  
  timerAlarmEnable(My_timer2);

  // /* --------------------------------------------- Initialize SD card */
  // SD_SPI_init( SD_SCK, SD_MISO, SD_MOSI, SD_CS );
  
  // if (!SD.exists(CSV_AMS_filename)) {
  //   Serial.println("Create AMS Master log file success");
  //   writeFile(SD, CSV_AMS_filename, CSV_AMS_HEADER);
  // } else {
  //   Serial.println("Log file exists, appending data");
  // }
  
  // char CSV_BMU_package[10];  // Buffer for filename (max: "8.csv" → ~6 bytes needed, 15 for safety)
  // // Create 8 file of BMU_Package
  // for(int i=0; i<BMU_NUM;i++) {
  //   snprintf(CSV_BMU_package, sizeof(CSV_BMU_package), "%s%d.csv", CSV_AMS_filename, i);
  //   if (!SD.exists(CSV_BMU_package)) {
  //     Serial.println("Create BMU Cells log success");
  //     writeFile(SD, CSV_BMU_package, CSV_BMU_HEADER);
  //   } else {
  //     Serial.println("Log file exists, appending data");
  //   }
  // }
  
    Serial.println("BCU__initialized__"); 
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/
  
void loop(){
  unsigned long currentMillis = millis();
// Check if Charger LV AUX plug is actually into ACCUM 2nd Floor connector (May change to external interrupt)
  (digitalRead(OBCIN)) ? (CHARGER_PLUGGED = true) : (CHARGER_PLUGGED = false);
  // AMS_Package.OBC_connect = CHARGER_PLUGGED;
/*___Task 1 : Communication ====================================================*/
  // BCU CMD & SYNC   (100ms cycle Broadcast to all BMU modules in Bus) 
  if (CAN_SEND_FLG1)
  {
    CAN_SEND_FLG1 = 0; // reset
    packAMSmsg(&sendMessage, SYNC_TIME , CHARGER_PLUGGED);
    twai_transmit(&sendMessage, 1);
    // Serial.println(1);
  } 
  if(CAN_SEND_FLG2 && CHARGER_PLUGGED)
  {
    CAN_SEND_FLG2 = 0; // reset
    packBCU_OBCmsg(&J1939msg, AMS_OK, ACCUM_ReadytoCharge , ACCUM_OverDivCritical , ACCUM_FULL);
    // **NOTE** 3rd to 5th Parameter will be updated via unpackBMUmsgtoAMS(); function
    twai_transmit(&J1939msg, 1);
  }
//------------------------------------CANRX

  //-------- Normal Receiving
  if (twai_receive(&receivedMessage, 1) == ESP_OK) 
  {
    // Reset BMU data for the one that has disconnected from CAN Bus
    // Unpack BMU frame and insert to BMU_Package[i] , AMS_Package:
    processBMUmsg(&receivedMessage, BMU_Package); // 200ms cycle & 500ms cycle of faultcode

    // Reset Accumulator Parameter before dynamically recalculate based on BMU current state
    AMS_Package = AMSdata();
    // Pack BMUframe to AMSframe according to the following condition
    for(int j = 0; j <BMU_NUM ; j++)
    {
      // if Module is connected to CANbus, set as connect, and recalculateAMS package a new
      if(isModuleActive(j)){ 
          BMU_Package[j].BMUconnected = 1;
          packing_AMSstruct(j);
      } else {
          BMU_Package[j].BMUconnected = 0;
      }
    }

    if(CHARGER_PLUGGED)
      processOBCmsg(&J1939msg); // Unpack CAN frame and insert to OBC_Package:  500ms cycle 
      
    // Update timeout flag and communication_timer
    CAN_TIMEOUT_FLG = false;
    communication_timer1 = millis();
  }
  //-------- Case of AMS is disconnected from CAN Bus
  else if (currentMillis - communication_timer1 >= DISCONNENCTION_TIMEOUT){
    // Loop the Shutdown state until the CAN Bus is active again
    digitalWrite(AMS_OUT,LOW);
    if( currentMillis - shutdown_timer >= 400 ){
        Serial.println("NO_BYTE_RECV");
        shutdown_timer = millis();
    }
    // resetAllStruct() only once
    if(CAN_TIMEOUT_FLG == false)
      resetAllStruct();
    // Mark flag true to bypass resetAllstruct() afterward.
    CAN_TIMEOUT_FLG = true;
    return;
  }

  //-------- Case of Any BMU module disconnected from CAN Bus
  if(checkModuleDisconnect(BMU_Package) == 0){
    // Loop the Shutdown state until the all BMU Modules are connected
    digitalWrite(AMS_OUT,LOW);
    if( currentMillis - shutdown_timer >= 400){

      Serial.println("THE_FOLLOWING_BMU_ARE_DISCONNECTED -- Please connect before operate:");
      for(int i=0; i < BMU_NUM ; i++ ){
        if(!BMU_Package[i].BMUconnected){
          Serial.printf("BMU Module no.%d \n\n", i+1);
        }
      }
        shutdown_timer = millis();
    }
    return;
  }
  
/*___Task 2 : Determine AMS_OK relay state ==================================================== */

    // Fault Table, OR-ed each type of Fault code bit which has been filled up when unpackBMUmsgtoAMS();
    // Old format of using both runtime calculation of ACCUM_VOLTAGE to determine VOLT_WARN & CRIT FLAG
    // OR AMS_Package flag that is result of ORing all VOLT_WARN & CRIT of all BMU Module
      // (AMS_Package.ACCUM_VOLTAGE >= ACCUM_MAXVOLTAGE) || (AMS_Package.OVERVOLT_CRITICAL)
      
    // Calculate OVERVOLT CRITITAL AND WARN FLAG , based on current voltage
    ( (AMS_Package.ACCUM_VOLTAGE >= ACCUM_MAXVOLTAGE)) 
              ? (OVER_VOLT_CRIT = 1) : (OVER_VOLT_CRIT = 0) ;
    ( AMS_Package.ACCUM_VOLTAGE >= 0.9 * ACCUM_MAXVOLTAGE) 
              ? (ACCUM_FULL = 1) : (ACCUM_FULL = 0);
    ( (AMS_Package.ACCUM_VOLTAGE <= ACCUM_MINVOLTAGE)) 
              ? (LOW_VOLT_CRIT = 1) : (LOW_VOLT_CRIT = 0);
    ( (AMS_Package.ACCUM_VOLTAGE <= 1.10 * ACCUM_MINVOLTAGE)) 
              ? (LOW_VOLT_WARN = 1) : (LOW_VOLT_WARN = 0);
    
    // Check overtemp critical and warning
    ( AMS_Package.OVERTEMP_CRITICAL > 0 ) ? (OVER_TEMP_CRIT = 1): (OVER_TEMP_CRIT = 0);
    ( (AMS_Package.OVERTEMP_WARNING > 0)) ? (OVER_TEMP_WARN = 1) : (OVER_TEMP_WARN = 0);
    
    // if differential voltage between any cells are critical, then at AMS level is fault
    (AMS_Package.OVERDIV_CRITICAL > 0) ? (ACCUM_OverDivCritical = 1) : (ACCUM_OverDivCritical = 0);
    (AMS_Package.OVERDIV_WARNING > 0) ? (ACCUM_OverDivWarn = 1) : (ACCUM_OverDivWarn = 0);

    // Dashboard light & Steering wheel display: LowVoltage, Module OverTemperature, Full Voltage
    // OVER_TEMP_CRIT = 0;OVER_TEMP_WARN = 0; OVER_VOLT_CRIT = 0; OVER_VOLT_WARN = 0;

    // Fault Condition for AMS_OK Shutdown
    bool ACCUMULATOR_Fault;
    ACCUMULATOR_Fault = OVER_VOLT_CRIT | LOW_VOLT_CRIT | OVER_TEMP_CRIT | ACCUM_OverDivCritical;
    (ACCUMULATOR_Fault > 0) ? (AMS_OK = 0) : (AMS_OK = 1);

/*------- Accumulator possible fault during charging ------------*/
    if(CHARGER_PLUGGED) { 
      // if OBC has no fault, and Accumulator has no fault , and All BMU cells are balanced
      uint16_t OBCFault = OBC_Package.OBCstatusbit; // 5 bit I think?
      // Set ReadytoCharge Flag, only if All BMU are ready to charge.
      (AMS_Package.ACCUM_CHG_READY > 0) ? (ACCUM_ReadytoCharge = 1) : (ACCUM_ReadytoCharge = 0);
      
      ((ACCUMULATOR_Fault | OBCFault | ACCUM_ReadytoCharge) > 0) ? (AMS_OK = 0) : (AMS_OK = 1);
    }
     
/* Task 3 : Shutdown , Dash Board interaction (Should be 1st priority ) ==================================================== */ 
    // For Active High Switch (Butterfly board)
    (AMS_OK) ? digitalWrite(AMS_OUT,HIGH) : digitalWrite(AMS_OUT,LOW);
    
    // // For Active Low switch (Yss Blackboard)
    // (AMS_OK) ? digitalWrite(AMS_OUT,LOW) : digitalWrite(AMS_OUT,HIGH);
    
    // (LOW_VOLT_WARN) ? digitalWrite(LVlight,HIGH) : digitalWrite(LVlight,LOW);
    // (OVER_TEMP_WARN) ? digitalWrite(TEMPlight,HIGH) : digitalWrite(TEMPlight,LOW);

    // Debug AMS state
    if(currentMillis - reference_time >= SYNC_TIME) {
      Serial.printf("AMS_OK: %d \n", AMS_OK);
      Serial.printf("AMS_VOLT: %2f \n", AMS_Package.ACCUM_VOLTAGE);
      Serial.printf("AMS_MAX: %2f \n", AMS_Package.ACCUM_MAXVOLTAGE);
      Serial.printf("AMS_MIN: %2f \n", AMS_Package.ACCUM_MINVOLTAGE);
      
      Serial.printf("OV_CRIT: %d \n", OVER_VOLT_CRIT); 
      Serial.printf("ACCUM_FULL: %d \n", ACCUM_FULL);
      Serial.printf("LV_CRIT: %d \n", LOW_VOLT_CRIT);
      Serial.printf("LV_WARN: %d \n", LOW_VOLT_WARN);
      Serial.printf("OV_TEMP_CRT: %d \n", OVER_TEMP_CRIT);
      Serial.printf("OV_TEMP_WARN: %d \n", OVER_TEMP_WARN);
      
      Serial.printf("OV_DIV_CRT: %d \n", ACCUM_OverDivCritical);
      Serial.printf("OV_DIV_WARN: %d \n", ACCUM_OverDivWarn);
      // Serial.printf("OBC_VOLT: %d \n",OBC_Package.OBCVolt);
      // Serial.printf("OBC_AMP: %d \n",OBC_Package.OBCAmp);
      // 2nd one is connected to dummy test kit
      // debugBMUmsg(0);
      // debugBMUFault(0);
      reference_time= millis();
    }

  } 

/* ==================================Main Local Functions==============================*/

void packAMSmsg(twai_message_t *BCUsent, uint16_t Sync_time, bool &is_charger_plugged) {

  BCUsent->identifier  = BCU_ADD; // BCU ID
  BCUsent->data_length_code = 8;
  // BMU synchronize time
  BCUsent->data[0] = Sync_time;
  // Notify Charge
  (is_charger_plugged) ? (BCUsent->data[1] = 1) : (BCUsent->data[1] = 0);
  // Distribute Default parameter
  BCUsent->data[2] = VmaxCell; 
  BCUsent->data[3] = VminCell; 
  BCUsent->data[4] = TempMaxCell; 
  BCUsent->data[5] = dVmax;
  BCUsent->data[6] = BMUUpdateEEPROM; 
  BCUsent->data[7] = 0x00; // Reserved
}

void packBCU_OBCmsg ( twai_message_t *BCUsent, bool &AMS_OK , bool &ReadytoCharge, bool &OverDivCritical_Yes, bool &Voltage_is_Full) {

  /* Set up BMS CAN frame*/
  BCUsent->identifier  = OBC_ADD; // refers to specification sheet
  BCUsent->data_length_code = 8;
  // Reserved
  BCUsent->data[5] = 0x00;
  BCUsent->data[6] = 0x00;
  BCUsent->data[7] = 0x00;
  // Condition 1 iF BMS_OK AND ACCUM is Ready Any Module not Critically OVERDIV, ACCUMULATOR VOLTAGE ISN"T FULL YET
  if((AMS_OK && ReadytoCharge) || !OverDivCritical_Yes || !Voltage_is_Full) {
    BCUsent->data[0] = 0x18; // V highbyte 
    BCUsent->data[1] = 0x00; // V lowbyte 240.0 V (( For 6 Module ))
    BCUsent->data[2] = 0x00; // A Highbyte
    BCUsent->data[3] = 0x64; // A Lowbyte 10.0 A
    BCUsent->data[4] = 0x00; // Control Byte 0 charger operate
  } else {
    // Condition 0 Shutdown message
    BCUsent->data[0] = 0x00; 
    BCUsent->data[1] = 0x00; 
    BCUsent->data[2] = 0x00; 
    BCUsent->data[3] = 0x00;
    BCUsent->data[4] = 0x01; // Control Byte 1 charger shutdown
  } 
}

void processBMUmsg ( twai_message_t* receivedframe , BMUdata *BMU_Package) {
  // Reset BMU struct Value
  dynamicModulereset(BMU_Package);

  // decodeCANID according to BP16 agreement
  StandardCANIDDecoded decodedCANID;
  decodeStandardCANID(&decodedCANID, (receivedframe->identifier) );
  
  // Distingush Module ID
    int i = decodedCANID.SRC - 1;
    if(i >= BMU_NUM) return;      
  
  // Mark timestamp of successfully received Module, No update for disconnected BMU.
  lastModuleResponse[i] = millis();

  /* ---------------- unpack ReceiveFrame to BMUframe ------------------- */
  /*  Message Priority 02 :: BMUModule & Cells data  */
  if(decodedCANID.PRIORITY == 0x02)
  {
    switch (decodedCANID.MSG_NUM) { 
      // MSG1 == Operation status
      case 1:
        // Charging Ready
        BMU_Package[i].BMUreadytoCharge = receivedframe->data[0];
        // Balancing Discharge cell number
        BMU_Package[i].BalancingDischarge_Cells = mergeHLbyte(receivedframe->data[1],receivedframe->data[2]);
        // Vbatt (Module) , dVmax(cell)
        BMU_Package[i].V_MODULE = mergeHLbyte(receivedframe->data[3], receivedframe->data[4]); 

        BMU_Package[i].DV = receivedframe->data[5]; 
        // Temperature sensor
        BMU_Package[i].TEMP_SENSE[0] = receivedframe->data[6];
        BMU_Package[i].TEMP_SENSE[1] = receivedframe->data[7];
        break;

      case 2:
        // Low series Side Cell C1-C8
        for(short j=0; j< 8; j++)
          BMU_Package[i].V_CELL[j] = receivedframe->data[j];
        break;

      case 3:
        // High series side Cell C8-CellNumber
        for( short j = 8; j < CELL_NUM; j++ )
          BMU_Package[i].V_CELL[j] = receivedframe->data[(j-8)];
        break;
    }
  }
  /*  Message Priority 01 :: FaultCode  */
  else if(decodedCANID.PRIORITY == 0x01)
  {
    switch (decodedCANID.MSG_NUM) {
      case 1:
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMU_Package[i].OVERVOLTAGE_WARNING =  mergeHLbyte( receivedframe->data[0], receivedframe->data[1] );  
        BMU_Package[i].OVERVOLTAGE_CRITICAL = mergeHLbyte( receivedframe->data[2], receivedframe->data[3] );  
        BMU_Package[i].LOWVOLTAGE_WARNING = mergeHLbyte( receivedframe->data[4], receivedframe->data[5] );
        BMU_Package[i].LOWVOLTAGE_CRITICAL = mergeHLbyte( receivedframe->data[6], receivedframe->data[7] ); 
        break;
      case 2:
        // Merge H and L byte of each FaultCode back to 10 bit binary
        BMU_Package[i].OVERTEMP_WARNING = mergeHLbyte( receivedframe->data[0], receivedframe->data[1] );  
        BMU_Package[i].OVERTEMP_CRITICAL = mergeHLbyte( receivedframe->data[2], receivedframe->data[3] ); 
        BMU_Package[i].OVERDIV_VOLTAGE_WARNING = mergeHLbyte( receivedframe->data[4], receivedframe->data[5] ); 
        BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL = mergeHLbyte( receivedframe->data[6], receivedframe->data[7] );
        break;
    }
  }
}

void processOBCmsg ( twai_message_t *J1939frame ) {
  // if message ID isnt 0x18FF50E5 , return
  if(J1939frame->identifier != 0x18FF50E5)
    return;
  
    // Monitor & Translate current Frame data
    uint8_t VoutH = J1939frame->data[0];
    uint8_t VoutL = J1939frame->data[1];
    uint8_t AoutH = J1939frame->data[2];
    uint8_t AoutL = J1939frame->data[3];
    OBC_Package.OBCstatusbit =  J1939frame->data[4]; // Status Byte
    OBC_Package.OBCVolt = mergeHLbyte(VoutH,VoutL);
    OBC_Package.OBCAmp = mergeHLbyte(AoutH,AoutL);

}

/* ==================================Sub Functions==============================*/

bool isModuleActive(int moduleIndex) {
  unsigned int MAX_SILENCE = DISCONNENCTION_TIMEOUT;
  return (millis() - lastModuleResponse[moduleIndex]) <= (MAX_SILENCE);
}
void resetModuleData(int moduleIndex){
  BMU_Package[moduleIndex] = BMUdata(); // Reset to default-constructed object
}
void packing_AMSstruct (int moduleIndex) {
  int &i = moduleIndex;
  // Recalculate AMS based on current BMU states
  AMS_Package.ACCUM_VOLTAGE += ( static_cast<float>(BMU_Package[i].V_MODULE)) * 0.2;
  AMS_Package.OVERTEMP_WARNING |= BMU_Package[i].OVERTEMP_WARNING;
  AMS_Package.OVERTEMP_CRITICAL |= BMU_Package[i].OVERTEMP_CRITICAL;
  AMS_Package.OVERDIV_WARNING |= BMU_Package[i].OVERDIV_VOLTAGE_WARNING;
  AMS_Package.OVERDIV_CRITICAL |= BMU_Package[i].OVERDIV_VOLTAGE_CRITICAL;
  
  AMS_Package.ACCUM_CHG_READY &= (BMU_Package[i].BMUreadytoCharge);
  
  // AMS_Package.OVERVOLT_CRITICAL |= BMU_Package[i].OVERVOLTAGE_CRITICAL;
  // AMS_Package.LOWVOLT_CRITICAL |= BMU_Package[i].LOWVOLTAGE_CRITICAL;
  // AMS_Package.OVERVOLT_WARNING |= BMU_Package[i].OVERVOLTAGE_WARNING;
  // AMS_Package.LOWVOLT_WARNING |= BMU_Package[i].LOWVOLTAGE_WARNING;
  // Available Module , and 
}
void resetAllStruct(){
  for (int i = 0; i < BMU_NUM; i++){
    resetModuleData(i);
    lastModuleResponse[i] = 0;
  }
  AMS_Package = AMSdata();
  OBC_Package = OBCdata();
}
bool checkModuleDisconnect(BMUdata *BMU_Package){

  bool disconnectedFromCAN = 1; 
  for(short i =0; i< BMU_NUM ; i++){
    if(BMU_Package[i].BMUconnected == 0)
      disconnectedFromCAN = 0; 
  }
    return disconnectedFromCAN;
}
void dynamicModulereset(BMUdata *BMU_Package){ 
  for(short i =0; i< BMU_NUM ; i++){
    // if any of the board aren't in connection => throw error
    if(BMU_Package[i].BMUconnected == 0){
      resetModuleData(i); // Reset that module data (revert voltage , temp., flags , etc. Back to zero)
    }   
  }
}

/* ==================================Serial Debugger==============================*/

void debugBMUmsg(int Module){

    // Serial.print("BMU_CHGready: "); Serial.println(BMU_Package[Module].BMUreadytoCharge);
    // Serial.print("V_Disch: "); Serial.println(BMU_Package[Module].BalancingDischarge_Cells,BIN);
    Serial.print("V_CELL[10]: ");
    // can change to vector , for easy looping funcion
    for(short i=0; i< CELL_NUM; i++){
      Serial.print(BMU_Package[Module].V_CELL[i] * 0.02); Serial.print("V.  ");
    } Serial.println();
    
    // Serial.print("V_MODULE: "); Serial.print(BMU_Package[Module].V_MODULE *0.2); Serial.println("V.  ");
    // Serial.print("DV: ") ; Serial.print(BMU_Package[Module].DV * 0.1); Serial.println("V.  ");

    Serial.print("TEMP[2]: ");
      Serial.print(BMU_Package[Module].TEMP_SENSE[0]*0.6 + 2); Serial.println("C.  ");
      Serial.print(BMU_Package[Module].TEMP_SENSE[1]*0.6 + 2); Serial.println("C.  ");

    Serial.println();

}
void debugBMUFault(int Module){
  
    Serial.print("OV_CRI: ");
    Serial.println(BMU_Package[Module].OVERVOLTAGE_CRITICAL,HEX);
    Serial.print("LV_CRI: ");
    Serial.println(BMU_Package[Module].LOWVOLTAGE_CRITICAL,HEX);
    Serial.print("OVT_CRI: ");
    Serial.println(BMU_Package[Module].OVERTEMP_CRITICAL, HEX);
    Serial.print("OVDIV_CRI: ");
    Serial.println(BMU_Package[Module].OVERDIV_VOLTAGE_CRITICAL,HEX);
    Serial.println();
  
    Serial.print("OV_W: ");
    Serial.println(BMU_Package[Module].OVERVOLTAGE_WARNING,HEX);
    Serial.print("LV_W: ");
    Serial.println(BMU_Package[Module].LOWVOLTAGE_WARNING,HEX);
    Serial.print("OVT_W: ");
    Serial.println(BMU_Package[Module].OVERTEMP_WARNING, HEX);
    Serial.print("OVDIV_CRI: ");
    Serial.println(BMU_Package[Module].OVERDIV_VOLTAGE_WARNING,HEX);
    Serial.println();  
    
}
void debugOBCmsg(){ 
    Serial.print("Voltage from OBC: "); Serial.print(OBC_Package.OBCVolt); Serial.println("V");
    Serial.print("Current from OBC: "); Serial.print(OBC_Package.OBCAmp); Serial.println("A");
    Serial.print("OBC status bit"); Serial.println(OBC_Package.OBCstatusbit);

    // Intepret Individual bit meaning
    bool *obcstatbitarray =  toBitarrayLSB(OBC_Package.OBCstatusbit); // Status Byte
    
    Serial.print("OBC status bit: ");
    switch (obcstatbitarray[0]) {
      case 1:
        Serial.println("ChargerHW = Faulty");
        break;
    }
    switch (obcstatbitarray[1]) {
      case 1:
        Serial.println("ChargerTemp = Overheat");
        break;
    }
    switch (obcstatbitarray[2]) {
      case 1:
        Serial.println("ChargerACplug = Reversed");
        break;
    }
    switch (obcstatbitarray[3]) {
      case 1:
        Serial.println("Charger detects: NO BATTERY VOLTAGES");
        break;
    }
    switch (obcstatbitarray[4]) {
      case 1:
        Serial.println("OBC Detect COMMUNICATION Time out: (6s)");
        break;
    }
}
