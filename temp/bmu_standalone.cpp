#include <Arduino.h>
#include <SPI.h>
#include <driver/twai.h>
#include "LTC6811.h"
#include "LTC681x.h"
#include "Linduino.h"
#include "CAN32_util.h"
#include "ams_common_conf.h"
#include "helper.h"
#include "ntstermistor.h"

// Example to write for filter -> Make it an output of twai_filter_config_t type , to get the data strcuture need

// =========================================================================
// HARDWARE GLUE CODE (SPI IMPLEMENTATIONS)
// =========================================================================
// void cs_low(uint8_t pin) { digitalWrite(CS_PIN, LOW); }
// void cs_high(uint8_t pin) { digitalWrite(CS_PIN, HIGH); }
// void delay_u(uint16_t micro) { delayMicroseconds(micro); }
// void delay_m(uint16_t milli) { delay(milli); }
// void spi_write_array(uint8_t len, uint8_t data[]) {
//   for (int i = 0; i < len; i++) SPI.transfer(data[i]);
// }
// void spi_write_read(uint8_t tx_Data[], uint8_t tx_len, uint8_t *rx_data, uint8_t rx_len) {
//   for (int i = 0; i < tx_len; i++) SPI.transfer(tx_Data[i]);
//   for (int i = 0; i < rx_len; i++) rx_data[i] = SPI.transfer(0xFF);
// }

/**************** Dummy Data (Replace with actual sensor/calculated data) *******************/
// BMU MSG 1 data
bool BMU_ReadyToCharge = false;              // Byte 0: Module ready to charge
uint16_t BalancingDischarge_Cells = 0x0000;  // Byte 1-2: 10-bit cell balancing status
uint8_t DV_raw = 0;                          // Byte 3: Delta V (factor 0.1V)
// uint8_t TempSensor1_raw = 0;               // Byte 4: Temp sensor 1 (offset 2, factor 0.0125)
// uint8_t TempSensor2_raw = 0;// Byte 5: Temp sensor 2

// BMU MSG 4 & 5 Fault flags (10-bit per fault type, each bit = 1 cell)
uint16_t OVERVOLTAGE_WARNING = 0x0000;       // Bytes 0-1
uint16_t OVERVOLTAGE_CRITICAL = 0x0000;      // Bytes 2-3
uint16_t LOWVOLTAGE_WARNING = 0x0000;        // Bytes 4-5
uint16_t LOWVOLTAGE_CRITICAL = 0x0000;       // Bytes 6-7
uint16_t OVERTEMP_WARNING = 0x0000;          // Bytes 0-1 (MSG5)
uint16_t OVERTEMP_CRITICAL = 0x0000;         // Bytes 2-3 (MSG5)
uint16_t OVERDIV_WARNING = 0x0000;           // Bytes 4-5 (MSG5)
uint16_t OVERDIV_CRITICAL = 0x0000;          // Bytes 6-7 (MSG5)





//CAN section
twai_message_t tx_message;
twai_message_t rx_message;

//bms section
#define TOTAL_IC    1
#define NUM_CELLS   10
#define CS_PIN      7
#define TEMP_SENSOR1_PIN A0
#define TEMP_SENSOR2_PIN A1
cell_asic bms_ic[TOTAL_IC];
float cellvoltages[NUM_CELLS];

#define CAN_TX_PIN 20
#define CAN_RX_PIN 21

bool canbusready = false;

// Default BMU parameters from ams_common_conf.h (Can be update at runtime) 
int transimission_time = BMS_COMMUNICATE_TIME;
float VmaxCell = VMAX_CELL;
float VminCell = VMIN_CELL;
float TempMaxCell = TEMP_MAX_CELL;
float dVmax = DVMAX;

/**************** Timing Variables *******************/
unsigned long prevMillis = 0;
unsigned long prevFaultMillis = 0;
unsigned long intervalMillis = 1000;       // Cell data transmission interval
unsigned long faultIntervalMillis = 1300;  // Fault code transmission interval
unsigned long lastCANHealthCheck = 0;
unsigned long canHealthCheckInterval = 5000; // Check CAN health every 5 seconds

/**************** Local Function Declaration *******************/
void readAllCells();
void balanceCells();
void packBMU_MSG1_OperationStatus(twai_message_t* msg, uint32_t id,uint8_t temp1,uint8_t temp2);
void packBMU_MSG2_CellsLowSeries(twai_message_t* msg, uint32_t id);
void packBMU_MSG3_CellsHighSeries(twai_message_t* msg, uint32_t id);
void packBMU_MSG4_FaultCode1(twai_message_t* msg, uint32_t id);
void packBMU_MSG5_FaultCode2(twai_message_t* msg, uint32_t id);
void processBCUConfigMsg(twai_message_t* msg);
void debugConfig();

float getTemp(int pin,int print);
void balanceCells();
bool reinitCAN();
void checkCANHealth();

uint8_t encode_temp(float temp_c);

/*******************************************************************
  ==============================Setup==============================
********************************************************************/
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { } // Wait up to 3 seconds for Serial
  delay(1000);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  SPI.begin(4, 5, 6, 7);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  Serial.println("✓ SPI Initialized");

  LTC6811_init_cfg(TOTAL_IC, bms_ic);
  LTC6811_reset_crc_count(TOTAL_IC, bms_ic);
  LTC6811_init_reg_limits(TOTAL_IC, bms_ic);

  // Initialize CAN using CAN32_util library
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  canbusready = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, t_config);
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/
void loop() {
    // TODO: Set from EEPROM or DIP switch
  uint32_t SESSION_TIME = millis();
  uint8_t temp1 = getTemp(TEMP_SENSOR1_PIN,1);
  uint8_t temp2 = getTemp(TEMP_SENSOR2_PIN,0);
; // Example temperature reading
  // Periodic CAN health check and recovery
  if (SESSION_TIME - lastCANHealthCheck >= canHealthCheckInterval) {
    checkCANHealth();
    lastCANHealthCheck = SESSION_TIME;
  }

  // Process incoming BCU messages (BCU config updates)
    if (CAN32_receiveCAN(&rx_message,canbusready) == ESP_OK)
      processBCUConfigMsg(&rx_message);
  
  readAllCells();
  if(BMU_ReadyToCharge) balanceCells(); 

  int ModuleNum = 5;
  // ======== Cell Data Transmission (1000ms interval) ========
  if(SESSION_TIME - prevMillis >= intervalMillis){
    // Serial.print("TEMP : ");
    // Serial.println(temp);
    // for (int i = 0; i < NUM_CELLS; i++) {
    //   Serial.print("Cell "); Serial.print(i + 1); 
    //   Serial.print(": "); Serial.print(cellvoltages[i], 4);
    //   Serial.println(" V");
    // }

    // BMU MSG 1: Operation Status (Priority=2, Msg=1)
    uint32_t bmu_id_msg1 = createExtendedCANID(2, ModuleNum, 1);
    packBMU_MSG1_OperationStatus(&tx_message, bmu_id_msg1,temp1,temp2);
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

    prevMillis = millis();
  }
  
  // TODO: Set from EEPROM or DIP switch
  // ======== Fault Code Transmission (1300ms interval) ========
  if(SESSION_TIME - prevFaultMillis >= faultIntervalMillis){
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

    prevFaultMillis = millis();
  }
}

// ============================================================================
// LTC6811 function
// ============================================================================
float getTemp(int pin, int print) {
  
  while (print == 1){
    Serial.print("tempC pin ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.println(resistance_to_celsius(read_ntc_resistance(pin)));
    print = 0;
  }
  
  return resistance_to_celsius(read_ntc_resistance(pin));
}


void readAllCells(){
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);  // Start conversion
  delay(10);  // Wait for conversion to complete
  int8_t error = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);
  for (int i = 0; i < NUM_CELLS; i++) {
    cellvoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001;
  }
}

void balanceCells(){
  return; // utilize basic parameter
}

// Reinitialize CAN bus after power outage or bus error
bool reinitCAN() {
  Serial.println("--- CAN Bus Reinitialization ---");

  // Stop and uninstall existing driver
  twai_stop();
  twai_driver_uninstall();
  delay(100);

  // Reinitialize CAN
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  canbusready = CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, t_config);

  if (canbusready) {
    Serial.println("CAN bus reinitialized successfully");
  } else {
    Serial.println("CAN bus reinitialization FAILED");
  }

  return canbusready;
}

// Check CAN health and reinitialize if needed
void checkCANHealth() {
  twai_status_info_t status_info;
  twai_get_status_info(&status_info);

  // Check for bus-off or stopped state (indicates power loss or severe error)
  if (status_info.state == TWAI_STATE_BUS_OFF ||
      status_info.state == TWAI_STATE_STOPPED) {
    Serial.println("[CAN] Bus error detected - attempting recovery...");
    reinitCAN();
  }
  // Check for high error counts (indicates communication issues)
  else if (status_info.tx_error_counter > 127 || status_info.rx_error_counter > 127) {
    Serial.printf("[CAN] High error count (TX:%d, RX:%d) - reinitializing...\n",
                  status_info.tx_error_counter, status_info.rx_error_counter);
    reinitCAN();
  }
}

// ============================================================================
// Message Packing
// ============================================================================

// Not sure how to encode temperature into 1 byte with given offset and factor
// since right now temp is float which requires 2 bytes to represent accurately. (1 decimal place)
// Thus I can use helper.h function to split high byte and low byte.
// Then change protocol to use 2 bytes for temperature representation.
uint8_t encode_temp(float temp_c) {
    // Clamp to valid range first
    if (temp_c < -40.0f) temp_c = -40.0f;
    if (temp_c > 87.5f) temp_c = 87.5f;
    
    // Apply formula
    float scaled = (temp_c + 40.0f) * 2.0f;
    
    // Round and convert to byte
    return (uint8_t)(scaled + 0.5f);  // +0.5 for rounding
}
// BMU MSG 1: Module Operation & Voltage Status (Priority=2, Msg=1)
// Byte 0: BMU Enter Charging Mode (0=NO, 1=YES)
// Byte 1-2: Cells in Balancing (10-bit, MSB first)
// Byte 3: Averaged Cell Voltage Difference DV (factor 0.1V)
// Byte 4: Temp Sensor 1 (offset 2, factor 0.0125)
// Byte 5: Temp Sensor 2 (offset 2, factor 0.0125)
// Byte 6-7: Reserved
void packBMU_MSG1_OperationStatus(twai_message_t* msg, uint32_t id, uint8_t temp1, uint8_t temp2) {
  msg->identifier = id;
  msg->extd = 1;  // Extended CAN ID
  msg->rtr = 0;
  msg->data_length_code = 8;

  msg->data[0] = BMU_ReadyToCharge ? 1 : 0;
  msg->data[1] = (BalancingDischarge_Cells >> 8) & 0xFF;  // High byte
  msg->data[2] = BalancingDischarge_Cells & 0xFF;         // Low byte
  msg->data[3] = DV_raw;
  msg->data[4] = encode_temp(temp1);
  msg->data[5] = encode_temp(temp2);
  msg->data[6] = 0x00;  // Reserved
  msg->data[7] = 0x00;  // Reserved
}

// BMU MSG 2: Cell Monitoring LOW SERIES (Priority=2, Msg=2)
// Byte 0-7: Cell 1-8 voltages (factor 0.02V)
void packBMU_MSG2_CellsLowSeries(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;  // Extended CAN ID
  msg->rtr = 0;
  msg->data_length_code = 8;

  for (int i = 0; i < 8; i++) {
    msg->data[i] = (uint8_t)(cellvoltages[i] / 0.02);
  }
}

// BMU MSG 3: Cell Monitoring HIGH SERIES (Priority=2, Msg=3)
// Byte 0-1: Cell 9-10 voltages (factor 0.02V)
// Byte 2-7: Reserved
void packBMU_MSG3_CellsHighSeries(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;  // Extended CAN ID
  msg->rtr = 0;
  msg->data_length_code = 8;

  msg->data[0] = (uint8_t)(cellvoltages[8] / 0.02);
  msg->data[1] = (uint8_t)(cellvoltages[9] / 0.02);
  msg->data[2] = 0x00;  // Reserved
  msg->data[3] = 0x00;  // Reserved
  msg->data[4] = 0x00;  // Reserved
  msg->data[5] = 0x00;  // Reserved
  msg->data[6] = 0x00;  // Reserved
  msg->data[7] = 0x00;  // Reserved
}

// BMU MSG 4: Module Fault Code 1 (Priority=1, Msg=1)
// Byte 0-1: Over Voltage Warning (10-bit, MSB first)
// Byte 2-3: Over Voltage Critical (10-bit, MSB first)
// Byte 4-5: Low Voltage Warning (10-bit, MSB first)
// Byte 6-7: Low Voltage Critical (10-bit, MSB first)
void packBMU_MSG4_FaultCode1(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;  // Extended CAN ID
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

// BMU MSG 5: Module Fault Code 2 (Priority=1, Msg=2)
// Byte 0-1: Over Temp Warning (10-bit, MSB first)
// Byte 2-3: Over Temp Critical (10-bit, MSB first)
// Byte 4-5: Over Div Voltage Warning (10-bit, MSB first)
// Byte 6-7: Over Div Voltage Critical (10-bit, MSB first)
void packBMU_MSG5_FaultCode2(twai_message_t* msg, uint32_t id) {
  msg->identifier = id;
  msg->extd = 1;  // Extended CAN ID
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

// Extract runtime config from BCU
void processBCUConfigMsg(twai_message_t* msg) {
  if (msg->identifier != BCU_ADD) return;

  // proceed to update if update flag is true
  bool BMUUpdateFlag = msg->data[6];
  if(!BMUUpdateFlag) return;

  transimission_time = msg->data[0];
  BMU_ReadyToCharge = msg->data[1];
  VmaxCell     = msg->data[2]*0.1;
  VminCell     = msg->data[3]*0.1;
  TempMaxCell  = msg->data[4];
  dVmax        = msg->data[5]*0.1;
}

// Debug: print current config
void debugConfig() {
  Serial.println("=== BMU Runtime Config ===");
  Serial.printf("SyncTime: %dms\n", transimission_time);
  Serial.printf("ChargerPlugged: %s\n", BMU_ReadyToCharge ? "YES" : "NO");
  Serial.printf("VmaxCell: %.1fV\n", VmaxCell);
  Serial.printf("VminCell: %.1fV\n", VminCell);
  Serial.printf("TempMax: %d°C\n", TempMaxCell);
  Serial.printf("DVmax: %.1fV\n", dVmax);
}
