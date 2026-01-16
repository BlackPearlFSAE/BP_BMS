#include <Arduino.h>
#include <SPI.h>
#include "driver/twai.h"
#include "LTC6811.h"
#include "LTC681x.h"
#include "Linduino.h"
#include "CAN32_util.h"

//CAN section
twai_message_t tx_message;
twai_message_t rx_message;

//bms section
#define TOTAL_IC    1
#define NUM_CELLS   10
#define CS_PIN      7
cell_asic bms_ic[TOTAL_IC];
float cellvoltages[NUM_CELLS];

#define CAN_TX_PIN 20
#define CAN_RX_PIN 21

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

bool canInitialized = false;

/**************** Local Function Declaration *******************/
void packCANData(uint8_t* data, float* voltages, uint8_t startCell, uint8_t numCells);
void readAllCells();
void sendCANFrame(uint32_t id, uint8_t* data, uint8_t len);

/**************** Timing Variables *******************/
unsigned long prevMillis = 0;
unsigned long intervalMillis = 1000;
unsigned long lastCANHealthCheck = 0;
unsigned long canHealthCheckInterval = 5000; // Check CAN health every 5 seconds

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
  CAN32_initCANBus(CAN_TX_PIN, CAN_RX_PIN, canInitialized, t_config);
}

/*******************************************************************
  =======================Mainloop===========================
********************************************************************/
void loop() {
  // Periodic CAN health check using CAN32_util debug
  if (millis() - lastCANHealthCheck >= canHealthCheckInterval) {
    lastCANHealthCheck = millis();
    uint32_t alerts = TWAI_ALERT_ALL;
    CAN32_twai_debug(alerts);
  }

  // Read incoming CAN messages continuously using CAN32_util
  if (canInitialized) {
    while (CAN32_receiveCAN(&rx_message) == ESP_OK) {
      CAN32_debugFrame(&rx_message);
    }
  }

  readAllCells();

  if(millis() - prevMillis >= intervalMillis){
    prevMillis = millis();

    // Pack and send cells 0-7 (8 bytes @ 0.02V resolution)
    uint8_t cellData1[8];
    packCANData(cellData1, cellvoltages, 0, 8);
    sendCANFrame(0x03, cellData1, 8);

    // Pack and send cells 8-9 (2 bytes @ 0.02V resolution)
    uint8_t cellData2[2];
    packCANData(cellData2, cellvoltages, 8, 2);
    sendCANFrame(0x04, cellData2, 2);
  }
}

/*******************************************************************
  =======================Local Functions===========================
********************************************************************/

void packCANData(uint8_t* data, float* voltages, uint8_t startCell, uint8_t numCells) {
  for (int i = 0; i < numCells && i < 8; i++) {
    data[i] = (uint8_t)(voltages[startCell + i]/0.02);
  }
}

void sendCANFrame(uint32_t id, uint8_t* data, uint8_t len) {
  if (!canInitialized) {
    Serial.println("CAN not initialized, skipping transmission");
    return;
  }

  if (len > 8) len = 8;

  tx_message.identifier = id;
  tx_message.extd = 0;
  tx_message.rtr = 0;
  tx_message.data_length_code = len;

  for (int i = 0; i < len; i++) {
    tx_message.data[i] = data[i];
  }

  int result = CAN32_sendCAN(&tx_message);

  if (result == ESP_OK) {
    Serial.print("CAN TX: 0x");
    Serial.print(id, HEX);
    Serial.print(" [");
    for (int i = 0; i < len; i++) {
      if (data[i] < 0x10) Serial.print("0");
      Serial.print(data[i], HEX);
      if (i < len - 1) Serial.print(" ");
    }
    Serial.println("]");
  } else if (result == ESP_ERR_TIMEOUT) {
    Serial.println("CAN TX TIMEOUT - bus busy");
  } else {
    Serial.print("CAN TX FAILED with error: ");
    Serial.println(result);
  }
}

void readAllCells(){
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);  // Start conversion
  delay(10);  // Wait for conversion to complete
  int8_t error = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);
  for (int i = 0; i < NUM_CELLS; i++) {
    cellvoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001;
    Serial.print("Cell ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(cellvoltages[i], 4);
    Serial.println(" V");
  }
}
