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

// Pack BMU cell voltages into CAN message
void packBMUCells(twai_message_t* msg, uint32_t id, float* voltages,
                        uint8_t startCell, uint8_t numCells, float resolution);

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

bool canInitialized = false;

/**************** Local Function Declaration *******************/
void readAllCells();

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
  // if (millis() - lastCANHealthCheck >= canHealthCheckInterval) {
  //   lastCANHealthCheck = millis();
  //   uint32_t alerts = TWAI_ALERT_ALL;
  //   CAN32_twai_debug(alerts);
  // }
  
  // // Read incoming CAN messages continuously using CAN32_util
  // if (canInitialized) {
  //   while (CAN32_receiveCAN(&rx_message) == ESP_OK) {
  //     CAN32_debugFrame(&rx_message);
  //   }
  // }

  readAllCells();

  if(millis() - prevMillis >= intervalMillis){
    prevMillis = millis();

    // Pack and send cells 0-7 (8 bytes @ 0.02V resolution)
    packBMUCells(&tx_message, 0x06, cellvoltages, 0, 8, 0.02);
    if (CAN32_sendCAN(&tx_message) != ESP_OK) {
      Serial.println("CAN TX failed for cells 0-7");
    }

    // Pack and send cells 8-9 (2 bytes @ 0.02V resolution)
    packBMUCells(&tx_message, 0x07, cellvoltages, 8, 2, 0.02);
    if (CAN32_sendCAN(&tx_message) != ESP_OK  ) {
      Serial.println("CAN TX failed for cells 8-9");
    }
  }
}

/*******************************************************************
  =======================Local Functions===========================
********************************************************************/

void readAllCells(){
  wakeup_sleep(TOTAL_IC);
  LTC6811_adcv(MD_7KHZ_3KHZ, DCP_ENABLED, CELL_CH_ALL);  // Start conversion
  delay(10);  // Wait for conversion to complete
  int8_t error = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic);
  for (int i = 0; i < NUM_CELLS; i++) {
    cellvoltages[i] = bms_ic[0].cells.c_codes[i] * 0.0001;
    // Serial.print("Cell ");
    // Serial.print(i + 1);
    // Serial.print(": ");
    // Serial.print(cellvoltages[i], 4);
    // Serial.println(" V");
  }
}

// Pack BMU cell voltages into CAN message
void packBMUCells(twai_message_t* msg, uint32_t id, float* voltages,
                        uint8_t startCell, uint8_t numCells, float resolution) {
  msg->identifier = id;
  msg->extd = 0;
  msg->rtr = 0;
  msg->data_length_code = (numCells > 8) ? 8 : numCells;

  for (int i = 0; i < msg->data_length_code; i++) {
    msg->data[i] = (uint8_t)(voltages[startCell + i] / resolution);
  }
}// Pack BMU cell voltages into CAN message



// Example to write for filter -> Make it an output of twai_filter_config_t type , to get the data strcuture need