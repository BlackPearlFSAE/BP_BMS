#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <RTClib.h>

// FreeRTOS for multi-core task management
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// Helper function
#include "BP_mobile_util.h"
#include "SD32_util.h"
#include "WIFI32_util.h"
#include "syncTime_util.h"
#include "RTClib_helper.h"
#include "ams_common_conf.h"

// ============================================================================
// BP MOBILE SERVER CONFIGURATION
// ============================================================================
const char* ssid = "realme C55";
const char* password = "realme1234";
const char* serverHost = "10.18.211.132";
const int serverPort = 3000;
const char* clientName = "ESP32 BMS Logger";

WebSocketsClient webSockets;
socketstatus webSocketStatus;
BPMobileConfig BPMobile(&webSockets, &webSocketStatus);
WebSocketsClient* BPwebSocket = BPMobile.webSocket;
socketstatus* BPsocketstatus = BPMobile.webSocketstatus;

// ============================================================================
// DEVICE BASE CONFIGURATION
// ============================================================================
// ---- LEDs
#define WIFI_LED 3
#define WS_LED 4

// ---- I2C
#define I2C1_SDA 18
#define I2C1_SCL 17
bool I2C1_connect = false;

// ---- SD SPI
#define SD_CS_PIN 10
#define SD_MOSI_PIN 11
#define SD_MISO_PIN 13
#define SD_SCK_PIN 12

// ---- UART
#define UART0_BAUD 115200

// ---- Timing configs
RTC_DS3231 rtc;
bool RTCavailable = false;
unsigned long lastTimeSourceSync = 0;
unsigned long lastExternalSync = 0;
uint64_t RTC_UNIX_TIME = 0;
const unsigned long LOCAL_SYNC_INTERVAL = 1000;
const unsigned long REMOTE_SYNC_INTERVAL = 60000;

// SD Card Timing Configuration
bool sdCardReady = false;
unsigned long lastSDLog = 0;
int dataPoint = 1;
int sessionNumber = 0;

const unsigned long SD_APPEND_INTERVAL = 200;
const unsigned long AMS_LOG_INTERVAL = 1000;    // AMS summary every 1 second
const unsigned long SD_FLUSH_INTERVAL = 1000;
const unsigned long SD_CLOSE_INTERVAL = 10000;

// Session directory and file paths
char sessionDirPath[32] = {0};
char bmuFilePaths[MODULE_NUM][48] = {0};
char amsFilePath[48] = {0};

// CSV Headers
const char* header_BMU = "DataPoint,UnixTime,SessionTime,V_MODULE,TEMP1,TEMP2,DV,"
                         "Cell1,Cell2,Cell3,Cell4,Cell5,Cell6,Cell7,Cell8,Cell9,Cell10,"
                         "OV_WARN,OV_CRIT,LV_WARN,LV_CRIT,OT_WARN,OT_CRIT,ODV_WARN,ODV_CRIT,"
                         "BAL_CELLS,Connected,ReadyToCharge";

const char* header_AMS = "DataPoint,UnixTime,SessionTime,ACCUM_VOLTAGE,ACCUM_MAXV,ACCUM_MINV,"
                         "AMS_OK,CHG_READY,OV_WARN,OV_CRIT,LV_WARN,LV_CRIT,OT_WARN,OT_CRIT,ODV_WARN,ODV_CRIT";

// ---- Global Data
BMUdata BMU_Package[MODULE_NUM];
AMSdata AMS_Package;

// ============================================================================
// BPMOBILE PUBLISHING CONFIG
// ============================================================================
const float BMU_CELLS_SAMPLING_RATE = 2.0;    // Hz - publish cell voltages
const float BMU_FAULT_SAMPLING_RATE = 1.0;    // Hz - publish fault flags
const float AMS_DATA_SAMPLING_RATE = 0.5;     // Hz - publish AMS summary

// Forward declarations for publishing
void publishBMUcells(BMUdata* bmu, int moduleNum);
void publishBMUfaults(BMUdata* bmu, int moduleNum);
void publishAMSstate(AMSdata* ams);
void registerClient(const char* clientName);

// ============================================================================
// FREERTOS
// ============================================================================
SemaphoreHandle_t dataMutex = NULL;
SemaphoreHandle_t serialMutex = NULL;  // For cross-core Serial prints
TaskHandle_t BPMobileTaskHandle = NULL;
TaskHandle_t sdTaskHandle = NULL;
QueueHandle_t sdQueue = NULL;

// SD Queue data structure
struct SDLogEntry {
  int dataPoint;
  uint64_t unixTime;
  uint64_t sessionTime;
  BMUdata bmu[MODULE_NUM];
  AMSdata ams;
};

// File handles for persistent logging (one per BMU + one for AMS)
static File bmuFiles[MODULE_NUM];
static File amsFile;
static bool filesOpen = false;

// Forward declarations
void append_BMU_toCSVFile(File& dataFile, BMUdata* bmu, int dp, uint64_t Timestamp, uint64_t session);
void append_AMS_toCSVFile(File& dataFile, AMSdata* ams, int dp, uint64_t Timestamp, uint64_t session);
void openAllFiles();
void closeAllFiles();
void flushAllFiles();

// ============================================================================
// BPMOBILE TASK - Runs on Core 0
// ============================================================================
void BPMobileTask(void* parameter) {
  // Task-local timing variables
  unsigned long taskLastBMUcells = 0;
  unsigned long taskLastBMUfaults = 0;
  unsigned long taskLastAMS = 0;

  // Local copies of sensor data
  BMUdata localBMU[MODULE_NUM];
  AMSdata localAMS;

  while (true) {
    unsigned long now = millis();

    // Handle WebSocket communication
    if (WiFi.status() == WL_CONNECTED) {
      BPwebSocket->loop();
    }

    // Update LED status
    digitalWrite(WIFI_LED, WiFi.status() == WL_CONNECTED ? 1 : 0);
    digitalWrite(WS_LED, BPsocketstatus->isConnected ? 1 : 0);

    // Publish data if registered and connected
    if (BPsocketstatus->isRegistered && BPsocketstatus->isConnected) {
      // Quick copy: take mutex briefly, copy data, release immediately
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        memcpy(localBMU, BMU_Package, sizeof(BMU_Package));
        localAMS = AMS_Package;
        xSemaphoreGive(dataMutex);
      }

      // Publish BMU cell voltages
      if (now - taskLastBMUcells >= (unsigned long)(1000.0 / BMU_CELLS_SAMPLING_RATE)) {
        for (int i = 0; i < MODULE_NUM; i++) {
          publishBMUcells(&localBMU[i], i);
        }
        taskLastBMUcells = now;
      }

      // Publish BMU faults
      if (now - taskLastBMUfaults >= (unsigned long)(1000.0 / BMU_FAULT_SAMPLING_RATE)) {
        for (int i = 0; i < MODULE_NUM; i++) {
          publishBMUfaults(&localBMU[i], i);
        }
        taskLastBMUfaults = now;
      }

      // Publish AMS summary
      if (now - taskLastAMS >= (unsigned long)(1000.0 / AMS_DATA_SAMPLING_RATE)) {
        publishAMSstate(&localAMS);
        taskLastAMS = now;
      }
    }

    // Small delay to prevent task starvation
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================================================
// SD TASK - Runs on Core 1
// ============================================================================
void sdTask(void* parameter) {
  SDLogEntry entry;
  unsigned long lastFlushTime = 0;
  unsigned long lastAMSLog = 0;
  int localDataPoint = 0;

  while (true) {

    // Wait for data from queue
    if (xQueueReceive(sdQueue, &entry, portMAX_DELAY) == pdTRUE) {
      if (!sdCardReady || !filesOpen) {
        continue;
      }

      unsigned long now = millis();
      // Write to all BMU files (every entry = 200ms)
      for (int i = 0; i < MODULE_NUM; i++) {
        if (bmuFiles[i]) {
          append_BMU_toCSVFile(bmuFiles[i], &entry.bmu[i], entry.dataPoint, entry.unixTime, entry.sessionTime);
        }
      }
      // Write to AMS file (every ~1 second)
      if (now - lastAMSLog >= AMS_LOG_INTERVAL) {
        if (amsFile) {
          append_AMS_toCSVFile(amsFile, &entry.ams, entry.dataPoint, entry.unixTime, entry.sessionTime);
        }
        lastAMSLog = now;
      }
      // Flush periodically
      if (now - lastFlushTime >= SD_FLUSH_INTERVAL) {
        flushAllFiles();
        lastFlushTime = now;
      }

      localDataPoint++;
      if (localDataPoint % 50 == 0) {
        Serial.printf("[SD Task] Logged %d entries\n", localDataPoint);
      }
    }
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(UART0_BAUD);
  delay(100);

  Serial.println("==================================================");
  Serial.println("       BMS Data Logger - Initializing");
  Serial.println("==================================================");

  // LED pins
  pinMode(WIFI_LED, OUTPUT);
  pinMode(WS_LED, OUTPUT);
  digitalWrite(WIFI_LED, 0);
  digitalWrite(WS_LED, 0);

  // I2C for RTC
  I2C1_connect = Wire1.begin(I2C1_SDA, I2C1_SCL);
  Wire1.setTimeout(2);

  // RTC Init
  RTCavailable = RTCinit(rtc,&Wire1);

  // Connect to WiFi
  initWiFi(ssid, password, 10);

  // Initialize NTP after WiFi connects
  if (WiFi.status() == WL_CONNECTED) {
    WiFi32_initNTP();
    // Set up WebSocket
    BPMobile.setClientName(clientName);
    BPMobile.setRegisterCallback(registerClient);
    BPMobile.initWebSocket(serverHost, serverPort, clientName);
  }

  // SD Card Init
  SD32_initSDCard(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN, sdCardReady);

  if (sdCardReady) {
    // Create session directory
    SD32_createSessionDir(sessionNumber, sessionDirPath);

    // Generate file paths for each BMU
    for (int i = 0; i < MODULE_NUM; i++) {
      SD32_generateFilenameInDir(bmuFilePaths[i], sessionDirPath, "bmu", i);
      Serial.printf("  BMU %d file: %s\n", i, bmuFilePaths[i]);
    }

    // Generate AMS summary file path
    SD32_generateFilenameInDir(amsFilePath, sessionDirPath, "ams_summary", -1);
    Serial.printf("  AMS file: %s\n", amsFilePath);

    // Create CSV files with headers
    for (int i = 0; i < MODULE_NUM; i++) {
      SD32_createCSVFile(bmuFilePaths[i], header_BMU);
    }
    SD32_createCSVFile(amsFilePath, header_AMS);

    // Open all files for persistent logging
    openAllFiles();
  }

  // RTCcalibrate(rtc, WiFi32_getNTPTime()/1000ULL,RTCavailable); 
  syncTime_setSyncPoint(RTC_UNIX_TIME, RTCavailable ? RTC_getUnix(rtc,RTCavailable) : 1000000000000ULL);

  Serial.print("Time synced: ");
  Serial.println(RTC_getISO(rtc,RTCavailable));
  // Create mutex for shared sensor data
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    Serial.println("[ERROR] Failed to create dataMutex!");
  }

  // Create mutex for Serial prints (prevents garbled output across cores)
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    Serial.println("[ERROR] Failed to create serialMutex!");
  }

  // Create SD queue (hold up to 30 entries)
  sdQueue = xQueueCreate(30, sizeof(SDLogEntry));
  if (sdQueue == NULL) {
    Serial.println("[ERROR] Failed to create SD queue!");
  }

  // Create BPMobile task on Core 0
  xTaskCreatePinnedToCore(
    BPMobileTask,
    "BPMobileTask",
    8192,  // Stack size for JSON serialization
    NULL,
    1,
    &BPMobileTaskHandle,
    0      // Core 0
  );
  Serial.println("[FreeRTOS] BPMobile task started on Core 0");

  // Create SD task on Core 1
  xTaskCreatePinnedToCore(
    sdTask,
    "SDTask",
    8192,  // Larger stack for multiple file operations
    NULL,
    1,
    &sdTaskHandle,
    1      // Core 1
  );
  Serial.println("[FreeRTOS] SD Logger task started on Core 1");

  Serial.println("==================================================");
  Serial.println("       BMS Data Logger - Ready");
  Serial.printf("       Session: %d | BMU Count: %d\n", sessionNumber, MODULE_NUM);
  Serial.printf("       Client: %s\n", clientName);
  Serial.println("==================================================");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {

  uint64_t SESSION_TIME_MS = millis();
  // Time Sync: fetch RTC DS3231 every 1s 
  uint64_t Time_placeholder = 0;
  if (SESSION_TIME_MS - lastTimeSourceSync >= LOCAL_SYNC_INTERVAL) {
    Time_placeholder = (uint64_t)RTC_getUnix(rtc,RTCavailable)*1000ULL;
    lastTimeSourceSync = SESSION_TIME_MS;
  }
  // Set syncpoint , and calculate UnixTime_ms + ElapseTime_ms 
  if (Time_placeholder > 0) (RTC_UNIX_TIME, Time_placeholder);
  uint64_t CURRENT_UNIX_TIME_MS = syncTime_calcRelative_ms(RTC_UNIX_TIME);
  /* DEBUG TIME */
  // char timeBuf[32];
  // syncTime_formatUnix(timeBuf, CURRENT_UNIX_TIME_MS, 7);  // UTC+7
  // Serial.println(timeBuf);
  // // Serial.println(CURRENT_UNIX_TIME_MS);
  // return;


  // Time Sync: Remote source -> DS3231 (60s period)
  if (SESSION_TIME_MS - lastExternalSync >= REMOTE_SYNC_INTERVAL) {
    uint64_t externalTime = WiFi32_getNTPTime();              // NTP
    // uint64_t externalTime = BPMobile_getLastServerTime();  // Server

    if (externalTime > 0 && RTCavailable) {
      if(syncTime_ifDrifted(RTC_UNIX_TIME, externalTime,1000))
        RTCcalibrate(rtc,RTC_UNIX_TIME/1000ULL,RTCavailable);
    }
    lastExternalSync = SESSION_TIME_MS;
  }


  // Mock data for testing (remove in production)
  #ifdef MOCK_DATA
  for (int i = 0; i < MODULE_NUM; i++) {
    BMU_Package[i].BMU_ID = i;
    // V_CELL: encoded as uint8_t with 0.02V factor (3.6V = 180, 4.1V = 205)
    uint16_t moduleSum = 0;
    for (int j = 0; j < CELL_NUM; j++) {
      BMU_Package[i].V_CELL[j] = 180 + (rand() % 25);  // 3.6-4.1V range
      moduleSum += BMU_Package[i].V_CELL[j];
    }
    BMU_Package[i].V_MODULE = moduleSum;  // Sum of encoded cell values
    // TEMP_SENSE: encoded as uint8_t with offset -40 and 0.5C factor (25C = 130)
    BMU_Package[i].TEMP_SENSE[0] = 130 + (rand() % 20);  // 25-35C range
    BMU_Package[i].TEMP_SENSE[1] = 130 + (rand() % 20);
    BMU_Package[i].DV = rand() % 3;  // 0-0.2V DV (factor 0.1V)
    BMU_Package[i].BMUconnected = true;
  }
  // AMS_Package: ACCUM_VOLTAGE is in actual volts (aggregated from BCU)
  AMS_Package.ACCUM_VOLTAGE = 336.0f + (rand() % 20);
  AMS_Package.AMS_OK = true;
  #endif

  // Check SD card health
  if (sdCardReady && !SD32_checkSDconnect()) {
    closeAllFiles();
    sdCardReady = false;
    Serial.println("[SD Card] Card removed - files closed safely");
  }

  // Queue SD log entry
  if (sdCardReady && sdQueue != NULL && (SESSION_TIME_MS - lastSDLog >= SD_APPEND_INTERVAL)) {
    SDLogEntry entry;
    entry.dataPoint = dataPoint;
    entry.unixTime = CURRENT_UNIX_TIME_MS;
    entry.sessionTime = SESSION_TIME_MS;

    // Copy sensor data with mutex
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      memcpy(entry.bmu, BMU_Package, sizeof(BMU_Package));
      entry.ams = AMS_Package;
      xSemaphoreGive(dataMutex);
    }

    // Non-blocking queue send
    if (xQueueSend(sdQueue, &entry, 0) == pdTRUE) {
      dataPoint++;
    } else {
      Serial.println("[Loop] SD queue full - data dropped");
    }

    lastSDLog = SESSION_TIME_MS;
  }

  // Small delay to prevent watchdog issues
  delay(10);
}

// ============================================================================
// FILE MANAGEMENT
// ============================================================================
void openAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    bmuFiles[i] = SD.open(bmuFilePaths[i], FILE_APPEND);
    if (!bmuFiles[i]) {
      Serial.printf("[SD Card] ERROR: Could not open BMU file %d\n", i);
    }
  }
  amsFile = SD.open(amsFilePath, FILE_APPEND);
  if (!amsFile) {
    Serial.println("[SD Card] ERROR: Could not open AMS file");
  }
  filesOpen = true;
  Serial.println("[SD Card] All files opened for logging");
}

void closeAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    if (bmuFiles[i]) {
      bmuFiles[i].flush();
      bmuFiles[i].close();
    }
  }
  if (amsFile) {
    amsFile.flush();
    amsFile.close();
  }
  filesOpen = false;
  Serial.println("[SD Card] All files closed");
}

void flushAllFiles() {
  for (int i = 0; i < MODULE_NUM; i++) {
    if (bmuFiles[i]) {
      bmuFiles[i].flush();
    }
  }
  if (amsFile) {
    amsFile.flush();
  }
}

// ============================================================================
// CSV APPENDER FUNCTIONS
// ============================================================================
void append_BMU_toCSVFile(File& dataFile, BMUdata* bmu, int dp, uint64_t Timestamp, uint64_t session) {
  // DataPoint, UnixTime, SessionTime
  dataFile.print(dp); dataFile.print(",");
  dataFile.print(Timestamp); dataFile.print(",");
  dataFile.print(session); dataFile.print(",");

  // V_MODULE (scaled: 0.02V per bit), TEMP1, TEMP2 (decoded: (val * 0.5) - 40), DV (scaled: 0.1V per bit)
  dataFile.print(bmu->V_MODULE * 0.02f, 2); dataFile.print(",");
  dataFile.print((bmu->TEMP_SENSE[0] * 0.5f) - 40.0f, 1); dataFile.print(",");
  dataFile.print((bmu->TEMP_SENSE[1] * 0.5f) - 40.0f, 1); dataFile.print(",");
  dataFile.print(bmu->DV * 0.1f, 2); dataFile.print(",");

  // Cell voltages (10 cells, scaled: 0.02V per bit)
  for (int i = 0; i < CELL_NUM; i++) {
    dataFile.print(bmu->V_CELL[i] * 0.02f, 3);
    dataFile.print(",");
  }

  // Fault flags (16-bit bitmasks, log as hex for readability)
  dataFile.print(bmu->OVERVOLTAGE_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERVOLTAGE_CRITICAL, HEX); dataFile.print(",");
  dataFile.print(bmu->LOWVOLTAGE_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->LOWVOLTAGE_CRITICAL, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERTEMP_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERTEMP_CRITICAL, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERDIV_VOLTAGE_WARNING, HEX); dataFile.print(",");
  dataFile.print(bmu->OVERDIV_VOLTAGE_CRITICAL, HEX); dataFile.print(",");

  // Status (balancing as hex bitmask)
  dataFile.print(bmu->BalancingDischarge_Cells, HEX); dataFile.print(",");
  dataFile.print(bmu->BMUconnected); dataFile.print(",");
  dataFile.println(bmu->BMUreadytoCharge);
}

void append_AMS_toCSVFile(File& dataFile, AMSdata* ams, int dp, uint64_t Timestamp, uint64_t session) {
  // DataPoint, UnixTime, SessionTime
  dataFile.print(dp); dataFile.print(",");
  dataFile.print(Timestamp); dataFile.print(",");
  dataFile.print(session); dataFile.print(",");

  // Accumulator data (ACCUM_VOLTAGE is already in volts from BCU aggregation)
  dataFile.print(ams->ACCUM_VOLTAGE, 2); dataFile.print(",");
  dataFile.print(ams->ACCUM_MAXVOLTAGE, 2); dataFile.print(",");
  dataFile.print(ams->ACCUM_MINVOLTAGE, 2); dataFile.print(",");

  // Status flags (bool values: 0 or 1)
  dataFile.print(ams->AMS_OK ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->ACCUM_CHG_READY ? 1 : 0); dataFile.print(",");

  // Warning/Critical flags (bool values: 0 or 1)
  dataFile.print(ams->OVERVOLT_WARNING ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->OVERVOLT_CRITICAL ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->LOWVOLT_WARNING ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->LOWVOLT_CRITICAL ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->OVERTEMP_WARNING ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->OVERTEMP_CRITICAL ? 1 : 0); dataFile.print(",");
  dataFile.print(ams->OVERDIV_WARNING ? 1 : 0); dataFile.print(",");
  dataFile.println(ams->OVERDIV_CRITICAL ? 1 : 0);
}
// ============================================================================
// BPMOBILE PUBLISHING FUNCTIONS
// ============================================================================
void registerClient(const char* name) {
  JsonDocument doc;
  doc["type"] = "register";
  doc["client_name"] = name;

  // Define topics for BMS data
  JsonArray topics = doc["topics"].to<JsonArray>();

  // BMU cell topics (one per module)
  for (int i = 0; i < MODULE_NUM; i++) {
    char topic[32];
    snprintf(topic, sizeof(topic), "bms/bmu%d/cells", i);
    topics.add(topic);
    snprintf(topic, sizeof(topic), "bms/bmu%d/faults", i);
    topics.add(topic);
  }
  topics.add("bms/ams/status");

  // Define topic metadata
  JsonObject metadata = doc["topic_metadata"].to<JsonObject>();

  for (int i = 0; i < MODULE_NUM; i++) {
    char topic[32];
    snprintf(topic, sizeof(topic), "bms/bmu%d/cells", i);
    JsonObject cellMeta = metadata[topic].to<JsonObject>();
    cellMeta["description"] = "BMU cell voltages";
    cellMeta["unit"] = "V";
    cellMeta["sampling_rate"] = BMU_CELLS_SAMPLING_RATE;

    snprintf(topic, sizeof(topic), "bms/bmu%d/faults", i);
    JsonObject faultMeta = metadata[topic].to<JsonObject>();
    faultMeta["description"] = "BMU fault flags";
    faultMeta["unit"] = "flags";
    faultMeta["sampling_rate"] = BMU_FAULT_SAMPLING_RATE;
  }

  JsonObject amsMeta = metadata["bms/ams/status"].to<JsonObject>();
  amsMeta["description"] = "AMS overall status";
  amsMeta["unit"] = "mixed";
  amsMeta["sampling_rate"] = AMS_DATA_SAMPLING_RATE;

  String registration;
  serializeJson(doc, registration);

  if (serialMutex && xSemaphoreTake(serialMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    Serial.println("[WebSocket] Sending registration...");
    xSemaphoreGive(serialMutex);
  }
  BPwebSocket->sendTXT(registration);
}

void publishBMUcells(BMUdata* bmu, int moduleNum) {
  uint64_t timestamp = WiFi32_getNTPTime();
  if (timestamp == 0) timestamp = millis();

  JsonDocument doc;
  doc["type"] = "data";

  char topic[32];
  snprintf(topic, sizeof(topic), "bms/bmu%d/cells", moduleNum);
  doc["topic"] = topic;

  JsonObject data = doc["data"].to<JsonObject>();
  data["module"] = moduleNum;
  data["v_module"] = bmu->V_MODULE * 0.02f;  // Scaled to actual volts

  JsonArray cells = data["cells"].to<JsonArray>();
  for (int i = 0; i < CELL_NUM; i++) {
    cells.add(bmu->V_CELL[i] * 0.02f);  // Scaled to actual volts
  }

  JsonArray temps = data["temps"].to<JsonArray>();
  for (int i = 0; i < TEMP_SENSOR_NUM; i++) {
    temps.add((bmu->TEMP_SENSE[i] * 0.5f) - 40.0f);  // Decoded to Celsius
  }

  data["dv"] = bmu->DV * 0.1f;  // Scaled to actual volts
  data["connected"] = bmu->BMUconnected;

  doc["timestamp"] = timestamp;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishBMUfaults(BMUdata* bmu, int moduleNum) {
  uint64_t timestamp = WiFi32_getNTPTime();
  if (timestamp == 0) timestamp = millis();

  JsonDocument doc;
  doc["type"] = "data";

  char topic[32];
  snprintf(topic, sizeof(topic), "bms/bmu%d/faults", moduleNum);
  doc["topic"] = topic;

  JsonObject data = doc["data"].to<JsonObject>();
  data["module"] = moduleNum;
  data["ov_warn"] = bmu->OVERVOLTAGE_WARNING;
  data["ov_crit"] = bmu->OVERVOLTAGE_CRITICAL;
  data["lv_warn"] = bmu->LOWVOLTAGE_WARNING;
  data["lv_crit"] = bmu->LOWVOLTAGE_CRITICAL;
  data["ot_warn"] = bmu->OVERTEMP_WARNING;
  data["ot_crit"] = bmu->OVERTEMP_CRITICAL;
  data["odv_warn"] = bmu->OVERDIV_VOLTAGE_WARNING;
  data["odv_crit"] = bmu->OVERDIV_VOLTAGE_CRITICAL;
  data["balancing"] = bmu->BalancingDischarge_Cells;
  data["ready_charge"] = bmu->BMUreadytoCharge;

  doc["timestamp"] = timestamp;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}

void publishAMSstate(AMSdata* ams) {
  uint64_t timestamp = WiFi32_getNTPTime();
  if (timestamp == 0) timestamp = millis();

  JsonDocument doc;
  doc["type"] = "data";
  doc["topic"] = "bms/ams/status";

  JsonObject data = doc["data"].to<JsonObject>();
  data["accum_voltage"] = ams->ACCUM_VOLTAGE;
  data["accum_max_v"] = ams->ACCUM_MAXVOLTAGE;
  data["accum_min_v"] = ams->ACCUM_MINVOLTAGE;
  data["ams_ok"] = ams->AMS_OK;
  data["chg_ready"] = ams->ACCUM_CHG_READY;
  data["ov_warn"] = ams->OVERVOLT_WARNING;
  data["ov_crit"] = ams->OVERVOLT_CRITICAL;
  data["lv_warn"] = ams->LOWVOLT_WARNING;
  data["lv_crit"] = ams->LOWVOLT_CRITICAL;
  data["ot_warn"] = ams->OVERTEMP_WARNING;
  data["ot_crit"] = ams->OVERTEMP_CRITICAL;
  data["odv_warn"] = ams->OVERDIV_WARNING;
  data["odv_crit"] = ams->OVERDIV_CRITICAL;

  doc["timestamp"] = timestamp;

  String msg;
  serializeJson(doc, msg);
  BPwebSocket->sendTXT(msg);
}
