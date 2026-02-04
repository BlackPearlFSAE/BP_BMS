#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SD32_util.h>

// ============================================================================
// PERSISTENT FILE HANDLE
// ============================================================================
static File _persistentFile;
static bool _persistentFileOpen = false;
static unsigned long _lastFlushTime = 0;
static unsigned long _lastCloseTime = 0;
static char _persistentFilePath[48] = {0};

// ============================================================================
// SD CARD INITIALIZATION
// ============================================================================

void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs, bool &sdCardReady) {
  Serial.println("--- SD Card Initialization ---");
  Serial.print("Initializing SD card...");
  SPI.begin(sd_sck, sd_miso, sd_mosi, sd_cs);

  if (!SD.begin(sd_cs, SPI, 4000000, "/sd", 10, false)) {
    Serial.println(" FAILED!");
    Serial.println("SD card logging disabled.");
    Serial.println("Check:");
    Serial.println("  - SD card is inserted");
    Serial.println("  - Connections are correct");
    Serial.println("  - SD card is formatted (FAT32)");
    sdCardReady = false;
    return;
  }
  Serial.println(" SUCCESS!");
  sdCardReady = true;

  uint8_t cardType = SD.cardType();
  Serial.print("Card Type: ");
  if (cardType == CARD_MMC) Serial.println("MMC");
  else if (cardType == CARD_SD) Serial.println("SDSC");
  else if (cardType == CARD_SDHC) Serial.println("SDHC");
  else Serial.println("UNKNOWN");

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card Size: %lluMB\n", cardSize);
}

bool SD32_checkSDconnect() {
  return SD.cardType() != CARD_NONE;
}

void SD32_getSDsize() {
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

// ============================================================================
// SESSION DIRECTORY MANAGEMENT
// ============================================================================

void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename, const char* prefix) {
  sessionNumber = 0;
  String searchPrefix = String(prefix) + "_";
  int prefixLen = searchPrefix.length();

  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      // entry.name() returns full path (e.g. "/Front_001.csv"), strip leading '/'
      String filename = entry.name();
      if (filename.startsWith("/")) filename = filename.substring(1);
      if (filename.startsWith(searchPrefix) && filename.endsWith(".csv")) {
        int endIdx = filename.indexOf(".csv");
        if (endIdx > prefixLen) {
          String numStr = filename.substring(prefixLen, endIdx);
          int fileNum = numStr.toInt();
          if (fileNum >= sessionNumber) {
            sessionNumber = fileNum + 1;
          }
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  char buffer[32];
  snprintf(buffer, sizeof(buffer), "/%s_%03d.csv", prefix, sessionNumber);
  strncpy(csvFilename, buffer, 31);
  csvFilename[31] = '\0';

  Serial.print("Generated unique filename: ");
  Serial.println(csvFilename);
}

void SD32_createSessionDir(int &sessionNumber, char* sessionDirPath, const char* prefix) {
  sessionNumber = 0;

  // Build search pattern: "{prefix}_session_"
  char searchPattern[32];
  snprintf(searchPattern, sizeof(searchPattern), "%s_session_", prefix);
  int patternLen = strlen(searchPattern);

  // Find next available session number
  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      if (entry.isDirectory()) {
        // entry.name() returns full path (e.g. "/Front_session_000"), strip leading '/'
        String dirname = entry.name();
        if (dirname.startsWith("/")) dirname = dirname.substring(1);
        if (dirname.startsWith(searchPattern)) {
          int num = dirname.substring(patternLen).toInt();
          if (num >= sessionNumber) {
            sessionNumber = num + 1;
          }
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  // Create session directory
  snprintf(sessionDirPath, 48, "/%s_session_%03d", prefix, sessionNumber);
  if (!SD.exists(sessionDirPath)) {
    SD.mkdir(sessionDirPath);
    Serial.printf("[SD] Created session directory: %s\n", sessionDirPath);
  }
}

void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index) {
  if (index >= 0) {
    snprintf(filepath, 48, "%s/%s_%d.csv", dirPath, prefix, index);
  } else {
    snprintf(filepath, 48, "%s/%s.csv", dirPath, prefix);
  }
}

// ============================================================================
// NON-PERSISTENT APPEND (opens/closes file each call)
// ============================================================================

void SD32_createCSVFile(char* csvFilename, const char* csvHeader) {
  File dataFile = SD.open((const char*)csvFilename, FILE_WRITE);
  if (dataFile) {
    dataFile.println(csvHeader);
    dataFile.flush();
    dataFile.close();
    Serial.printf("[SD] CSV file created: %s\n", csvFilename);
  } else {
    Serial.println("[SD] ERROR: Could not create CSV file!");
  }
}

void SD32_appendBulkDataToCSV(const char* filepath, AppenderFunc* appenders, void** dataArray, size_t count) {
  File file = SD.open(filepath, FILE_APPEND);
  if (!file) {
    Serial.println("[SD] ERROR: Could not open file!");
    return;
  }

  for (size_t i = 0; i < count; i++) {
    appenders[i](file, dataArray[i]);
  }

  file.println();
  file.flush();
  file.close();
}

// ============================================================================
// PERSISTENT FILE FUNCTIONS
// ============================================================================

bool SD32_openPersistentFile(const char* filepath) {
  if (_persistentFileOpen) {
    Serial.println("[SD] Persistent file already open");
    return true;
  }

  _persistentFile = SD.open(filepath, FILE_APPEND);
  if (!_persistentFile) {
    Serial.println("[SD] ERROR: Could not open persistent file!");
    _persistentFileOpen = false;
    return false;
  }

  _persistentFileOpen = true;
  _lastFlushTime = millis();
  strncpy(_persistentFilePath, filepath, sizeof(_persistentFilePath) - 1);
  _persistentFilePath[sizeof(_persistentFilePath) - 1] = '\0';
  Serial.printf("[SD] Persistent file opened: %s\n", filepath);
  return true;
}

void SD32_closePersistentFile() {
  if (_persistentFileOpen && _persistentFile) {
    _persistentFile.flush();
    _persistentFile.close();
    _persistentFileOpen = false;
    _persistentFilePath[0] = '\0';
    Serial.println("[SD] Persistent file closed");
  }
}

bool SD32_isPersistentFileOpen() {
  return _persistentFileOpen;
}

void SD32_flushPersistentFile() {
  if (_persistentFileOpen && _persistentFile) {
    _persistentFile.flush();
    _lastFlushTime = millis();
  }
}

void SD32_appendBulkDataPersistent(AppenderFunc* appenders, void** dataArray, size_t count, unsigned long flushIntervalMs, unsigned long closeIntervalMs) {
  if (!_persistentFileOpen || !_persistentFile) {
    Serial.println("[SD] ERROR: Persistent file not open!");
    return;
  }

  for (size_t i = 0; i < count; i++) {
    appenders[i](_persistentFile, dataArray[i]);
  }

  _persistentFile.println();

  unsigned long now = millis();
  if (flushIntervalMs == 0 || (now - _lastFlushTime >= flushIntervalMs)) {
    _persistentFile.flush();
    _lastFlushTime = now;
  }
  if (closeIntervalMs > 0 && (now - _lastCloseTime >= closeIntervalMs)) {
    _persistentFile.flush();
    _persistentFile.close();
    // Reopen the file
    _persistentFile = SD.open(_persistentFilePath, FILE_APPEND);
    if (!_persistentFile) {
      _persistentFileOpen = false;
      Serial.println("[SD] ERROR: Could not reopen persistent file after cycle!");
    }
    _lastCloseTime = now;
  }
}
