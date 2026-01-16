
#include "Arduino.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <SD32_util.h>

// ============================================================================
// PERSISTENT FILE HANDLE - Solution 1 & 2
// ============================================================================
static File _persistentFile;
static bool _persistentFileOpen = false;
static unsigned long _lastFlushTime = 0;

void SD32_getSDsize(){
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs,bool &sdCardReady) { 
  Serial.println("--- SD Card Initialization ---");
  Serial.print("Initializing SD card...");
  SPI.begin(sd_sck, sd_miso, sd_mosi, sd_cs);
  
  if (!SD.begin(sd_cs)) {
    Serial.println(" FAILED!");
    Serial.println("SD card logging disabled.");
    Serial.println("Check:");
    Serial.println("  - SD card is inserted");
    Serial.println("  - Connections are correct");
    Serial.println("  - SD card is formatted (FAT32)");
    sdCardReady = false; // Passed referece
    return;
  }
  Serial.println(" SUCCESS!");
  sdCardReady = true;
  
  // // Display card info
  uint8_t cardType = SD.cardType();
  Serial.print("Card Type: ");
  if (cardType == CARD_MMC) 
    Serial.println("MMC");
  else if (cardType == CARD_SD) 
    Serial.println("SDSC");
  else if (cardType == CARD_SDHC) 
    Serial.println("SDHC");
  else 
    Serial.println("UNKNOWN");
  
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Card Size: %lluMB\n", cardSize);
}

bool SD32_checkSDconnect(){
  uint8_t cardType = SD.cardType();

  // If the card is pulled out, cardType will return CARD_NONE (0)
  if(cardType == CARD_NONE){
    return false;
  }
  return true;
}

// ============================================================================
// PERSISTENT FILE FUNCTIONS (Solution 1 & 2)
// ============================================================================

bool SD32_openPersistentFile(fs::FS &fs, const char* filepath) {
  if (_persistentFileOpen) {
    Serial.println("[SD Card] Persistent file already open");
    return true;
  }

  _persistentFile = fs.open(filepath, FILE_APPEND);
  if (!_persistentFile) {
    Serial.println("[SD Card] ERROR: Could not open persistent file!");
    _persistentFileOpen = false;
    return false;
  }

  _persistentFileOpen = true;
  _lastFlushTime = millis();
  Serial.printf("[SD Card] Persistent file opened: %s\n", filepath);
  return true;
}

void SD32_closePersistentFile() {
  if (_persistentFileOpen && _persistentFile) {
    _persistentFile.flush();
    _persistentFile.close();
    _persistentFileOpen = false;
    Serial.println("[SD Card] Persistent file closed");
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

void SD32_appendBulkDataPersistent(AppenderFunc* appenders, void** dataArray, size_t count, unsigned long flushIntervalMs) {
  if (!_persistentFileOpen || !_persistentFile) {
    Serial.println("[SD Card] ERROR: Persistent file not open!");
    return;
  }

  // Write all data using appenders (fast - just RAM buffer)
  for (size_t i = 0; i < count; i++) {
    appenders[i](_persistentFile, dataArray[i]);
  }

  // End CSV line
  _persistentFile.println();

  // Flush based on time interval (Solution 2)
  unsigned long now = millis();
  if (flushIntervalMs == 0 || (now - _lastFlushTime >= flushIntervalMs)) {
    _persistentFile.flush();
    _lastFlushTime = now;
  }
}

void SD32_createCSVFile(char* csvFilename, const char* csvHeader){
  File dataFile = SD.open((const char*)csvFilename, FILE_WRITE);
  
  if (dataFile) {
    // Updated CSV header with DateTime column
    dataFile.println(csvHeader);
    dataFile.flush();
    dataFile.close();
    Serial.println("[SD Card] CSV header created with DateTime column");
  } else {
    Serial.println("[SD Card] ERROR: Could not create CSV file!");
  }

}
// Pass unique file name in (default prefix: "datalog")
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename) {
  SD32_generateUniqueFilename(sessionNumber, csvFilename, "datalog");
}

// Overload with custom prefix
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename, const char* prefix) {
  // Find the next available session number
  sessionNumber = 0;

  // Build search pattern from prefix
  String searchPrefix = String(prefix) + "_";
  int prefixLen = searchPrefix.length();

  // Check existing files and find the highest session number
  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      String filename = entry.name();
      // Look for files matching pattern: prefix_NNN.csv
      if (filename.startsWith(searchPrefix) && filename.endsWith(".csv")) {
        // Extract the number
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

  // Generate filename with 3-digit session number
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "/%s_%03d.csv", prefix, sessionNumber);
  strncpy(csvFilename, buffer, 31);
  csvFilename[31] = '\0';

  Serial.print("Generated unique filename: ");
  Serial.println(csvFilename);
}

// Create session directory with auto-incrementing number
void SD32_createSessionDir(int &sessionNumber, char* dirPath) {
  sessionNumber = 0;

  // Check existing directories and find the highest session number
  File root = SD.open("/");
  if (root) {
    File entry = root.openNextFile();
    while (entry) {
      if (entry.isDirectory()) {
        String dirname = entry.name();
        // Look for directories matching pattern: session_NNN
        if (dirname.startsWith("session_")) {
          int prefixLen = 8;  // Length of "session_"
          String numStr = dirname.substring(prefixLen);
          int dirNum = numStr.toInt();
          if (dirNum >= sessionNumber) {
            sessionNumber = dirNum + 1;
          }
        }
      }
      entry.close();
      entry = root.openNextFile();
    }
    root.close();
  }

  // Generate directory path
  snprintf(dirPath, 32, "/session_%03d", sessionNumber);

  // Create the directory
  if (SD.mkdir(dirPath)) {
    Serial.printf("[SD Card] Session directory created: %s\n", dirPath);
  } else {
    Serial.printf("[SD Card] Failed to create directory: %s\n", dirPath);
  }
}

// Generate filepath inside a directory
void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index) {
  if (index >= 0) {
    snprintf(filepath, 48, "%s/%s_%03d.csv", dirPath, prefix, index);
  } else {
    // No index (e.g., for ams_summary.csv)
    snprintf(filepath, 48, "%s/%s.csv", dirPath, prefix);
  }
}


// List Directory
void SD32_listDir(fs::FS &fs, const char *dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        SD32_listDir(fs, file.path(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

// Make Directory folders
void SD32_createDir(fs::FS &fs, const char *path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

// remove Directory folders
void SD32_removeDir(fs::FS &fs, const char *path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

// read file
void SD32_readFile(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

// Overwrite file
void SD32_writeFile(fs::FS &fs, const char *path, const char* message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (!file.print(message)) 
    Serial.println("Write failed");

  file.close();
}

// Update file at last line
void SD32_appendFile(fs::FS &fs, const char *path, const char* message) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  // The compiler will check if .print() supports type T
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.flush();
  file.close();
}

void SD32_appendCSV(fs::FS &fs, const char *path, uint64_t &message, bool continueLine) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  // The compiler will check if .print() supports type T
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.print(",");
  if(!continueLine){
    file.flush();
    file.close();
  }
}

void SD32_appendCSV(fs::FS &fs, const char *path, int &message, bool continueLine) {
  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  // The compiler will check if .print() supports type T
  if (!file.print(message)) {
    Serial.println("Append failed");
  }
  file.print(",");
  if(!continueLine){
    file.flush();
    file.close();
  }
}

// Log data in bulk
void SD32_appendBulkDataToCSV(fs::FS &fs,const char* filepath, AppenderFunc* appenders, void** dataArray, size_t count) {
  // File dataFile = fs.open(filepath, FILE_APPEND);
  File file = fs.open(filepath, FILE_APPEND);
  if (!file) {
    Serial.println("[SD Card] ERROR: Could not open file!");
    return;
  }
  // Call all appenders with safety checks
  // for (size_t i = 0; i < count; i++) {
  //   if (!appenders || !appenders[i] || !dataArray || !dataArray[i]) {
  //     Serial.printf("[SD Card] WARNING: Appender %u has null pointer, skipping\n", i);
  //     continue;
  //   }
  //   appenders[i](file, dataArray[i]);
  // }

  for (size_t i = 0; i < count; i++)
    appenders[i](file, dataArray[i]);

  // End CSV line
  file.println();
  file.flush();
  file.close();
}

// rename
void SD32_renameFile(fs::FS &fs, const char *path1, const char *path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

// delete
void SD32_deleteFile(fs::FS &fs, const char *path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

// Tester
void SD32_testFileIO(fs::FS &fs, const char *path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %lu ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }

  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %lu ms\n", 2048 * 512, end);
  file.close();
}

