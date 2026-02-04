#ifndef SD32_UTIL_H
#define SD32_UTIL_H

#include <FS.h>

// ============================================================================
// SD CARD INITIALIZATION
// ============================================================================
void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs, bool &sdCardReady);
bool SD32_checkSDconnect();
void SD32_getSDsize();

// ============================================================================
// SESSION & FILENAME MANAGEMENT
// ============================================================================
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename, const char* prefix);
void SD32_createSessionDir(int &sessionNumber, char* sessionDirPath, const char* prefix);
void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index);

// ============================================================================
// CSV LOGGING - APPENDER ARCHITECTURE
// ============================================================================
typedef void (*AppenderFunc)(File&, void*);

// Create CSV file with header
void SD32_createCSVFile(char* csvFilename, const char* csvHeader);
// Non-persistent: opens file, appends, closes (simple but slower)
void SD32_appendBulkDataToCSV(const char* filepath, AppenderFunc* appenders, void** dataArray, size_t count);

// ============================================================================
// PERSISTENT FILE HANDLE (recommended for continuous logging)
// ============================================================================
// Open file once, keep open for continuous logging
bool SD32_openPersistentFile(const char* filepath);
void SD32_closePersistentFile();
bool SD32_isPersistentFileOpen();

// Append data to persistent file
// flushIntervalMs: time between flushes (0 = flush every write)
void SD32_appendBulkDataPersistent(AppenderFunc* appenders, void** dataArray, size_t count, unsigned long flushIntervalMs, unsigned long closeIntervalMs);

// Force flush (call before power off or SD removal)
void SD32_flushPersistentFile();

#endif
