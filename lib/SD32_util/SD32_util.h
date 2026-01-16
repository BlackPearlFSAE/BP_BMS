#include <cstdint>
// declare class and namespace of file system libary (include and call in .cpp src file)
namespace fs{
class FS;
};

// ------- SD CARD LOGGING - APPENDER ARCHITECTURE
typedef void (*AppenderFunc)(File&, void*);
void SD32_getSDsize();
void SD32_initSDCard(int sd_sck, int sd_miso, int sd_mosi, int sd_cs,bool &sdCardReady);
bool SD32_checkSDconnect();

// ------- PERSISTENT FILE HANDLE (Solution 1 & 2) -------
// Open file once, keep open for continuous logging
bool SD32_openPersistentFile(fs::FS &fs, const char* filepath);
void SD32_closePersistentFile();
bool SD32_isPersistentFileOpen();
// Append data without opening/closing file each time
// flushIntervalMs: time between flushes (0 = flush every write, recommended: 2000-5000ms)
void SD32_appendBulkDataPersistent(AppenderFunc* appenders, void** dataArray, size_t count, unsigned long flushIntervalMs);
// Force flush (call before power off or SD removal)
void SD32_flushPersistentFile();

void SD32_writeFile(fs::FS &fs, const char *path, const char* message);
void SD32_appendFile(fs::FS &fs, const char *path, const char* message);
void SD32_appendCSV(fs::FS &fs, const char *path, uint64_t &message,bool continueLine);
void SD32_appendCSV(fs::FS &fs, const char *path, int &message, bool continueLine);
void SD32_appendBulkDataToCSV(fs::FS &fs,const char* filepath, AppenderFunc* appenders, void** dataArray, size_t count);

void SD32_listDir(fs::FS &fs, const char *dirname, uint8_t levels);
void SD32_createDir(fs::FS &fs, const char *path);
void SD32_removeDir(fs::FS &fs, const char *path);
void SD32_readFile(fs::FS &fs, const char *path);
void SD32_renameFile(fs::FS &fs, const char *path1, const char *path2);
void SD32_deleteFile(fs::FS &fs, const char *path);
void SD32_testFileIO(fs::FS &fs, const char *path);

void SD32_createCSVFile(char* csvFilename, const char* csvHeader);
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename);
// Overload with custom prefix (e.g., "bmu", "ams")
void SD32_generateUniqueFilename(int &sessionNumber, char* csvFilename, const char* prefix);
// Session directory management
void SD32_createSessionDir(int &sessionNumber, char* dirPath);
void SD32_generateFilenameInDir(char* filepath, const char* dirPath, const char* prefix, int index);