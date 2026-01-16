#ifndef WIFI32_UTIL_H
#define WIFI32_UTIL_H

#include <cstdint>

// ============================================================================
// WIFI INITIALIZATION
// ============================================================================
void initWiFi(const char* ssid, const char* password, int attempt);
bool WiFi32_isConnected();
int  WiFi32_getRSSI();

// ============================================================================
// NTP TIME FUNCTIONS
// ============================================================================
void WiFi32_initNTP(const char* ntpServer1 = "pool.ntp.org",
                    const char* ntpServer2 = "time.nist.gov",
                    long gmtOffsetSec = 0);
uint64_t WiFi32_getNTPTime();  // Returns Unix timestamp in ms, 0 if unavailable
bool WiFi32_isNTPSynced();

#endif // WIFI32_UTIL_H
