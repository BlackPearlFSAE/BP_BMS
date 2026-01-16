#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include <WIFI32_util.h>

// ============================================================================
// NTP STATE
// ============================================================================
static bool _ntpInitialized = false;

// ============================================================================
// WIFI INITIALIZATION
// ============================================================================
void initWiFi(const char* ssid, const char* password, int attempt) {
  Serial.println("--- WiFi Initialization ---");
  Serial.printf("Connecting to: %s\n",ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < attempt) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection FAILED!");
    return;
  }
  Serial.printf("WiFi connected! at IP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Signal strength:%d dbm\n",WiFi.RSSI());
}

bool WiFi32_isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

int WiFi32_getRSSI() {
  return WiFi.RSSI();
}

// ============================================================================
// NTP TIME FUNCTIONS
// ============================================================================
void WiFi32_initNTP(const char* ntpServer1, const char* ntpServer2, long gmtOffsetSec) {
  configTime(gmtOffsetSec, 0, ntpServer1, ntpServer2);
  _ntpInitialized = true;
  Serial.println("[NTP] Initialized");
}

uint64_t WiFi32_getNTPTime() {
  if (!_ntpInitialized || !WiFi32_isConnected()) return 0;

  time_t now;
  if (time(&now) < 1000000000) return 0;  // Invalid time (before ~2001)

  return (uint64_t)now * 1000ULL;  // Convert seconds to ms
}

bool WiFi32_isNTPSynced() {
  time_t now;
  return _ntpInitialized && time(&now) > 1000000000;
}
