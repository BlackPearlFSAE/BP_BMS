#include <Arduino.h>
#include <syncTime_util.h>
// #include <RTClib.h>  // For RTC_DS3231

unsigned long sync_point = 0;  // millis() when last synced
// Sync status flag
bool localTimeIsSynced = false;

// ============================================================================
// CORE TIME FUNCTIONS
// ============================================================================

/**
 * Set absolute time from ANY source (RTC, NTP, Server, Manual)
 * Accepts: Unix timestamp in milliseconds
 * Also sets ESP32 internal RTC via settimeofday()
**/
void syncTime_setSyncPoint(uint64_t &localTime , uint64_t Timesource_s) {
  // Set ESP32 internal RTC via settimeofday

  localTime = Timesource_s;
  sync_point = millis();  // Mark sync point
  localTimeIsSynced = true;
}

uint32_t syncTime_getElapse_ms(){
  return millis()- sync_point;
}

/**
 * Get current device time (absolute if synced, relative if not)
 * Returns: Unix timestamp in milliseconds
 * This auto-calculates elapsed time since last sync
 */
uint64_t syncTime_calcRelative_ms(uint64_t localTime_s) {
  if (!localTimeIsSynced) {
    // Not synced - return device uptime
    return (uint64_t)millis();
  }
  uint32_t elapse_ms = syncTime_getElapse_ms();
  return localTime_s + (uint64_t)elapse_ms;
}

/**
 * Check if device time has been synchronized
 * Returns: true if synced with external source, false if using millis()
 */
bool syncTime_isSynced() {
  return localTimeIsSynced;
}

/**
 * Get current drift between device time and external source
 * Returns: drift in ms (positive = device ahead, negative = device behind)
 */
int64_t syncTime_getDrift(uint64_t localTime_s, uint64_t TimeSource_ms) {
  uint64_t currentTime = syncTime_calcRelative_ms(localTime_s);
  return (int64_t)currentTime - (int64_t)TimeSource_ms;
}

/**
 * Sync from external source (Server/NTP) with optional RTC write-back
 * Only syncs if drift exceeds threshold
 */
bool syncTime_ifDrifted(uint64_t &localTime, uint64_t TimeSource_ms,
                           uint64_t driftThreshold_ms) {
  // Check drift first
  int64_t drift = syncTime_getDrift(localTime, TimeSource_ms);

  if (llabs(drift) < (long long)driftThreshold_ms) {
    return false;  // Drift within tolerance, skip sync
  }

  // Update device time
  syncTime_setSyncPoint(localTime, TimeSource_ms);

  return true;
}

// ============================================================================
// FORMATTING FUNCTIONS - Timezone Support
// ============================================================================

/**
 * Format Unix timestamp to human-readable string with timezone offset
 * Parameters:
 *   - outBuf: Output buffer (minimum 32 chars)
 *   - unixMs: Unix timestamp in milliseconds
 *   - timezoneOffsetHours: Hours offset from UTC (e.g., 7 for Bangkok, -5 for EST)
 */
void syncTime_formatUnix(char* outBuf, uint64_t unixMs, int timezoneOffsetHours) {
  // Convert to seconds and apply timezone offset
  uint64_t unixSec = (unixMs / 1000ULL) + (timezoneOffsetHours * 3600);

  time_t rawtime = (time_t)unixSec;
  struct tm* timeinfo = gmtime(&rawtime);

  // Format: YYYY-MM-DD HH:MM:SS
  sprintf(outBuf, "%04d-%02d-%02d %02d:%02d:%02d",
          timeinfo->tm_year + 1900,
          timeinfo->tm_mon + 1,
          timeinfo->tm_mday,
          timeinfo->tm_hour,
          timeinfo->tm_min,
          timeinfo->tm_sec);
}

/**
 * Convenience wrapper for UTC formatting
 */
void syncTime_formatUnix_UTC(char* outBuf, uint64_t unixMs) {
  syncTime_formatUnix(outBuf, unixMs, 0);
}