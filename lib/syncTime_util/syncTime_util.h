#ifndef SYNCTIME_UTIL_V2_H
#define SYNCTIME_UTIL_V2_H

#include <cstdint>
/**
 * CORE CONCEPT:
 * - localTime_ms: Unix timestamp from ANY source (RTC/NTP/Server)
 * - deviceRelativeTime: Calculated on-the-fly using millis() offset
 *
 * USAGE:
 * 1. Sync from any source: syncTime(unixMs)
 * 2. Get current time: syncTime_getElapse()
 * 3. Format for display: syncTime_formatUnix(buf, unixMs, timezoneOffset)
 */

// extern uint64_t localTime;           // Unix timestamp in ms (from external source)
// extern unsigned long deviceRelativeTime_syncPoint; // millis() when last synced
// extern bool localTimeIsSynced;                  // Sync status flag

// ============================================================================
// CORE TIME FUNCTIONS (New Clean API)
// ============================================================================
/**
 * sync LocalTime ANY source (RTC, NTP, Server, Manual)
 * set sync point to calculate relative_ms
 *
 * Example usage:
 *   syncTime(rtc.now().unixtime() * 1000ULL);  // From RTC
 *   syncTime(ntpTime);                          // From NTP
 *   syncTime(serverTime);                       // From WebSocket
 */
void syncTime_setSyncPoint(uint64_t &localTime , uint64_t Timesource_s);

/**
 * Calculate relative Time in ms scale , 
 * et current device time (auto-calculates from sync point)
 * Returns: Unix timestamp in milliseconds
 *
 * - If synced: localTime_ms + elapsed millis()
 * - If not synced: millis()
 */
uint64_t syncTime_calcRelative_ms(uint64_t localTime_s);

uint32_t syncTime_getElapse_ms();


uint64_t syncTime_calcRelative_ms(uint64_t localTime_s);

/**
 * Check if device time has been synchronized with external source
 * Returns: true if synced, false if using millis()
 */
bool syncTime_isSynced();


/**
 * Sync from external source (Server/NTP) with optional RTC write-back
 * Only syncs if drift exceeds threshold
 *
 * Parameters:
 *   - localTime: Reference to device time variable
 *   - TimeSource_ms: External timestamp in milliseconds
 *   - driftThreshold_ms: Minimum drift to trigger sync (default: 1000ms)
 *
 * Returns: true if synced, false if skipped (drift < threshold)
 */
bool syncTime_ifDrifted(uint64_t &localTime, uint64_t TimeSource_ms,
                           uint64_t driftThreshold_ms = 1000ULL);

/**
 * Get current drift between device time and external source
 * Useful for diagnostics without triggering sync
 *
 * Returns: drift in ms (positive = device ahead, negative = device behind)
 */
int64_t syncTime_getDrift(uint64_t localTime, uint64_t TimeSource_ms);



/**
 * Format Unix timestamp to human-readable string with timezone support
 *
 * Parameters:
 *   - outBuf: Output buffer (minimum 32 chars)
 *   - unixMs: Unix timestamp in milliseconds
 *   - timezoneOffsetHours: Hours offset from UTC
 *     Examples: 0=UTC, 7=Bangkok, -5=EST, 8=Singapore
 *
 * Output format: "YYYY-MM-DD HH:MM:SS"
 */
void syncTime_formatUnix(char* outBuf, uint64_t unixMs, int timezoneOffsetHours);

/**
 * Convenience wrappers for common timezones
 */
void syncTime_formatUnix_UTC(char* outBuf, uint64_t unixMs);     // UTC (GMT+0)

#endif // SYNCTIME_UTIL_V2_H