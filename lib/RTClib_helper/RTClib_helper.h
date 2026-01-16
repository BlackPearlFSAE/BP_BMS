#ifndef RTCLIB_HELPER_H
#define RTCLIB_HELPER_H
#include <cstdint>

class TwoWire;
class RTC_DS3231;
class String;

/**
 * Initialize DS3231 RTC on specified I2C bus
 * Parameters:
 *   - WireRTC: TwoWire* object (e.g., &Wire1 for I2C1)
 * Returns: true if RTC found and initialized, false otherwise
 */
bool RTCinit(RTC_DS3231 &rtc,TwoWire* WireRTC);
/**
 * Set RTC time from Unix timestamp (seconds)
 * Parameters:
 *   - unix_time: Unix timestamp in seconds
 */
void RTCcalibrate(RTC_DS3231 &rtc,uint32_t unix_time, bool &flag);
/**
 * Set RTC time from compile-time date/time
 * (Use only for development/testing)
 */
void RTCcalibrate(RTC_DS3231 &rtc, bool &flag);

/**
 * Get current Unix timestamp (seconds) from RTC
 * Returns: Unix timestamp (0 if RTC unavailable)
 */
uint32_t RTC_getUnix(RTC_DS3231 &rtc, bool &flag);

/**
 * Get current time as ISO 8601 formatted string
 * Returns: String in format "YYYY-MM-DD HH:MM:SS" (or "Unknown" if unavailable)
 */
String RTC_getISO(RTC_DS3231 &rtc, bool &flag);

#endif  // RTCLIB_HELPER_H
