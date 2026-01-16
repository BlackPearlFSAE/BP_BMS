# syncTime_util_v2 - Clean Time System

## Core Concept

**Your Vision Implemented:**
- `DeviceTime_Absolute` = Unix timestamp from ANY source (RTC/NTP/Server)
- `DeviceTime_Relative` = Auto-calculated from sync point + millis()
- **Source-agnostic**: Swap time sources without changing code
- **Simple uint64_t**: No library dependencies, universal format

---

## Quick Start

### 1. Sync from ANY Source

```cpp
// From RTC hardware
uint64_t rtcTime = RTC_getUnix() * 1000ULL;  // Convert to ms
syncTime_setAbsolute(deviceAbsoluteTime, rtcTime);

// From NTP server
uint64_t ntpTime = getNTPTime();  // Already in ms
syncTime_setAbsolute(deviceAbsoluteTime, ntpTime);

// From WebSocket server
uint64_t serverTime = jsonDoc["timestamp"];
syncTime_setAbsolute(deviceAbsoluteTime, serverTime);

// Manual (for testing)
syncTime_setAbsolute(deviceAbsoluteTime, 1736000000000ULL);  // Jan 4, 2026
```

### 2. Get Current Time

```cpp
// Always returns current time (auto-calculated)
uint64_t now = syncTime_getRelative(deviceAbsoluteTime);  // Unix timestamp in ms

// Check if synced
if (syncTime_isSynced()) {
  Serial.println("Time is synced!");
} else {
  Serial.println("Using millis() fallback");
}
```

### 3. Format for Display

```cpp
char buf[32];
uint64_t now = syncTime_getRelative(deviceAbsoluteTime);

// Bangkok time (UTC+7)
syncTime_formatUnix(buf, now, 7);
Serial.println(buf);  // "2026-01-04 19:30:45"

// UTC time
syncTime_formatUnix_UTC(buf, now);
Serial.println(buf);  // "2026-01-04 12:30:45"

// Any timezone
syncTime_formatUnix(buf, now, -5);  // EST (UTC-5)
Serial.println(buf);  // "2026-01-04 07:30:45"
```

---

## Advanced Features

### Drift Checking (Auto-Resync)

```cpp
// Only resyncs if drift exceeds 1 second (default)
bool resynced = syncTime_resync(deviceAbsoluteTime, newServerTime);

if (resynced) {
  Serial.println("Time was resynced (drift exceeded threshold)");
} else {
  Serial.println("No resync needed (drift < 1 second)");
}

// Custom drift threshold (5 seconds)
syncTime_resync(deviceAbsoluteTime, newServerTime, 5000ULL);
```

### Direct State Access

```cpp
// Read internal state directly
Serial.print("Absolute time: ");
Serial.println(deviceAbsoluteTime_ms);

Serial.print("Sync point (millis): ");
Serial.println(deviceRelativeTime_syncPoint);

Serial.print("Is synced: ");
Serial.println(deviceTimeIsSynced ? "Yes" : "No");
```

---

## Migration from Old Code

### Old API (Still Works!)

```cpp
// These still work for backward compatibility:
uint64_t time = getSynchronizedTime();           // Same as syncTime_getRelative()
syncDevice_to_serverTime(serverTime);            // Same as syncTime_resync()
formatDateTimeBangkok(buf, time);                // Deprecated
if (timeIsSynchronized) { ... }                  // Same as syncTime_isSynced()
```

### New Clean API (Recommended)

```cpp
// Use these for new code:
uint64_t time = syncTime_getRelative(deviceAbsoluteTime);
syncTime_setAbsolute(deviceAbsoluteTime, sourceTime);
syncTime_formatUnix(buf, time, 7);  // Bangkok = UTC+7
if (syncTime_isSynced()) { ... }
```

---

## CSV Logging Example

```cpp
void logToSD() {
  uint64_t now = syncTime_getRelative(deviceAbsoluteTime);
  char dateStr[32];

  if (syncTime_isSynced()) {
    uint64_t unixSec = now / 1000ULL;
    formatUnixTime_Bangkok(dateStr, now);

    // Log: Unix timestamp, Bangkok time, data
    dataFile.print(unixSec);
    dataFile.print(",");
    dataFile.print(dateStr);
    dataFile.print(",");
    dataFile.println(sensorValue);
  } else {
    // Fallback: millis, NOT_SYNCED, data
    dataFile.print(millis() / 1000);
    dataFile.print(",NOT_SYNCED,");
    dataFile.println(sensorValue);
  }
}
```

---

## BPMobile Publishing Example

```cpp
void publishData() {
  uint64_t timestamp = syncTime_getRelative(deviceAbsoluteTime);

  // Timeout check (invalid timestamp)
  if (timestamp < 1000000000000ULL) return;

  JsonDocument doc;
  doc["type"] = "data";
  doc["topic"] = "sensor/temperature";
  doc["data"]["value"] = temperature;
  doc["timestamp"] = timestamp;  // Unix ms

  String msg;
  serializeJson(doc, msg);
  webSocket->sendTXT(msg);
}
```

---

## Time Source Priority (Example Pattern)

```cpp
void syncTimeFromBestSource() {
  // Try RTC first
  if (rtcAvailable) {
    uint64_t rtcTime = RTC_getUnix() * 1000ULL;
    syncTime_setAbsolute(deviceAbsoluteTime, rtcTime);
    Serial.println("Synced from RTC");
    return;
  }

  // Fallback to NTP
  if (WiFi.isConnected()) {
    uint64_t ntpTime = getNTPTime();
    if (ntpTime > 0) {
      syncTime_setAbsolute(deviceAbsoluteTime, ntpTime);
      Serial.println("Synced from NTP");
      return;
    }
  }

  // Fallback to millis() - no sync
  Serial.println("No time source available, using millis()");
}
```

---

## Benefits vs Old System

| Old System | New System |
|------------|------------|
| `getSynchronizedTime()` hides logic | `getDeviceRelativeTime()` clear name |
| `baseTimestamp` hidden in library | `deviceAbsoluteTime_ms` exposed |
| Time source unclear | **Source-agnostic by design** |
| Bangkok timezone hardcoded | `formatUnixTime(buf, time, offset)` |
| Mix of concerns | **Separation of concerns** |

---

## Architecture Summary

```
┌─────────────────────────────────────────┐
│  Time Sources (Choose ANY)              │
│  - RTC Hardware                         │
│  - NTP Server                           │
│  - WebSocket Server                     │
│  - Manual Setting                       │
└────────────────┬────────────────────────┘
                 │
                 ▼
    setDeviceAbsoluteTime(unixMs)
                 │
                 ▼
┌─────────────────────────────────────────┐
│  Internal State                         │
│  - deviceAbsoluteTime_ms (Unix)         │
│  - deviceRelativeTime_syncPoint         │
│  - deviceTimeIsSynced (flag)            │
└────────────────┬────────────────────────┘
                 │
                 ▼
    getDeviceRelativeTime()
    (auto-calculates: base + elapsed)
                 │
                 ▼
┌─────────────────────────────────────────┐
│  Usage                                  │
│  - SD Card Logging                      │
│  - BPMobile Publishing                  │
│  - Display (with timezone format)       │
└─────────────────────────────────────────┘
```

---

## CSV Header Matches Your Vision

```
Timestamp,DateTime,DataPoint,...
├─ Unix seconds (absolute, universal)
├─ Bangkok string (human-readable)
└─ Data point counter
```

✅ **Timestamp** = Unix seconds (works with any time source)
✅ **DateTime** = Bangkok formatted string
✅ **Source-agnostic** = Change RTC→NTP→Server without touching CSV logic

---

## Questions Answered

**Q: Does CSV match my vision?**
A: ✅ Yes! Timestamp is Unix (universal), DateTime is Bangkok (local display)

**Q: Can I swap time sources?**
A: ✅ Yes! `setDeviceAbsoluteTime()` accepts ANY source

**Q: Do I need drift checking?**
A: ✅ Yes, already built-in with 1-second threshold

**Q: What about timezone?**
A: ✅ Separate concern - `formatUnixTime(buf, time, offsetHours)`

**Q: Backward compatible?**
A: ✅ Yes! Old function names still work
