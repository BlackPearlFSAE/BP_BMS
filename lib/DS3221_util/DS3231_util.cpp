#include <Arduino.h>
#include <Wire.h>
#include <RTClib.h>
#include "DS3231_util.h"

bool RTCinit(RTC_DS3231 &rtc,TwoWire* WireRTC) {
  if (!rtc.begin(WireRTC)) {
    Serial.println("[RTC] Not found");
    return false;
  }
  Serial.println("[RTC] Found");
  return true;
}

void RTCcalibrate(RTC_DS3231 &rtc,uint32_t unix_time, bool &flag) {
  if (!flag) return;
  rtc.adjust(DateTime(unix_time));
}

void RTCcalibrate(RTC_DS3231 &rtc, bool &flag) {
  if (!flag) return;
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

uint32_t RTC_getUnix(RTC_DS3231 &rtc, bool &flag) {
  if (!flag) return 0;
  return rtc.now().unixtime();
}

String RTC_getISO(RTC_DS3231 &rtc , bool &flag) {
  if (!flag) return "Unknown";
  return rtc.now().timestamp(DateTime::TIMESTAMP_FULL);
}
