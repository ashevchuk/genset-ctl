/*
 * datetime.h — minimal calendar/Unix-time value type.
 *
 * Replaces RTClib's DateTime class (RTClib itself was already reduced to
 * exactly this — RTC_DS3231/RTC_I2C's I2C access was replaced by
 * rtc_dev.h/.cpp; this was the last piece keeping RTClib + its
 * Adafruit_BusIO dependency linked in for a class that never touches I2C
 * at all). Pure calendar math, ported directly from RTClib.
 */
#ifndef DATETIME_H
#define DATETIME_H

#include <Arduino.h>

#define SECONDS_FROM_1970_TO_2000 946684800UL  // Unixtime for 2000-01-01 00:00:00

class DateTime {
public:
    // Default-constructs to 2000-01-01 00:00:00, same as RTClib.
    DateTime(uint32_t t = SECONDS_FROM_1970_TO_2000);
    DateTime(uint16_t year, uint8_t month, uint8_t day,
             uint8_t hour = 0, uint8_t min = 0, uint8_t sec = 0);

    // From the compiler's __DATE__/__TIME__ macros, e.g. DateTime(F(__DATE__), F(__TIME__)).
    DateTime(const char *date, const char *time);
    DateTime(const __FlashStringHelper *date, const __FlashStringHelper *time);

    uint16_t year(void) const { return 2000U + yOff_; }
    uint8_t month(void) const { return m_; }
    uint8_t day(void) const { return d_; }
    uint8_t hour(void) const { return hh_; }
    uint8_t minute(void) const { return mm_; }
    uint8_t second(void) const { return ss_; }

    uint8_t dayOfTheWeek(void) const;  // 0 = Sunday .. 6 = Saturday
    uint32_t unixtime(void) const;

private:
    uint8_t yOff_, m_, d_, hh_, mm_, ss_;
};

#endif // DATETIME_H
