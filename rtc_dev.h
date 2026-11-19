/*
 * rtc_dev.h — minimal DS3231 driver, direct Wire access.
 *
 * Replaces RTClib's RTC_DS3231 (which goes through an extra Adafruit_BusIO
 * layer) with a thin driver that only implements the handful of operations
 * this project actually uses, and — unlike RTClib — surfaces I2C failure
 * instead of silently leaving stale/undefined data in the output. Uses our
 * own DateTime (datetime.h — a straight port of RTClib's, pure calendar
 * math, no I2C) as the value type.
 *
 * Every call does exactly one bounded I2C transaction; the global
 * Wire.setTimeOut(50) (set in setup()) bounds how long that transaction can
 * block. Callers should hold xI2CMutex around these calls, same as before.
 */
#ifndef RTC_DEV_H
#define RTC_DEV_H

#include <Arduino.h>
#include "datetime.h"

class RtcDs3231 {
public:
    // Probes the DS3231 at its fixed I2C address. Returns false if it
    // doesn't ack.
    bool begin(void);

    // Reads the current time. `out` is left untouched on failure so a
    // caller can just skip updating its own copy on a failed read.
    bool now(DateTime &out);

    // Sets the time and clears the oscillator-stop (lost power) flag.
    bool adjust(const DateTime &dt);

    // Oscillator Stop Flag: true if the RTC lost power/stopped since last
    // set. `out` is left untouched on failure.
    bool lostPower(bool &out);

    // On-die temperature sensor, in the DS3231's native 0.25 C resolution.
    // `out` is left untouched on failure.
    bool getTemperature(float &out);

private:
    bool readRegister(uint8_t reg, uint8_t &val);
    bool writeRegister(uint8_t reg, uint8_t val);
};

// The one RtcDs3231 instance, defined in esp32-gen-ctl-ai-rtos.cpp — exposed
// here (2026-09-24) so i2c.cpp's I2CTask can use it directly, same pattern
// as oled_dev.h's `extern OledDev oled;`.
extern RtcDs3231 rtc;

#endif // RTC_DEV_H
