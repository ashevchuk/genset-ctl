#include "rtc_dev.h"
#include <Wire.h>

#define DS3231_ADDR        0x68
#define DS3231_REG_TIME    0x00
#define DS3231_REG_CONTROL 0x0E
#define DS3231_REG_STATUS  0x0F
#define DS3231_REG_TEMP    0x11

static inline uint8_t bcd2bin(uint8_t v) { return v - 6 * (v >> 4); }
static inline uint8_t bin2bcd(uint8_t v) { return v + 6 * (v / 10); }

// DateTime::dayOfTheWeek() returns 0=Sunday..6=Saturday; DS3231's day
// register wants 1=Monday..7=Sunday.
static inline uint8_t dowToDs3231(uint8_t d) { return d == 0 ? 7 : d; }

bool RtcDs3231::begin(void) {
    Wire.beginTransmission(DS3231_ADDR);
    return Wire.endTransmission() == 0;
}

bool RtcDs3231::readRegister(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(reg);
    // Plain STOP, not repeated start — see ads1115_dev.cpp's readRegister()
    // for why: Wire.setTimeOut() doesn't bound endTransmission(false).
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((int)DS3231_ADDR, 1) != 1) return false;
    if (!Wire.available()) return false;
    val = Wire.read();
    return true;
}

bool RtcDs3231::writeRegister(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool RtcDs3231::now(DateTime &out) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(DS3231_REG_TIME);
    if (Wire.endTransmission() != 0) return false;  // plain STOP — see readRegister()
    if (Wire.requestFrom((int)DS3231_ADDR, 7) != 7) return false;

    uint8_t buf[7];
    for (uint8_t i = 0; i < 7; i++) {
        if (!Wire.available()) return false;
        buf[i] = Wire.read();
    }

    // buf: sec, min, hour, day-of-week, date, month(+century bit), year
    out = DateTime(bcd2bin(buf[6]) + 2000U, bcd2bin(buf[5] & 0x7F),
                   bcd2bin(buf[4]), bcd2bin(buf[2]), bcd2bin(buf[1]),
                   bcd2bin(buf[0] & 0x7F));
    return true;
}

bool RtcDs3231::adjust(const DateTime &dt) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(DS3231_REG_TIME);
    Wire.write(bin2bcd(dt.second()));
    Wire.write(bin2bcd(dt.minute()));
    Wire.write(bin2bcd(dt.hour()));
    Wire.write(bin2bcd(dowToDs3231(dt.dayOfTheWeek())));
    Wire.write(bin2bcd(dt.day()));
    Wire.write(bin2bcd(dt.month()));
    Wire.write(bin2bcd(dt.year() - 2000U));
    if (Wire.endTransmission() != 0) return false;

    uint8_t status;
    if (!readRegister(DS3231_REG_STATUS, status)) return false;
    return writeRegister(DS3231_REG_STATUS, status & ~0x80);  // clear OSF
}

bool RtcDs3231::lostPower(bool &out) {
    uint8_t status;
    if (!readRegister(DS3231_REG_STATUS, status)) return false;
    out = (status >> 7) & 0x01;
    return true;
}

bool RtcDs3231::getTemperature(float &out) {
    Wire.beginTransmission(DS3231_ADDR);
    Wire.write(DS3231_REG_TEMP);
    if (Wire.endTransmission() != 0) return false;  // plain STOP — see readRegister()
    if (Wire.requestFrom((int)DS3231_ADDR, 2) != 2) return false;
    if (!Wire.available()) return false;
    int8_t msb = (int8_t)Wire.read();
    if (!Wire.available()) return false;
    uint8_t lsb = Wire.read();
    out = (float)msb + (lsb >> 6) * 0.25f;
    return true;
}
