#include "rtc_dev.h"
#include "i2c_bus.h"

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
    return i2c_bus_probe(DS3231_ADDR);
}

bool RtcDs3231::readRegister(uint8_t reg, uint8_t &val) {
    // Plain STOP, not repeated start — see ads1115_dev.cpp's readRegister().
    if (!i2c_bus_write(DS3231_ADDR, &reg, 1)) return false;
    return i2c_bus_read(DS3231_ADDR, &val, 1);
}

bool RtcDs3231::writeRegister(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_bus_write(DS3231_ADDR, buf, sizeof(buf));
}

bool RtcDs3231::now(DateTime &out) {
    uint8_t reg = DS3231_REG_TIME;
    if (!i2c_bus_write(DS3231_ADDR, &reg, 1)) return false;  // plain STOP — see readRegister()
    uint8_t buf[7];
    if (!i2c_bus_read(DS3231_ADDR, buf, sizeof(buf))) return false;

    // buf: sec, min, hour, day-of-week, date, month(+century bit), year
    out = DateTime(bcd2bin(buf[6]) + 2000U, bcd2bin(buf[5] & 0x7F),
                   bcd2bin(buf[4]), bcd2bin(buf[2]), bcd2bin(buf[1]),
                   bcd2bin(buf[0] & 0x7F));
    return true;
}

bool RtcDs3231::adjust(const DateTime &dt) {
    uint8_t buf[8] = {
        DS3231_REG_TIME,
        bin2bcd(dt.second()),
        bin2bcd(dt.minute()),
        bin2bcd(dt.hour()),
        bin2bcd(dowToDs3231(dt.dayOfTheWeek())),
        bin2bcd(dt.day()),
        bin2bcd(dt.month()),
        bin2bcd(dt.year() - 2000U),
    };
    if (!i2c_bus_write(DS3231_ADDR, buf, sizeof(buf))) return false;

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
    uint8_t reg = DS3231_REG_TEMP;
    if (!i2c_bus_write(DS3231_ADDR, &reg, 1)) return false;  // plain STOP — see readRegister()
    uint8_t buf[2];
    if (!i2c_bus_read(DS3231_ADDR, buf, sizeof(buf))) return false;
    int8_t msb = (int8_t)buf[0];
    uint8_t lsb = buf[1];
    out = (float)msb + (lsb >> 6) * 0.25f;
    return true;
}
