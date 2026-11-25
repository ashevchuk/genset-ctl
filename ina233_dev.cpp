#include "ina233_dev.h"
#include "i2c_bus.h"

#define INA233_READ_VIN 0x88  // PMBus command, direct format, 1.25 mV/LSB

// Calibration offset: this INA233 reads battery voltage 0.096V low versus
// a reference meter. Added back here rather than at each call site so it
// applies uniformly everywhere getBusVoltage_V() is used.
#define INA233_VBUS_OFFSET_V 0.096f

bool Ina233Dev::begin(void) {
    return i2c_bus_probe(addr_);
}

bool Ina233Dev::readWord(uint8_t reg, uint16_t &val) {
    // Plain STOP, not repeated start — see ads1115_dev.cpp's readRegister().
    if (!i2c_bus_write(addr_, &reg, 1)) return false;
    uint8_t buf[2];
    if (!i2c_bus_read(addr_, buf, sizeof(buf))) return false;
    val = ((uint16_t)buf[1] << 8) | buf[0];  // PMBus word reads are LSB first
    return true;
}

bool Ina233Dev::getBusVoltage_V(float &out) {
    uint16_t raw;
    if (!readWord(INA233_READ_VIN, raw)) return false;
    // Direct-format VBUS: Y = (X * 10^-R - b) / m, with m=8, R=2, b=0 —
    // i.e. 1.25 mV/LSB. Kept as an unsigned value (matches the original
    // library's own, likely-unintentional, unsigned treatment of this
    // reading — VIN is unipolar in this application so it's never
    // exercised in practice).
    out = raw / 800.0f + INA233_VBUS_OFFSET_V;
    return true;
}

bool Ina233Dev::getBusVoltage_V(double &out) {
    float v;
    if (!getBusVoltage_V(v)) return false;
    out = v;
    return true;
}
