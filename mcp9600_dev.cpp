#include "mcp9600_dev.h"
#include <Wire.h>

#define MCP9600_REG_HOT_JUNC  0x00
#define MCP9600_REG_STATUS    0x04
#define MCP9600_REG_DEVICE_ID 0x20

#define MCP9600_DEV_ID_UPPER  0x40    // expected upper byte of DEVICE_ID
#define MCP9600_RESOLUTION_C  0.0625f // deg C per LSB

// MCP9600 erratum: clock stretching sometimes makes the write phase report a
// spurious failure even though the register pointer latched correctly — the
// original library ignored endTransmission()'s result here too. A previous
// version of this file retried up to 4 times per call to survive that, but
// under FreeRTOS that turned a single real bus hiccup into a multi-second
// xI2CMutex hold (each attempt individually bounded by Wire.setTimeOut(),
// but 4 of them back to back across two register reads adds up) — starving
// every other task waiting on the same mutex. Single-attempt now: a failed
// read just returns false and the caller (SensorTask) logs it and moves on;
// see FLOG_EV_SENSOR_FAIL.
bool Mcp9600Dev::readRegister8(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.endTransmission();  // see comment above — result deliberately ignored
    if (Wire.requestFrom((int)addr_, 1) == 1) {
        val = Wire.read();
        return true;
    }
    return false;
}

bool Mcp9600Dev::readRegister16(uint8_t reg, uint16_t &val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.endTransmission();  // see readRegister8() — result deliberately ignored
    if (Wire.requestFrom((int)addr_, 2) == 2) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        val = ((uint16_t)hi << 8) | lo;
        return true;
    }
    return false;
}

bool Mcp9600Dev::writeRegister8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

bool Mcp9600Dev::isConnected(void) {
    Wire.beginTransmission(addr_);
    return Wire.endTransmission() == 0;
}

bool Mcp9600Dev::checkDeviceID(void) {
    uint16_t id;
    // Original library reads DEVICE_ID twice ("the first read doesn't seem
    // to work, but the second does") — preserved for parity; the first
    // result is intentionally discarded.
    readRegister16(MCP9600_REG_DEVICE_ID, id);
    if (!readRegister16(MCP9600_REG_DEVICE_ID, id)) return false;
    return (uint8_t)(id >> 8) == MCP9600_DEV_ID_UPPER;
}

bool Mcp9600Dev::begin(uint8_t address) {
    addr_ = address;
    return checkDeviceID();
}

bool Mcp9600Dev::available(void) {
    uint8_t status;
    if (!readRegister8(MCP9600_REG_STATUS, status)) return false;
    return (status >> 6) & 0x01;
}

bool Mcp9600Dev::getThermocoupleTemp(float &out) {
    uint16_t raw;
    if (!readRegister16(MCP9600_REG_HOT_JUNC, raw)) return false;
    out = (int16_t)raw * MCP9600_RESOLUTION_C;

    // Clear the data-ready bit (same as the original library). Best
    // effort — if this particular write fails, the next available() call
    // just won't see a fresh reading yet, which is harmless.
    uint8_t status;
    if (readRegister8(MCP9600_REG_STATUS, status)) {
        writeRegister8(MCP9600_REG_STATUS, status & ~0x40);
    }
    return true;
}
