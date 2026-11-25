/*
 * mcp9600_dev.h — minimal MCP9600 thermocouple driver, direct I2C access (i2c_bus.h).
 *
 * Replaces the SparkFun MCP9600 library with a thin driver exposing only
 * what this project uses: presence check, device-ID check, data-ready
 * flag, and hot-junction (thermocouple) temperature. Uses power-on-reset
 * register defaults (type K, no filtering) — same as the original library,
 * which never writes any config registers either.
 */
#ifndef MCP9600_DEV_H
#define MCP9600_DEV_H

#include <stdint.h>
#include <stddef.h>

class Mcp9600Dev {
public:
    // Sets the I2C address and checks the device ID (same as the original
    // library's begin() — which deliberately does NOT also call
    // isConnected(): a documented MCP9600 quirk is that it won't ACK a
    // second bus probe immediately after the first).
    bool begin(uint8_t address);

    bool isConnected(void);
    bool checkDeviceID(void);

    // True only if a fresh conversion is ready (and the read itself
    // succeeded — an I2C failure reads as "not available", a safe default
    // since callers already treat this as "skip until next check").
    bool available(void);

    // Hot junction (thermocouple) temperature in Celsius. `out` is left
    // untouched on I2C failure, so callers that pre-seed it with a
    // fallback default (e.g. "assume warm if sensor absent") keep that
    // fallback instead of getting a bogus 0.0 reading.
    bool getThermocoupleTemp(float &out);

private:
    bool readRegister8(uint8_t reg, uint8_t &val);
    bool readRegister16(uint8_t reg, uint16_t &val);
    bool writeRegister8(uint8_t reg, uint8_t val);

    uint8_t addr_ = 0;
};

// The one Mcp9600Dev instance, defined in esp32-gen-ctl-ai-rtos.cpp —
// exposed here (2026-09-24) so i2c.cpp's I2CTask can use it directly, same
// pattern as oled_dev.h's `extern OledDev oled;`.
extern Mcp9600Dev tempSensor;

#endif // MCP9600_DEV_H
