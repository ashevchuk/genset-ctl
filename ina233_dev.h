/*
 * ina233_dev.h — minimal INA233 driver, direct I2C access (i2c_bus.h).
 *
 * Replaces the INA233 library with a thin driver exposing only what this
 * project uses: bus voltage (PMBus READ_VIN, direct-format 1.25 mV/LSB).
 *
 * The original library's begin() was a no-op (just Wire.begin(), already
 * called globally in setup()) — it never actually checked the device was
 * present, and none of its read functions checked I2C return codes at all,
 * so a disconnected/wedged INA233 would silently return whatever garbage
 * was left in an uninitialized Wire read buffer. This version checks every
 * step and returns false on failure instead.
 *
 * getBusVoltage_V() also applies a fixed calibration offset (see
 * INA233_VBUS_OFFSET_V in ina233_dev.cpp) to correct a measured 0.096V
 * low bias versus a reference meter on this hardware.
 */
#ifndef INA233_DEV_H
#define INA233_DEV_H

#include <stdint.h>
#include <stddef.h>

class Ina233Dev {
public:
    explicit Ina233Dev(uint8_t addr) : addr_(addr) {}

    // Actually probes the device (the original library's begin() didn't).
    bool begin(void);

    // Bus voltage in volts. `out` is left untouched on I2C failure.
    // Two overloads: call sites in this project use both float and double.
    bool getBusVoltage_V(float &out);
    bool getBusVoltage_V(double &out);

private:
    bool readWord(uint8_t reg, uint16_t &val);
    uint8_t addr_;
};

// The one Ina233Dev instance, defined in esp32-gen-ctl-ai-rtos.cpp — exposed
// here (2026-09-24) so i2c.cpp's I2CTask can use it directly, same pattern
// as oled_dev.h's `extern OledDev oled;`.
extern Ina233Dev IC1;

#endif // INA233_DEV_H
