/*
 * ads1115_dev.h — minimal ADS1115 driver, direct Wire access.
 *
 * Replaces ADS1115_WE with a thin driver fixed at the one configuration
 * this project actually uses on both chips: +/-4096 mV range, 250 SPS,
 * continuous mode, comparator/ALERT disabled. Only the two operations
 * actually called (switch channel, read result) are exposed.
 *
 * Kept the original enum name/values (ADS1115_MUX, ADS1115_COMP_x_GND) so
 * call sites throughout the .ino didn't need to change — these are just
 * mnemonic names for the ADS1115 config register's MUX field, not
 * something the external library "owns".
 */
#ifndef ADS1115_DEV_H
#define ADS1115_DEV_H

#include <Arduino.h>

enum ADS1115_MUX : uint16_t {
    ADS1115_COMP_0_GND = 0x4000,
    ADS1115_COMP_1_GND = 0x5000,
    ADS1115_COMP_2_GND = 0x6000,
    ADS1115_COMP_3_GND = 0x7000,
};

class Ads1115Dev {
public:
    explicit Ads1115Dev(uint8_t addr) : addr_(addr) {}

    // Probes the device and applies the fixed configuration above (starts
    // on channel 0 vs GND). Returns false if it doesn't ack.
    bool init(void);

    // Switches the active input channel. In continuous mode the device
    // needs two conversions at the new channel before a read is valid —
    // same as the original library, a real blocking delay() (~8ms at our
    // fixed 250 SPS), not an I2C wait.
    bool setCompareChannels(ADS1115_MUX mux);

    // Reads the latest conversion, scaled to millivolts at the fixed
    // +/-4096mV range. Returns 0.0 on I2C failure — matching the original
    // library's de-facto behavior on failure, deliberately not NAN: several
    // values from this driver go straight into raw (unquoted) JSON via
    // serialized(), and NaN there would produce invalid JSON and break
    // every WS/HTTP client's parser.
    float getResult_mV(void);

private:
    bool writeRegister(uint8_t reg, uint16_t val);
    bool readRegister(uint8_t reg, uint16_t &val);

    uint8_t addr_;
};

#endif // ADS1115_DEV_H
