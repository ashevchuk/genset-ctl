#include "ads1115_dev.h"
#include <Wire.h>

#define ADS1115_REG_CONV      0x00
#define ADS1115_REG_CONFIG    0x01
#define ADS1115_REG_LO_THRESH 0x02
#define ADS1115_REG_HI_THRESH 0x03

// Fixed bits (everything except MUX, which is ORed in per call):
//   PGA   = +/-4096 mV      (0x0200)
//   MODE  = continuous      (0x0000)
//   DR    = 250 SPS         (0x00A0)
//   COMP_QUE = disable ALERT (0x0003)
#define ADS1115_BASE_CONFIG (0x0200 | 0x0000 | 0x00A0 | 0x0003)

bool Ads1115Dev::writeRegister(uint8_t reg, uint16_t val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write((uint8_t)(val >> 8));
    Wire.write((uint8_t)(val & 0xFF));
    return Wire.endTransmission() == 0;
}

bool Ads1115Dev::readRegister(uint8_t reg, uint16_t &val) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    // Plain STOP, not repeated start: Wire.setTimeOut() does not bound a
    // repeated-start (endTransmission(false)) transaction, so a slave that
    // clock-stretches indefinitely can wedge this whole task holding
    // xI2CMutex (see esp32-gen-ctl-ai-rtos.ino's upload-handler comment).
    // Releasing the bus between the pointer write and the data read keeps
    // both steps individually bounded by the 50ms timeout.
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((int)addr_, 2) != 2) return false;
    if (!Wire.available()) return false;
    uint8_t msb = Wire.read();
    if (!Wire.available()) return false;
    uint8_t lsb = Wire.read();
    val = ((uint16_t)msb << 8) | lsb;
    return true;
}

bool Ads1115Dev::init(void) {
    Wire.beginTransmission(addr_);
    if (Wire.endTransmission() != 0) return false;

    // Disable the comparator/ALERT pin outright (matches the original
    // library's init(), which writes these even though COMP_QUE=3 above
    // already disables it).
    if (!writeRegister(ADS1115_REG_LO_THRESH, 0x8000)) return false;
    if (!writeRegister(ADS1115_REG_HI_THRESH, 0x7FFF)) return false;

    return writeRegister(ADS1115_REG_CONFIG, ADS1115_BASE_CONFIG | ADS1115_COMP_0_GND);
}

bool Ads1115Dev::setCompareChannels(ADS1115_MUX mux) {
    if (!writeRegister(ADS1115_REG_CONFIG, ADS1115_BASE_CONFIG | mux)) return false;
    delay(8);  // two conversions at 250 SPS before the new channel settles
    return true;
}

float Ads1115Dev::getResult_mV(void) {
    uint16_t raw;
    if (!readRegister(ADS1115_REG_CONV, raw)) return 0.0f;
    return ((int16_t)raw * 1.0f / 32768.0f) * 4096.0f;
}
