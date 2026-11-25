#include "ads1115_dev.h"
#include "i2c_bus.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

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
    uint8_t buf[3] = {reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF)};
    return i2c_bus_write(addr_, buf, sizeof(buf));
}

bool Ads1115Dev::readRegister(uint8_t reg, uint16_t &val) {
    // Plain STOP, not repeated start, between the pointer write and the data
    // read: each step is its own transaction, individually bounded by the
    // bus timeout (i2c_bus.cpp), so a slave that clock-stretches
    // indefinitely can only stall one bounded step.
    if (!i2c_bus_write(addr_, &reg, 1)) return false;
    uint8_t buf[2];
    if (!i2c_bus_read(addr_, buf, sizeof(buf))) return false;
    val = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

bool Ads1115Dev::init(void) {
    if (!i2c_bus_probe(addr_)) return false;

    // Disable the comparator/ALERT pin outright (matches the original
    // library's init(), which writes these even though COMP_QUE=3 above
    // already disables it).
    if (!writeRegister(ADS1115_REG_LO_THRESH, 0x8000)) return false;
    if (!writeRegister(ADS1115_REG_HI_THRESH, 0x7FFF)) return false;

    return writeRegister(ADS1115_REG_CONFIG, ADS1115_BASE_CONFIG | ADS1115_COMP_0_GND);
}

bool Ads1115Dev::setCompareChannels(ADS1115_MUX mux) {
    if (!writeRegister(ADS1115_REG_CONFIG, ADS1115_BASE_CONFIG | mux)) return false;
    vTaskDelay(pdMS_TO_TICKS(8));  // two conversions at 250 SPS before the new channel settles
    return true;
}

float Ads1115Dev::getResult_mV(void) {
    uint16_t raw;
    if (!readRegister(ADS1115_REG_CONV, raw)) return 0.0f;
    return ((int16_t)raw * 1.0f / 32768.0f) * 4096.0f;
}
