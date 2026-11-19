#include "fram_dev.h"
#include <Wire.h>

// Reserved "Get Device ID" broadcast address (Cypress/Infineon & Fujitsu
// FRAM datasheets) — not this chip's own address.
#define FRAM_META_ADDR 0x7C

#define FRAM_BLOCKSIZE 24  // matches the original library's chunk size

int FRAM32::begin(uint8_t address) {
    addr_ = address;
    Wire.beginTransmission(addr_);
    return (Wire.endTransmission() == 0) ? 0 : -1;
}

bool FRAM32::writeBlock(uint32_t memAddr, const uint8_t *obj, uint8_t size) {
    uint8_t addr = addr_;
    if (memAddr & 0xFFFE0000) return false;  // outside the 17-bit address space
    if (memAddr & 0x00010000) addr += 1;      // A16 bit -> second I2C address

    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(memAddr >> 8));
    Wire.write((uint8_t)(memAddr & 0xFF));
    for (uint8_t i = 0; i < size; i++) {
        Wire.write(obj[i]);
    }
    return Wire.endTransmission() == 0;
}

bool FRAM32::readBlock(uint32_t memAddr, uint8_t *obj, uint8_t size) {
    uint8_t addr = addr_;
    if (memAddr & 0xFFFE0000) return false;
    if (memAddr & 0x00010000) addr += 1;

    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(memAddr >> 8));
    Wire.write((uint8_t)(memAddr & 0xFF));
    // Plain STOP, not repeated start — see ads1115_dev.cpp's readRegister()
    // for why: Wire.setTimeOut() doesn't bound endTransmission(false).
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((int)addr, (int)size) != size) return false;
    for (uint8_t i = 0; i < size; i++) {
        if (!Wire.available()) return false;
        obj[i] = Wire.read();
    }
    return true;
}

bool FRAM32::write(uint32_t memAddr, const uint8_t *obj, uint16_t size) {
    const uint8_t *p = obj;
    while (size >= FRAM_BLOCKSIZE) {
        if (!writeBlock(memAddr, p, FRAM_BLOCKSIZE)) return false;
        memAddr += FRAM_BLOCKSIZE;
        p += FRAM_BLOCKSIZE;
        size -= FRAM_BLOCKSIZE;
    }
    if (size > 0) {
        if (!writeBlock(memAddr, p, (uint8_t)size)) return false;
    }
    return true;
}

bool FRAM32::read(uint32_t memAddr, uint8_t *obj, uint16_t size) {
    uint8_t *p = obj;
    while (size >= FRAM_BLOCKSIZE) {
        if (!readBlock(memAddr, p, FRAM_BLOCKSIZE)) return false;
        memAddr += FRAM_BLOCKSIZE;
        p += FRAM_BLOCKSIZE;
        size -= FRAM_BLOCKSIZE;
    }
    if (size > 0) {
        if (!readBlock(memAddr, p, (uint8_t)size)) return false;
    }
    return true;
}

bool FRAM32::getMetaData(uint32_t &out) {
    Wire.beginTransmission(FRAM_META_ADDR);
    Wire.write((uint8_t)(addr_ << 1));
    if (Wire.endTransmission() != 0) return false;  // plain STOP — see readBlock()
    if (Wire.requestFrom((int)FRAM_META_ADDR, 3) != 3) return false;

    uint32_t value = 0;
    for (uint8_t i = 0; i < 3; i++) {
        if (!Wire.available()) return false;
        value = (value << 8) | Wire.read();
    }
    out = value;
    return true;
}

uint16_t FRAM32::getManufacturerID(void) {
    uint32_t value;
    if (!getMetaData(value)) return 0;
    return (value >> 12) & 0x0FFF;
}

uint16_t FRAM32::getProductID(void) {
    uint32_t value;
    if (!getMetaData(value)) return 0;
    return value & 0x0FFF;
}

uint16_t FRAM32::getSize(void) {
    uint32_t value;
    if (!getMetaData(value)) return 0;
    // Cypress/Infineon density field only — this project's chip is
    // always the FM24V10 (128 KB), never a Fujitsu part.
    uint16_t density = (value >> 8) & 0x0F;
    return (1UL << density) * 8;  // KB
}
