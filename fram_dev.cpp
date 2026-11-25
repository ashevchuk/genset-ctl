#include "fram_dev.h"
#include "i2c_bus.h"
#include <string.h>

// Reserved "Get Device ID" broadcast address (Cypress/Infineon & Fujitsu
// FRAM datasheets) — not this chip's own address.
#define FRAM_META_ADDR 0x7C

#define FRAM_BLOCKSIZE 24  // matches the original library's chunk size

int FRAM32::begin(uint8_t address) {
    addr_ = address;
    return i2c_bus_probe(addr_) ? 0 : -1;
}

bool FRAM32::writeBlock(uint32_t memAddr, const uint8_t *obj, uint8_t size) {
    uint8_t addr = addr_;
    if (memAddr & 0xFFFE0000) return false;  // outside the 17-bit address space
    if (memAddr & 0x00010000) addr += 1;      // A16 bit -> second I2C address

    uint8_t buf[2 + FRAM_BLOCKSIZE];
    if (size > FRAM_BLOCKSIZE) return false;
    buf[0] = (uint8_t)(memAddr >> 8);
    buf[1] = (uint8_t)(memAddr & 0xFF);
    memcpy(buf + 2, obj, size);
    return i2c_bus_write(addr, buf, 2 + size);
}

bool FRAM32::readBlock(uint32_t memAddr, uint8_t *obj, uint8_t size) {
    uint8_t addr = addr_;
    if (memAddr & 0xFFFE0000) return false;
    if (memAddr & 0x00010000) addr += 1;

    uint8_t memAddrBuf[2] = {(uint8_t)(memAddr >> 8), (uint8_t)(memAddr & 0xFF)};
    // Plain STOP, not repeated start — see ads1115_dev.cpp's readRegister().
    if (!i2c_bus_write(addr, memAddrBuf, sizeof(memAddrBuf))) return false;
    return i2c_bus_read(addr, obj, size);
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
    uint8_t devAddr = (uint8_t)(addr_ << 1);
    if (!i2c_bus_write(FRAM_META_ADDR, &devAddr, 1)) return false;  // plain STOP — see readBlock()
    uint8_t buf[3];
    if (!i2c_bus_read(FRAM_META_ADDR, buf, sizeof(buf))) return false;
    out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
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
