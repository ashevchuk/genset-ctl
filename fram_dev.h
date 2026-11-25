/*
 * fram_dev.h — minimal FM24V10 FRAM driver, direct I2C access (i2c_bus.h).
 *
 * Replaces the FRAM_I2C library's FRAM32 class (which this project uses
 * specifically because the FM24V10 is 128 KB — a 17-bit address space,
 * addressed as two 64 KB pages selected by the A16 bit, which the base
 * FRAM class can't reach). Kept the class name FRAM32 since fram_log.h/
 * fram_log.cpp reference that type directly — only the internals change.
 *
 * Unlike the original library, write()/read() (and writeObject/readObject)
 * return bool and check every I2C step (endTransmission return code,
 * requestFrom count, available() before each read) instead of silently
 * assuming success — the original tracked an internal error code but never
 * actually surfaced it through these calls.
 */
#ifndef FRAM_DEV_H
#define FRAM_DEV_H

#include <stdint.h>
#include <stddef.h>

class FRAM32 {
public:
    // Returns 0 on success (matches the original library's convention —
    // both call sites in the .ino check `== 0` / `!= 0`), -1 on failure.
    int begin(uint8_t address = 0x50);

    bool write(uint32_t memAddr, const uint8_t *obj, uint16_t size);
    bool read(uint32_t memAddr, uint8_t *obj, uint16_t size);

    template <class T> bool writeObject(uint32_t memAddr, T &obj) {
        return write(memAddr, (const uint8_t *)&obj, sizeof(obj));
    }
    template <class T> bool readObject(uint32_t memAddr, T &obj) {
        return read(memAddr, (uint8_t *)&obj, sizeof(obj));
    }

    // Diagnostic-only (setup() boot printout); 0 on failure, same as the
    // original library.
    uint16_t getManufacturerID(void);
    uint16_t getProductID(void);
    uint16_t getSize(void);  // KB

private:
    bool writeBlock(uint32_t memAddr, const uint8_t *obj, uint8_t size);
    bool readBlock(uint32_t memAddr, uint8_t *obj, uint8_t size);
    bool getMetaData(uint32_t &out);

    uint8_t addr_ = 0x50;
};

#endif // FRAM_DEV_H
