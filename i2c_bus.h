/*
 * i2c_bus.h — the single I2C master bus (ESP-IDF i2c_master driver).
 *
 * Mirrors what Arduino's Wire did on top of the same driver: port 0,
 * SDA 21 / SCL 22, 100 kHz, 50 ms per-transaction timeout, internal pull-ups,
 * one lazily-added device handle per address. Writes and reads are separate
 * transactions (plain STOP between them), matching how every driver here
 * already talked to its chip.
 *
 * I2CTask is still the only runtime user (see rtos_config.h); the internal
 * lock only keeps the bus consistent if that rule is ever broken.
 */
#ifndef I2C_BUS_H
#define I2C_BUS_H

#include <stddef.h>
#include <stdint.h>

bool i2c_bus_begin(void);

// Address-only transaction: true if a device ACKs.
bool i2c_bus_probe(uint8_t addr);

bool i2c_bus_write(uint8_t addr, const uint8_t *data, size_t len);
bool i2c_bus_read(uint8_t addr, uint8_t *data, size_t len);

#endif // I2C_BUS_H
