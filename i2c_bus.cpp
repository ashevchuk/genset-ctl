#include "i2c_bus.h"
#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#define I2C_BUS_PORT       0
#define I2C_BUS_SDA        21
#define I2C_BUS_SCL        22
#define I2C_BUS_CLOCK_HZ   100000
#define I2C_BUS_TIMEOUT_MS 50  // bounds each transaction so a NACK/stuck slave can't spin the ISR into the Interrupt WDT
#define I2C_ADDR_COUNT     128

static i2c_master_bus_handle_t s_bus = NULL;
static i2c_master_dev_handle_t s_dev[I2C_ADDR_COUNT] = {};
static SemaphoreHandle_t s_lock = NULL;

bool i2c_bus_begin(void) {
    i2c_master_bus_config_t cfg = {};
    cfg.i2c_port = I2C_BUS_PORT;
    cfg.sda_io_num = (gpio_num_t)I2C_BUS_SDA;
    cfg.scl_io_num = (gpio_num_t)I2C_BUS_SCL;
    cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = 1;
    if (i2c_new_master_bus(&cfg, &s_bus) != ESP_OK) return false;
    s_lock = xSemaphoreCreateMutex();
    return s_lock != NULL;
}

static i2c_master_dev_handle_t device(uint8_t addr) {
    if (s_dev[addr] == NULL) {
        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = addr;
        cfg.scl_speed_hz = I2C_BUS_CLOCK_HZ;
        i2c_master_bus_add_device(s_bus, &cfg, &s_dev[addr]);
    }
    return s_dev[addr];
}

static bool lock(void) {
    return s_lock != NULL && xSemaphoreTake(s_lock, portMAX_DELAY) == pdTRUE;
}

bool i2c_bus_probe(uint8_t addr) {
    if (addr >= I2C_ADDR_COUNT || !lock()) return false;
    bool ok = i2c_master_probe(s_bus, addr, I2C_BUS_TIMEOUT_MS) == ESP_OK;
    xSemaphoreGive(s_lock);
    return ok;
}

bool i2c_bus_write(uint8_t addr, const uint8_t *data, size_t len) {
    if (addr >= I2C_ADDR_COUNT || !lock()) return false;
    i2c_master_dev_handle_t dev = device(addr);
    bool ok = dev != NULL && i2c_master_transmit(dev, data, len, I2C_BUS_TIMEOUT_MS) == ESP_OK;
    xSemaphoreGive(s_lock);
    return ok;
}

bool i2c_bus_read(uint8_t addr, uint8_t *data, size_t len) {
    if (addr >= I2C_ADDR_COUNT || !lock()) return false;
    i2c_master_dev_handle_t dev = device(addr);
    bool ok = dev != NULL && i2c_master_receive(dev, data, len, I2C_BUS_TIMEOUT_MS) == ESP_OK;
    xSemaphoreGive(s_lock);
    return ok;
}
