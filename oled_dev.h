/*
 * oled_dev.h — thin I2C transport for the SSD1306Ascii text/font engine.
 *
 * SSD1306Ascii (the base class, kept unmodified from the library — ~400
 * lines of font/glyph/cursor logic with zero I2C code) calls a single pure
 * virtual, writeDisplay(uint8_t b, uint8_t mode), as its only hardware
 * hook. The library's own transport, SSD1306AsciiWire, implements that in
 * ~20 lines — and never checks a single Wire return code. This replaces
 * just that transport, not the rendering engine (reimplementing font
 * rendering from scratch would be a large, high-risk rewrite for no real
 * I2C-robustness benefit — the hang risk is entirely in the transport).
 */
#ifndef OLED_DEV_H
#define OLED_DEV_H

#include <Arduino.h>
#include <Wire.h>
#include <SSD1306Ascii.h>

class OledDev : public SSD1306Ascii {
public:
    void begin(const DevType *dev, uint8_t i2cAddr);

    // Call once at the top of each render pass (before any print()/clear()/
    // setCursor()), so this frame gets its own chance regardless of whether
    // the previous frame hit a failure — see writeDisplay()'s early-bail.
    void beginFrame(void) { lastOk_ = true; nBuffered_ = 0; }

    // True if every I2C transaction since the last call succeeded. Note:
    // writeDisplay() is a void pure-virtual, called per byte (or per
    // batch — see below) from deep inside the base class's rendering
    // code, so a failure can't be propagated synchronously back to
    // whatever print()/setCursor()/clear() call triggered it. This flag
    // is the most visibility available without forking the base class.
    bool lastWriteOk(void) const { return lastOk_; }

protected:
    void writeDisplay(uint8_t b, uint8_t mode) override;

private:
    uint8_t addr_ = 0;
    uint8_t nBuffered_ = 0;  // pending bytes in the current batched transaction
    bool lastOk_ = true;
};

// The one OledDev instance, defined in esp32-gen-ctl-ai-rtos.cpp — exposed
// here (2026-09-24) so oled_menu.cpp's rendering code can use it directly,
// the same way every other consumer in the main .cpp already does.
extern OledDev oled;

#endif // OLED_DEV_H
