#include "oled_dev.h"
#include "i2c_bus.h"

void OledDev::begin(const DevType *dev, uint8_t i2cAddr) {
    addr_ = i2cAddr;
    nBuffered_ = 0;
    init(dev);  // SSD1306Ascii::init() — sends the panel init sequence via writeDisplay()
}

void OledDev::writeDisplay(uint8_t b, uint8_t mode) {
    // Early-bail: once this frame has hit a failed transmission, stop
    // issuing further I2C bytes for the rest of this render pass. Without
    // this, a genuinely unresponsive bus meant every remaining byte of a
    // full render (~150-200+ transactions: clear + cursor moves + glyph
    // data) still got attempted, each still paying up to Wire's 50ms
    // timeout before failing — turning one bad transaction into several
    // seconds of xI2CMutex held by DisplayTask. See beginFrame(), called
    // once per render pass, which resets this latch.
    if (!lastOk_) return;

    // Same batching as the original SSD1306AsciiWire (OPTIMIZE_I2C, the
    // library's default): group up to 16 consecutive RAM-mode bytes into
    // one I2C transaction instead of one transaction per byte — a real
    // difference for a full-screen redraw over I2C. Added on top: actually
    // check each transaction's result, which the original never did.
    if (nBuffered_ > kBatchMax || (nBuffered_ && mode == SSD1306_MODE_CMD)) {
        if (!flushBatch()) return;
    }
    if (nBuffered_ == 0) {
        len_ = 0;
        buf_[len_++] = (mode == SSD1306_MODE_CMD) ? 0x00 : 0x40;
    }
    buf_[len_++] = b;
    if (mode == SSD1306_MODE_RAM_BUF) {
        nBuffered_++;
    } else {
        flushBatch();
    }
}

bool OledDev::flushBatch(void) {
    nBuffered_ = 0;
    if (!i2c_bus_write(addr_, buf_, len_)) {
        lastOk_ = false;
        return false;
    }
    return true;
}
