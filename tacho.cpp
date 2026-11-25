/*
 * tacho.cpp — Hardware frequency counter using ESP32 PCNT (ESP-IDF v5)
 *
 * PCNT counts both rising and falling edges on TACHO_PIN (matches original
 * attachInterrupt(CHANGE) behavior so RPM_DIV calibration stays unchanged).
 *
 * tacho_update() reads the accumulated count and elapsed time, then computes:
 *   edges/min  → tacho_get_rpm()   (same units as Tachometer::getRPM())
 *   edges/sec  → tacho_get_hz()    (same units as Tachometer::getHz())
 *
 * The PCNT accumulator never resets between update calls — we track delta
 * counts to avoid the race between read and clear.
 */

#include "tacho.h"
#include <stdint.h>
#include <stddef.h>
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

static pcnt_unit_handle_t s_unit       = NULL;
static int32_t            s_last_count = 0;
static int64_t            s_last_us    = 0;

// Results — volatile: written by tacho_update() (Core 1 SensorTask),
// read by tacho_get_*() which may be called from Core 0 (StateMachineTask).
// uint32_t and float are 32-bit aligned → atomic on Xtensa LX6.
static volatile uint32_t  s_rpm_raw = 0;
static volatile float     s_hz_raw  = 0.0f;

// ---------------------------------------------------------------------------

void tacho_setup() {
    // PCNT unit with large high_limit; accum_count accumulates across overflow
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit         = -1;      // counter only increases
    unit_cfg.high_limit        = 30000;
    unit_cfg.flags.accum_count = 1;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_unit));

    // Glitch filter: ignore pulses shorter than TACHO_GLITCH_NS (5 µs)
    // Rejects noise well above the ~2 kHz signal from the generator
    pcnt_glitch_filter_config_t flt = {};
    flt.max_glitch_ns = TACHO_GLITCH_NS;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_unit, &flt));

    // Single channel: count BOTH rising and falling edges (= CHANGE)
    // No control signal needed (level_gpio_num = -1)
    pcnt_chan_config_t ch_cfg = {};
    ch_cfg.edge_gpio_num  = TACHO_PIN;
    ch_cfg.level_gpio_num = -1;
    pcnt_channel_handle_t ch = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_unit, &ch_cfg, &ch));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,   // rising  → +1
        PCNT_CHANNEL_EDGE_ACTION_INCREASE)); // falling → +1
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // Watch point so accum_count fires its accumulation interrupt at limit
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_unit, 30000));

    ESP_ERROR_CHECK(pcnt_unit_enable(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_unit));

    s_last_count = 0;
    s_last_us    = esp_timer_get_time();
}

void tacho_update() {
    int raw = 0;
    pcnt_unit_get_count(s_unit, &raw);
    int64_t now_us = esp_timer_get_time();

    int32_t delta = raw - s_last_count;   // edges since last call
    int64_t dt_us = now_us - s_last_us;   // elapsed µs

    s_last_count = raw;
    s_last_us    = now_us;

    if (delta > 0 && dt_us > 0) {
        // edges/min — same units as Tachometer::getRPM()
        s_rpm_raw = (uint32_t)((int64_t)delta * 60000000LL / dt_us);
        // edges/sec — same units as Tachometer::getHz()
        s_hz_raw  = (float)delta * 1000000.0f / (float)dt_us;
    } else {
        s_rpm_raw = 0;
        s_hz_raw  = 0.0f;
    }
}

uint32_t tacho_get_rpm() { return s_rpm_raw; }
float    tacho_get_hz()  { return s_hz_raw;  }
