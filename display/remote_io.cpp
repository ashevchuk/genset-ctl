#include "remote_io.h"

#define PIN_BTN_START         6
#define PIN_BTN_STOP          7
#define PIN_AUTOSTART_ARMED   9
#define PIN_AUTOSTART_TRIGGER 14
#define PIN_OUT_LINK_OK       15
#define PIN_OUT_GEN_RUNNING   16

#define DEBOUNCE_BUTTON_MS 40   // IO6/IO7 — human-operated momentary buttons
#define DEBOUNCE_LEVEL_MS  150  // IO9/IO14 — externally-driven level signals

struct DebouncedInput {
    uint8_t  pin;
    uint16_t stable_ms;      // required stable duration before accepting a new level
    bool     raw_active;     // last raw reading (active = pulled low)
    bool     stable_active;  // debounced/confirmed level
    uint32_t last_change_ms;
};

static DebouncedInput s_btn_start      = { PIN_BTN_START,         DEBOUNCE_BUTTON_MS, false, false, 0 };
static DebouncedInput s_btn_stop       = { PIN_BTN_STOP,          DEBOUNCE_BUTTON_MS, false, false, 0 };
static DebouncedInput s_autostart_arm  = { PIN_AUTOSTART_ARMED,   DEBOUNCE_LEVEL_MS,  false, false, 0 };
static DebouncedInput s_autostart_trig = { PIN_AUTOSTART_TRIGGER, DEBOUNCE_LEVEL_MS,  false, false, 0 };

// Samples the pin, updates the debounce bookkeeping, and returns the current
// debounced level (unchanged from the last call unless the raw reading has
// now held stable for `stable_ms`).
static bool debounce_update(DebouncedInput &in) {
    bool     raw_active = (digitalRead(in.pin) == LOW);
    uint32_t now        = millis();

    if (raw_active != in.raw_active) {
        in.raw_active     = raw_active;
        in.last_change_ms = now;
    }
    if (in.raw_active != in.stable_active &&
        now - in.last_change_ms >= in.stable_ms) {
        in.stable_active = in.raw_active;
    }
    return in.stable_active;
}

void remote_io_init(void) {
    pinMode(PIN_BTN_START,         INPUT_PULLUP);
    pinMode(PIN_BTN_STOP,          INPUT_PULLUP);
    pinMode(PIN_AUTOSTART_ARMED,   INPUT_PULLUP);
    pinMode(PIN_AUTOSTART_TRIGGER, INPUT_PULLUP);
    pinMode(PIN_OUT_LINK_OK,       OUTPUT);
    pinMode(PIN_OUT_GEN_RUNNING,   OUTPUT);
    digitalWrite(PIN_OUT_LINK_OK,     LOW);
    digitalWrite(PIN_OUT_GEN_RUNNING, LOW);

    // Seed each input's debounced state from a real read so the first
    // remote_io_poll() call never sees a spurious edge against the
    // structs' zero-initialized defaults.
    s_btn_start.raw_active      = s_btn_start.stable_active      = (digitalRead(PIN_BTN_START)         == LOW);
    s_btn_stop.raw_active       = s_btn_stop.stable_active       = (digitalRead(PIN_BTN_STOP)          == LOW);
    s_autostart_arm.raw_active  = s_autostart_arm.stable_active  = (digitalRead(PIN_AUTOSTART_ARMED)   == LOW);
    s_autostart_trig.raw_active = s_autostart_trig.stable_active = (digitalRead(PIN_AUTOSTART_TRIGGER) == LOW);
}

void remote_io_poll(const GenData &gen, bool link_ok) {
    // --- Start/Stop push buttons — fire once on the debounced falling edge ---
    bool start_was = s_btn_start.stable_active;
    bool start_now = debounce_update(s_btn_start);
    if (!start_was && start_now) {
        Serial.println("[REMOTE_IO] Start button pressed -> start_retry");
        ws_send("start_retry");
    }

    bool stop_was = s_btn_stop.stable_active;
    bool stop_now = debounce_update(s_btn_stop);
    if (!stop_was && stop_now) {
        Serial.println("[REMOTE_IO] Stop button pressed -> stop");
        ws_send("stop");
    }

    // --- Autostart trigger, gated by the armed switch ---
    bool armed = debounce_update(s_autostart_arm);

    bool trig_was = s_autostart_trig.stable_active;
    bool trig_now = debounce_update(s_autostart_trig);

    if (!trig_was && trig_now) {
        if (armed) {
            Serial.println("[REMOTE_IO] Autostart trigger grounded (armed) -> start_retry");
            ws_send("start_retry");
        } else {
            Serial.println("[REMOTE_IO] Autostart trigger grounded but not armed — ignored");
        }
    }
    if (trig_was && !trig_now) {
        // Unconditional — see remote_io.h's own note on this exact point:
        // fires regardless of the armed switch and regardless of who
        // actually started the generator.
        Serial.println("[REMOTE_IO] Autostart trigger released -> stop (unconditional)");
        ws_send("stop");
    }

    // --- Status outputs ---
    digitalWrite(PIN_OUT_LINK_OK, link_ok ? HIGH : LOW);
    if (gen.valid) {
        digitalWrite(PIN_OUT_GEN_RUNNING, gen.running ? HIGH : LOW);
    }
    // else: leave PIN_OUT_GEN_RUNNING exactly as it was — see remote_io.h.
}
