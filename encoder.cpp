/*
 * encoder.cpp — Hardware quadrature decoder using ESP32 PCNT (ESP-IDF v5)
 *
 * PCNT peripheral counts quadrature edges entirely in hardware — zero CPU load,
 * no GPIOs blocked in software ISR, no FreeRTOS mutex calls from ISR context.
 *
 * 4x decoding uses two PCNT channels on the same unit:
 *   Channel 0: edge = A, ctrl = B
 *   Channel 1: edge = B, ctrl = A
 * Together they count all four state transitions per quadrature cycle.
 *
 * accum_count=1 enables software accumulation on overflow so the logical
 * range is unlimited (not bounded by the ±100 hardware limit).
 *
 * Button ISR is intentionally minimal: records press timestamp and sets a flag.
 * Debounce is checked on release using esp_timer_get_time().
 */

#include "encoder.h"
#include <stdint.h>
#include <stddef.h>
#include "driver/pulse_cnt.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_attr.h"

static pcnt_unit_handle_t s_pcnt_unit    = NULL;
static int32_t            s_last_position = 0;

// Written from ISR — must be volatile
static volatile bool     s_btn_clicked  = false;
static volatile uint32_t s_btn_press_ms = 0;

// ---------------------------------------------------------------------------
// Button ISR — minimal: no Serial, no mutex, no blocking calls
// ---------------------------------------------------------------------------
static void IRAM_ATTR encoder_btn_isr(void *arg) {
    (void)arg;
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (!gpio_get_level((gpio_num_t)ENCODER_PIN_BTN)) {
        // Falling edge: button pressed (active LOW)
        s_btn_press_ms = now_ms;
    } else {
        // Rising edge: button released — validate debounce window
        if ((now_ms - s_btn_press_ms) >= ENCODER_BTN_DEBOUNCE_MS) {
            s_btn_clicked = true;
        }
    }
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void encoder_setup() {
    // --- PCNT unit ----------------------------------------------------------
    // ±100 hardware window; accum_count lets the driver accumulate across
    // overflows so pcnt_unit_get_count() returns the true running total.
    pcnt_unit_config_t unit_cfg = {};
    unit_cfg.low_limit          = -100;
    unit_cfg.high_limit         =  100;
    unit_cfg.flags.accum_count  = 1;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_pcnt_unit));

    // Glitch filter: reject pulses shorter than 1 µs (mechanical bounce)
    pcnt_glitch_filter_config_t filter_cfg = {};
    filter_cfg.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_pcnt_unit, &filter_cfg));

    // --- Channel 0: edge = A, ctrl = B -------------------------------------
    //
    // Base edge action:  A rising  → INCREASE (+1)
    //                    A falling → DECREASE (-1)
    // Level modifier:    B = HIGH  → KEEP   (action unchanged)
    //                    B = LOW   → INVERSE (action flipped)
    //
    // Result (CW = positive):
    //   A↑  B=H → +1   (01→11 transition)
    //   A↑  B=L → -1   (CCW)
    //   A↓  B=H → -1   (CCW)
    //   A↓  B=L → +1   (10→00 transition)
    //
    pcnt_chan_config_t ch0_cfg = {};
    ch0_cfg.edge_gpio_num  = ENCODER_PIN_A;
    ch0_cfg.level_gpio_num = ENCODER_PIN_B;
    pcnt_channel_handle_t ch0 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &ch0_cfg, &ch0));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch0,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,    // A rising
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));  // A falling
    // API: set_level_action(chan, high_act, low_act)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch0,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,       // B = HIGH → keep
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));  // B = LOW  → invert

    // --- Channel 1: edge = B, ctrl = A (completes 4x decoding) ------------
    //
    // Base edge action:  B rising  → INCREASE (+1)
    //                    B falling → DECREASE (-1)
    // Level modifier:    A = HIGH  → INVERSE (action flipped)
    //                    A = LOW   → KEEP   (action unchanged)
    //
    // Result (CW = positive):
    //   B↑  A=L → +1   (00→01 transition)
    //   B↑  A=H → -1   (CCW)
    //   B↓  A=H → +1   (11→10 transition)
    //   B↓  A=L → -1   (CCW)
    //
    pcnt_chan_config_t ch1_cfg = {};
    ch1_cfg.edge_gpio_num  = ENCODER_PIN_B;
    ch1_cfg.level_gpio_num = ENCODER_PIN_A;
    pcnt_channel_handle_t ch1 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt_unit, &ch1_cfg, &ch1));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch1,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,    // B rising
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));  // B falling
    // API: set_level_action(chan, high_act, low_act)
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch1,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE,    // A = HIGH → invert
        PCNT_CHANNEL_LEVEL_ACTION_KEEP));     // A = LOW  → keep

    // Enable accumulation watch points at the hardware limits
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt_unit,  100));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(s_pcnt_unit, -100));

    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt_unit));
    s_last_position = 0;

    // Button: any-edge interrupt, minimal ISR only sets a flag
    gpio_config_t btn_cfg = {};
    btn_cfg.pin_bit_mask = 1ULL << ENCODER_PIN_BTN;
    btn_cfg.mode         = GPIO_MODE_INPUT;
    btn_cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    btn_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    btn_cfg.intr_type    = GPIO_INTR_ANYEDGE;
    ESP_ERROR_CHECK(gpio_config(&btn_cfg));
    // The ISR service is shared; it may already be installed by other code.
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)ENCODER_PIN_BTN, encoder_btn_isr, NULL));
}

int32_t encoder_get_position() {
    int raw = 0;
    pcnt_unit_get_count(s_pcnt_unit, &raw);
    return (int32_t)(raw / ENCODER_STEPS);
}

bool encoder_changed() {
    int32_t pos = encoder_get_position();
    if (pos != s_last_position) {
        s_last_position = pos;
        return true;
    }
    return false;
}

bool encoder_button_clicked() {
    if (s_btn_clicked) {
        s_btn_clicked = false;
        return true;
    }
    return false;
}

void encoder_reset() {
    pcnt_unit_clear_count(s_pcnt_unit);
    s_last_position = 0;
}
