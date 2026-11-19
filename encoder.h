#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * encoder.h — Hardware quadrature decoder using ESP32 PCNT peripheral
 *
 * Replaces AiEsp32RotaryEncoder library.
 * No ISR for counting: hardware does it entirely.
 * Button uses a minimal ISR that only sets a volatile flag (no Serial, no mutex).
 *
 * Pins 36 (B) and 39 (A) are INPUT_ONLY on ESP32 — no software pull-up possible.
 * External pull-ups required on PCB (already present based on original wiring).
 *
 * 4x decoding: counts all 4 quadrature edges → 4 PCNT counts per physical detent.
 * encoder_get_position() divides by ENCODER_STEPS to return logical clicks.
 */

// Pins — match .ino definitions
#define ENCODER_PIN_A    39   // SENSOR_VN  (INPUT_ONLY, ext pull-up required)
#define ENCODER_PIN_B    36   // SENSOR_VP  (INPUT_ONLY, ext pull-up required)
#define ENCODER_PIN_BTN  34   // INPUT_ONLY, ext pull-up required, active LOW

// 4x quadrature: 4 raw PCNT counts = 1 physical detent
#define ENCODER_STEPS    4

// Button debounce window (ms)
#define ENCODER_BTN_DEBOUNCE_MS  50

// Initialise PCNT unit + channels + button interrupt
void encoder_setup();

// Logical position: raw_count / ENCODER_STEPS
int32_t encoder_get_position();

// True if position changed since last call (clears internal reference)
bool encoder_changed();

// True (and clears flag) if a debounced button click was detected
bool encoder_button_clicked();

// Reset accumulated count to zero
void encoder_reset();
