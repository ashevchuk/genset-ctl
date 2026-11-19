#pragma once

#include <stdint.h>
#include <stdbool.h>

/*
 * tacho.h — Hardware frequency counter for tachometer using ESP32 PCNT
 *
 * Replaces Tachometer library + TachoISR.
 * PCNT counts both rising and falling edges in hardware (CHANGE equivalent).
 * No ISR at all for counting — zero CPU load.
 *
 * Usage:
 *   1. Call tacho_setup() once in setup().
 *   2. Call tacho_update() periodically (e.g. from SensorTask).
 *      It computes RPM/Hz from pulses since last call.
 *   3. Call tacho_get_rpm() / tacho_get_hz() to read results.
 *
 * tacho_get_rpm() and tacho_get_hz() return values in the same units as
 * the old Tachometer::getRPM() / ::getHz(), so tachoRPM() / tachoFREQ()
 * wrappers in the main sketch work unchanged.
 *
 * Pin 35 is INPUT_ONLY on ESP32 — no software pull-up.
 * External pull-up required on PCB (already present).
 */

#define TACHO_PIN        35     // INPUT_ONLY, external pull-up on PCB
#define TACHO_GLITCH_NS  5000   // reject pulses narrower than 5 µs (≈200 kHz max)

// Initialise PCNT unit — call once from setup()
void tacho_setup();

// Compute RPM/Hz from pulses accumulated since last call.
// Must be called at a stable period (e.g. every PERIOD_SENSOR_MS).
void tacho_update();

// Returns edges/minute — same unit as Tachometer::getRPM().
// Divide by RPM_DIV in the caller to get actual RPM.
uint32_t tacho_get_rpm();

// Returns edges/second — same unit as Tachometer::getHz().
float tacho_get_hz();
