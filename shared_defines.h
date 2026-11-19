#pragma once

// Trivial, no-linkage #define macros that used to be hand-duplicated across
// several translation units (idf/main/esp32-gen-ctl-ai-rtos.cpp,
// statemachine.cpp, oled_menu.cpp, wifi_setup.cpp) — each file only needed
// a couple of them, and since they're pure text substitution (no linkage to
// enforce a single definition the way a function/variable would), copying
// them looked cheap at the time. A shared header costs nothing a duplicate
// #define didn't already cost (identical expansion, same translation
// units), but it closes the one real risk duplication carries: nothing
// catches two copies silently drifting apart if only one is ever edited.

#include <Arduino.h>
#include "statemachine.h" // sendOutputCommand()/CMD_REG_ON, for RTOS_REG_ON

// millis()-based "now" used throughout timing logic (state machine timers,
// the main .cpp's own VBAT probe and WiFi reconnect backoff).
#define CURRENT_TIME millis()

// Smoothed<>/SmoothedSafe<> sample count for the ADC2 channels (adc_2_0/
// adc_2_1). ADC1_SMOOTH/RPM_SMOOTH and their _METHOD siblings
// (idf/main/esp32-gen-ctl-ai-rtos.cpp) are never referenced outside that
// file and stay defined there.
#define ADC2_SMOOTH 1

// Minimum fuel-level rise (L) that counts as "the pump is actually filling
// the tank" for the no-rise watchdog — matches the 1-decimal precision the
// smoothed reading is displayed/logged at, so ADC/smoothing jitter alone
// can't look like real progress and reset the timer.
#define FUEL_PUMP_RISE_HYSTERESIS_L 0.1f

// Default hostname/mDNS name and wifi.json "hostname" field fallback, used
// before any config is loaded (or if the field is missing).
#define WIFI_HOSTNAME_DEF "gen-ctl"

// RTOS-safe REG_ON for use from any task — posts to xOutputCommandQueue via
// sendOutputCommand() instead of touching shift_reg directly, so it's safe
// outside ShiftRegTask/StateMachineTask too (currently used from
// DisplayTask's OLED menu, mm_battery_select). RTOS_REG_OFF/RTOS_REG_SET
// stay defined locally in statemachine.cpp — nothing outside it currently
// needs them.
#define RTOS_REG_ON(REG) sendOutputCommand(CMD_REG_ON, REG, true)
