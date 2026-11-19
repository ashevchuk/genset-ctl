#pragma once

#include <Arduino.h>

// Buzzer default timing constants (in milliseconds)
#define BUZZER_DEFAULT_DURATION_MS   100
#define BUZZER_DEFAULT_PAUSE_MS      100
#define BUZZER_ALARM_DURATION_MS     500
#define BUZZER_ALARM_PAUSE_MS        500
#define BUZZER_SHORT_BEEP_MS         50
#define BUZZER_LONG_BEEP_MS          300

// Non-blocking buzzer control API. All functions are asynchronous (post to
// xBuzzerCommandQueue, see rtos_config.h) and processed by buzzer_process(),
// called once per tick from ShiftRegTask.
//
// Usage:
//   buzzerBeepOnce();                 // single beep, default duration
//   buzzerBeepOnce(200);              // single 200ms beep
//   buzzerBeep(100, 3, 50);           // 3 beeps: 100ms ON, 50ms pause
//   buzzerBeepDouble(); / buzzerBeepTriple();
//   buzzerBeepInfinite(500, 500);     // continuous beeping
//   buzzerAlarm();                    // alarm pattern (BUZZER_ALARM_* timing)
//   buzzerStop();                     // stop any active pattern
//   bool active = buzzerIsActive();
//
// Note: for synchronous (blocking) buzzer control, use direct REG_ON/REG_OFF
// with delay() instead — these are non-blocking.
void buzzerBeep(uint16_t duration_ms, uint8_t count, uint16_t pause_ms);
void buzzerBeepInfinite(uint16_t duration_ms, uint16_t pause_ms);
void buzzerStop(void);
bool buzzerIsActive(void);
void buzzerBeepOnce(uint16_t duration_ms);
void buzzerBeepOnce(void);
void buzzerBeepDouble(uint16_t duration_ms, uint16_t pause_ms);
void buzzerBeepDouble(void);
void buzzerBeepTriple(uint16_t duration_ms, uint16_t pause_ms);
void buzzerBeepTriple(void);
void buzzerAlarm(uint16_t duration_ms, uint16_t pause_ms);
void buzzerAlarm(void);

// Drains xBuzzerCommandQueue and advances the current beep pattern by one
// tick. Call once per ShiftRegTask iteration; must be called with
// xShiftRegMutex already held, same convention as every other shift-reg
// write in that task.
void buzzer_process(void);

// Implemented in esp32-gen-ctl-ai-rtos.cpp, which still owns shift_reg/
// SET_SHIFT_REG/reg_write() — buzzer.cpp has no business touching the shift
// register directly. Toggles REG_BUZZER and flushes the shift register.
void shiftreg_write_buzzer(bool on);
