#pragma once

#include <stdint.h>
#include <stddef.h>
#include "rtos_config.h"  // SHIFT_REG/SHIFT_REG_IN enums

// Physical shift-register I/O: SN74HC595N output register (relays/choke/
// buzzer/etc., SHIFT_REG enum) + CD4021BE input register (protection inputs,
// SHIFT_REG_IN enum) — see rtos_config.h for both enums. Extracted 2026-09-24
// from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code organization").
//
// shift_reg/shift_reg_in themselves are exposed here (not just behind
// functions) because the REG_*/IS_REG_* macros below — used pervasively
// throughout esp32-gen-ctl-ai-rtos.cpp's state machine and command handling,
// not just inside this module — read/write them directly. This is the same
// shape as g_state in rtos_config.h: was already a plain global read from
// many places before any file split, this just makes that legal across
// translation units too.
extern uint16_t shift_reg;
extern uint8_t shift_reg_in;

#define SET_SHIFT_REG(REG, STATE) \
  ((STATE) ? (shift_reg |= (uint16_t)(1u << (REG))) : (shift_reg &= (uint16_t)~(1u << (REG))))

#define REG_SYNC_STATE() reg_write()

#define REG_SET_STATE(REG, STATE) reg_set_state(REG, STATE)
#define REG_ON_STATE(REG) REG_SET_STATE(REG, true)
#define REG_OFF_STATE(REG) REG_SET_STATE(REG, false)
#define REG_TURN_STATE(REG) REG_SET_STATE(REG, (IS_REG_ON(REG) ? false : true))

#define SET_REG(REG, STATE) reg_set(REG, STATE)
#define REG_ON(REG) SET_REG(REG, true)
#define REG_OFF(REG) SET_REG(REG, false)
#define REG_TURN(REG) SET_REG(REG, (IS_REG_ON(REG) ? false : true))

#define IS_REG_ON(REG) (shift_reg & (1 << (REG)))
#define IS_REG_OFF(REG) (!IS_REG_ON(REG))

#define IS_REG_IN_ON(REG) (shift_reg_in & (1 << (REG)))
#define IS_REG_IN_OFF(REG) (!IS_REG_IN_ON(REG))

// Named-register lookup tables (for debug/status JSON that lists every
// register by name — see esp32-gen-ctl-ai-rtos.cpp's raw-WS-command handler
// and /metrics-adjacent dashboard state handler).
typedef struct ShiftRegItem {
  uint8_t reg;
  const char *name;
} ShiftRegItem;

typedef ShiftRegItem ShiftRegList[];

extern ShiftRegList ShiftRegs;
extern const uint8_t ShiftRegListLength;

extern ShiftRegList ShiftRegsIn;
extern const uint8_t ShiftRegInListLength;

// Low-level I/O. reg_set()/reg_set_state()/reg_write() are cross-task safe
// (xShiftRegMutex, recursive — see shiftreg.cpp's own comment); reg_read()/
// read_regs() likewise.
uint8_t reg_read(void);
uint8_t read_regs(void);
void setup_shift_reg_in(void);
void reg_write(void);
void reg_set_state(uint8_t reg, bool reg_state);
void reg_set(uint8_t reg, bool reg_state);
void setup_shift_reg(void);

// ShiftRegTask: owns all physical shift-register I/O (relay outputs +
// protection inputs), drains xOutputCommandQueue, and also drives
// buzzer_process() (buzzer.h) — the buzzer's own physical pin is one more
// shift-register bit (REG_BUZZER), so processing it under the same
// xShiftRegMutex pass as everything else avoids a second lock/unlock.
void ShiftRegTask(void *pvParameters);
