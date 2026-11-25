#pragma once

#include <stdint.h>
#include <stddef.h>
#include "rtos_config.h"

// Generator state machine (StateMachineTask, switch_STATE_*/process_STATE_*)
// — see CLAUDE.md's "Generator state machine" section. Extracted from
// esp32-gen-ctl-ai-rtos.cpp 2026-09-24 in two passes (see
// [[main-cpp-modularization]] project memory): pass 1 moved the battery/
// choke/PSU-backup/FRAM-transition helper cluster these functions call;
// pass 2 (this header's current state) moved StateMachineTask itself and
// the switch_STATE_*/process_STATE_* handlers here too, so everything below
// is now this module's real, permanent public surface — not a mid-migration
// shim.

// ----------------------------------------------------------------------------
// Battery/choke/PSU-backup helpers — called from switch_STATE_*/
// process_STATE_*/process_state_check(), now also in this module.
// ----------------------------------------------------------------------------

// Forces the battery online (REG_BATT_FORCE) if not already sensed, settle-
// polls the INA233 for a stable reading, then restores the prior state.
void measure_bat_voltage();

// The choke solenoid needs battery power to hold closed — force it on/off
// around a cold-start choke sequence, independent of any other reason the
// battery might already be forced on.
void choke_ensure_battery(void);
void choke_release_battery(void);

// Configurable (BatteryConfig_t.psu_loss_backup_enabled): while idle, forces
// the battery on if both PSU_SENSE and GEN_SENSE are lost, releases it once
// either returns. Call once per StateMachineTask iteration from
// process_STATE_IDLE() only — STARTING/STARTED/RUNNING have their own
// REG_BATT_FORCE lifecycle this must not fight with.
void psu_backup_check(void);

// Derives FRAM journal events (START_OK/START_FAIL/EMERGENCY_STOP/STOPPED)
// from a state transition — call from switch_state() right before the state
// actually changes, while old_s still reflects what the engine was doing.
void flog_on_transition(state old_s, state new_s);

// Charging-session tracking (start time + active flag, used for
// FLOG_EV_CHARGE_START/_STOP duration/reason).
void flog_charge_started(bool auto_started);
void flog_charge_stopped(uint8_t reason);

// True if any hardware protection input or configured software temperature
// limit currently forbids starting/running.
bool safety_start_inhibited(void);

// Cached head-temp reads (g_state.sensors, refreshed by I2CTask — no direct
// I2C access from here). flog_head_temp_or_warm() falls back to 99.0 (i.e.
// "warm", no choke) if the sensor is absent/invalid rather than trusting a
// stale/never-set 0.0, which would otherwise read as "cold" for the wrong
// reason.
float flog_head_temp(void);
float flog_head_temp_or_warm(void);

// ----------------------------------------------------------------------------
// StateMachineTask itself.
// ----------------------------------------------------------------------------
void StateMachineTask(void *pvParameters);

// ----------------------------------------------------------------------------
// Raw state-transition machinery. runtimeState/prevState are non-static
// (extern here) and the STATE()/IS_CURRENT_STATE() macros are exported —
// not because anything outside this module should be driving the state
// machine directly, but because the main .cpp's is_not_working() needs to
// read the current state the same way the state machine's own handlers do. Everything else in
// this file (the switch_STATE_*/process_STATE_* handlers, switch_state()'s
// own definition) is genuinely private to statemachine.cpp.
// ----------------------------------------------------------------------------
extern state runtimeState;
extern state prevState;

bool switch_state(const state new_state, const bool update_old);

// Genuinely public: oled_menu.cpp's RTOS_REG_ON (shared_defines.h) posts
// register commands from outside this module via these — they just
// xQueueSend(xOutputCommandQueue, ...), safe to call from any task. Was
// missing from this header until 2026-09-25 (see shared_defines.h): the
// only reason oled_menu.cpp used to carry its own hand-written forward
// declaration of sendOutputCommand() instead of a real #include.
void sendOutputCommand(OutputCommandType_t type, uint8_t reg, bool state);
void sendSyncCommand(uint16_t full_state);

// Still defined (non-static) in the main .cpp: is_not_working() and its
// siblings is_running()/is_stopped() are also used by report.cpp,
// esp_now_link.cpp and mqtt.cpp.
// readBatPeriod is likewise a main .cpp global (see its own comment there —
// duplicating its initializer here would be a cross-TU static-init-order
// hazard), just read/written by the state machine handlers in this file.
bool is_running(void);
bool is_stopped(void);
bool is_not_working(void);
extern unsigned long long readBatPeriod;

#define PREV_STATE prevState
#define SET_PREV_STATE(old_state) prevState = (old_state)
#define CURRENT_STATE runtimeState
#define SET_CURRENT_STATE(new_state) runtimeState = new_state
#define IS_NEW_STATE(new_state) (bool)(CURRENT_STATE != (new_state))
#define IS_CURRENT_STATE(check_state) (bool)(CURRENT_STATE == (check_state))

#define MAKE_PROCESS_STATE_FN_NAME(x) process_##x()
#define PROCESS_STATE(x) MAKE_PROCESS_STATE_FN_NAME(x)

#define MAKE_FN_NAME(x) switch_##x()
#define SWITCH_STATE(x) MAKE_FN_NAME(x)

// switch_state() already checks IS_NEW_STATE() itself and returns false
// without touching the switch/case body if new_state isn't actually new, so
// this doesn't need to re-check it via a short-circuited && (a boolean
// expression relied on for control flow, at every STATE(x); call site that
// discards the result) — a plain function call is the same behavior with
// none of that.
#define STATE(new_state) switch_state(new_state, true)
