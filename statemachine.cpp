/*
 * statemachine.cpp — generator state machine. See statemachine.h.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp in two passes (see
 * CLAUDE.md's "Code organization" and [[main-cpp-modularization]] project
 * memory): pass 1 moved the battery/choke/PSU-backup/FRAM-transition helper
 * cluster; pass 2 (this file's current state) moved StateMachineTask itself
 * and the switch_STATE_x/process_STATE_x handlers here too, so the timing/
 * flag statics below are now genuinely private to this module (they were a
 * temporary extern-in-statemachine.h shim during pass 1, when the main .cpp
 * still read/wrote them directly — no longer needed now that every reader
 * moved here as well).
 */

#include "statemachine.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "config.h"
#include "shiftreg.h"
#include "smoothed_safe.h"
#include "debug.h"
#include "ina233_dev.h"
#include "buzzer.h"
#include <math.h>
#include "shared_defines.h" // CURRENT_TIME/ADC2_SMOOTH/FUEL_PUMP_RISE_HYSTERESIS_L
#include <esp_task_wdt.h>
#include "uptime.h"

// ---- externs into esp32-gen-ctl-ai-rtos.cpp's own globals ------------------
extern bool ina233_present;   // set in setup(), avoids repeated NACK if sensor absent
extern SmoothedSafe<double> adc_1_0;
extern SmoothedSafe<double> adc_1_1;
extern SmoothedSafe<double> adc_1_2;
extern SmoothedSafe<double> adc_2_0;
extern SmoothedSafe<double> adc_2_1;
double fuel_level_from_adc(double adc_value);

// The raw state itself — non-static (statemachine.h's own extern), since
// is_not_working()/start_working()/start_retry_working()/stop_working() in
// the main .cpp still read/drive it via the STATE()/IS_CURRENT_STATE()
// macros (also declared in statemachine.h).
state runtimeState = STATE_INIT;
state prevState = STATE_INIT;

// Everything below is genuinely private now — no reader left outside this
// file after pass 2.
static bool choke_forced_battery = false;
static bool psu_backup_engaged = false;
static uint32_t flog_last_stop_req_ms = 0;
static bool flog_pending_temp_limit_stop = false;

static unsigned long long afterRunPeriodTime = 0;
static bool               afterRunPeriod = false;
static unsigned long long readBatTime = 0;
static unsigned long long startTime = 0;
static unsigned long long startRetryTime = 0;
static unsigned long long startTry = 0;
static unsigned long long stoppingTime = 0;
static unsigned long long loadOffTime = 0;
static bool               stoppingToRetry = false;
static unsigned long long runningTime = 0;
static bool     battery_dependency_forced = false;
static uint32_t fuel_pump_last_rise_ms = 0;
static float    fuel_pump_last_rise_level = 0.0f;
static bool     charging = false;
static uint32_t batt_low_since_ms = 0;
static uint32_t charge_pending_since_ms = 0;

// Tuning for the settle-polling loop in measure_bat_voltage() below.
#define BAT_FORCE_STABLE_INTERVAL_MS 20
#define BAT_FORCE_STABLE_TOLERANCE_V 0.05
#define BAT_FORCE_STABLE_MAX_MS 150

// On-demand INA233 read for measure_bat_voltage()'s settle-poll loop below —
// needs a much tighter cadence (BAT_FORCE_STABLE_INTERVAL_MS) than the
// passive ~500ms sensor cache, so it asks I2CTask directly via the job
// queue instead. Only ever called from StateMachineTask, so one static
// semaphore for this call site is enough (calls are already sequential).
static bool job_read_ina233_now(void *arg) {
    return IC1.getBusVoltage_V(*(double *)arg);
}
static bool read_ina233_now(double *voltage) {
    static SemaphoreHandle_t s_done = NULL;
    if (s_done == NULL) s_done = xSemaphoreCreateBinary();
    // Static, not the caller's own pointer directly: if i2c_run_on_owner()
    // times out (job queued behind a long one, e.g. a FRAM wipe), the
    // caller may already have returned by the time I2CTask actually runs
    // it — writing into the caller's stack slot at that point would
    // corrupt whatever now lives at that address. Static storage stays
    // valid regardless; only copied into *voltage on confirmed success,
    // matching the old code's behavior of leaving voltage untouched
    // (keeping its prior value) whenever the read didn't happen.
    static double s_voltage;
    if (!i2c_run_on_owner(job_read_ina233_now, &s_voltage, s_done, pdMS_TO_TICKS(200))) return false;
    *voltage = s_voltage;
    return true;
}

void measure_bat_voltage() {
  if (!ina233_present) {
    adc_2_1.add(12.4f);
    readBatTime = CURRENT_TIME;
    SDEBUG_CHARGING("[BAT] INA233 not present — NO_CHARGING scenario, simulated voltage 12.4V");
    return;
  }

  double voltage = 0.0;

  read_regs();

  bool reg_bat_force_off = IS_REG_OFF(REG_BATT_FORCE);

  SDEBUG_CHARGING("[BAT] measure_bat_voltage: BAT_SENSE=", IS_REG_IN_ON(REG_IN_BAT_SENSE),
         " BATT_FORCE=", IS_REG_ON(REG_BATT_FORCE), " (was_off=", reg_bat_force_off, ")");

  // BAT
  if (IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
    read_ina233_now(&voltage);
    SDEBUG_CHARGING("[BAT] measure_bat_voltage: battery already online, raw=", dbg_fixed(voltage, 3), "V");
  } else {
    if (IS_REG_OFF(REG_BATT_FORCE)) {
      SDEBUG_CHARGING("[BAT] measure_bat_voltage: BAT_SENSE=OFF, enabling BATT_FORCE to connect battery for measurement");
      REG_ON(REG_BATT_FORCE);

      unsigned long long waitTime = CURRENT_TIME;

      while ((CURRENT_TIME - waitTime <= 1000) &&
             (IS_REG_IN_OFF(REG_IN_BAT_SENSE))) {
        read_regs();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1));
      };

      if (IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
        // A fixed 20ms settle delay before a single read wasn't always
        // enough (confirmed live: occasional missed/unsettled readings).
        // Instead of guessing a bigger fixed number, poll: read repeatedly
        // BAT_FORCE_STABLE_INTERVAL_MS apart until two consecutive samples
        // agree within BAT_FORCE_STABLE_TOLERANCE_V, capped at
        // BAT_FORCE_STABLE_MAX_MS total — falls back to the last sample
        // read if it never settles (a possibly-slightly-off reading beats
        // no reading at all).
        unsigned long long settleStart = CURRENT_TIME;
        double prevVoltage = 0.0;
        bool haveFirst = false;
        [[maybe_unused]] bool settled = false;  // read only by SDEBUG_CHARGING

        do {
          vTaskDelay(pdMS_TO_TICKS(BAT_FORCE_STABLE_INTERVAL_MS));
          esp_task_wdt_reset();
          read_ina233_now(&voltage);
          if (haveFirst && fabs(voltage - prevVoltage) <= BAT_FORCE_STABLE_TOLERANCE_V) {
            settled = true;
            break;
          }
          prevVoltage = voltage;
          haveFirst = true;
        } while (CURRENT_TIME - settleStart < BAT_FORCE_STABLE_MAX_MS);

        SDEBUG_CHARGING("[BAT] measure_bat_voltage: battery connected, raw=", dbg_fixed(voltage, 3),
               "V (", settled ? "settled" : "gave up waiting to settle",
               ", total wait ", (unsigned long)(CURRENT_TIME - waitTime), "ms)");
      } else {
        SDEBUG_CHARGING("[BAT] measure_bat_voltage: battery did not appear after 1000ms — no measurement");
      }

      if (reg_bat_force_off) {
        SDEBUG_CHARGING("[BAT] measure_bat_voltage: restoring BATT_FORCE=OFF, BATT_OFF=ON");
        REG_OFF(REG_BATT_FORCE);
        REG_ON(REG_BATT_OFF);

        waitTime = CURRENT_TIME;

        while ((CURRENT_TIME - waitTime <= 1000) &&
               (IS_REG_IN_ON(REG_IN_BAT_SENSE))) {
          read_regs();
          esp_task_wdt_reset();
          vTaskDelay(pdMS_TO_TICKS(1));
        };

        REG_OFF(REG_BATT_OFF);
        SDEBUG_CHARGING("[BAT] measure_bat_voltage: battery disconnected after ", (unsigned long)(CURRENT_TIME - waitTime), "ms");
      }
    } else {
      SDEBUG_CHARGING("[BAT] measure_bat_voltage: BATT_FORCE already ON but BAT_SENSE=OFF — no measurement");
    }
  }

  // Reschedule after every attempt, not only a successful one: with the
  // battery absent each attempt pulses BATT_FORCE/BATT_OFF, and a stale
  // readBatTime made StateMachineTask retry (and click the relays) every tick.
  readBatTime = CURRENT_TIME;

  if (voltage > 0.0) {
    for (uint8_t i = 0; i < ADC2_SMOOTH; i++) {
      adc_2_1.add(voltage);
    }
    SDEBUG_CHARGING("[BAT] measure_bat_voltage: stored raw=", dbg_fixed(voltage, 3), "V smoothed=", dbg_fixed(adc_2_1.get(), 3), "V");
  } else {
    SDEBUG_CHARGING("[BAT] measure_bat_voltage: no valid voltage, smoothed unchanged=", dbg_fixed(adc_2_1.get(), 3), "V");
  }
}

// The choke solenoid needs battery power to hold closed. If the battery
// isn't currently connected (REG_IN_BAT_SENSE off), force it on and wait for
// it to actually appear before the caller stages REG_CHOKE — mirrors
// measure_bat_voltage()'s force/wait-for-sense pattern. No-op if the battery
// is already connected, or already forced on by something else (in which
// case it isn't ours to release later).
void choke_ensure_battery(void) {
  if (IS_REG_IN_ON(REG_IN_BAT_SENSE)) return;
  if (IS_REG_ON(REG_BATT_FORCE)) return;

  SDEBUG("[CHOKE] battery not sensed — forcing REG_BATT_FORCE for choke power");
  REG_ON(REG_BATT_FORCE);
  choke_forced_battery = true;

  unsigned long long waitTime = CURRENT_TIME;
  while ((CURRENT_TIME - waitTime <= 1000) && IS_REG_IN_OFF(REG_IN_BAT_SENSE)) {
    read_regs();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  if (IS_REG_IN_OFF(REG_IN_BAT_SENSE)) {
    SDEBUG("[CHOKE] battery still not sensed after 1000ms — choke may be unpowered");
  }
}

// Releases a battery connection forced on by choke_ensure_battery(), if any
// — no-op otherwise. Call once the choke has turned off (or the start
// attempt is being abandoned).
void choke_release_battery(void) {
  if (!choke_forced_battery) return;

  SDEBUG("[CHOKE] releasing forced battery connection");
  REG_OFF(REG_BATT_FORCE);
  REG_ON(REG_BATT_OFF);

  unsigned long long waitTime = CURRENT_TIME;
  while ((CURRENT_TIME - waitTime <= 1000) && IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
    read_regs();
    esp_task_wdt_reset();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  REG_OFF(REG_BATT_OFF);

  choke_forced_battery = false;
}

// Configurable via /wifi-config (BatteryConfig_t.psu_loss_backup_enabled).
// While the generator isn't running: if BOTH PSU_SENSE and GEN_SENSE drop —
// i.e. the controller has no external power source left at all — force the
// battery on (REG_BATT_FORCE) to keep the controller powered; once either
// one returns, release it (REG_BATT_FORCE off + a REG_BATT_OFF latch-reset
// pulse, same pattern as choke_release_battery()/measure_bat_voltage()).
// GEN_SENSE alone (PSU lost, genset output already powering the controller)
// deliberately does NOT engage this — forcing the battery on top of a live
// external supply is pointless. Tracking a single combined signal (instead
// of PSU_SENSE alone) also means: if backup is already engaged and GEN_SENSE
// then appears, it gets released same as a PSU return would; and if backup
// wasn't engaged because GEN_SENSE was covering for a lost PSU_SENSE and GEN
// then disappears too, that now correctly engages backup instead of being
// missed (a second edge that could never fire against PSU_SENSE alone, since
// it had already been sitting at "off" the whole time). Only called from
// process_STATE_IDLE() — STARTING/STARTED/RUNNING already have their own
// REG_BATT_FORCE lifecycle (choke power, then unconditional cleanup on
// start) that this must not fight with.
void psu_backup_check(void) {
  static bool psu_backup_prev_ext_power_ok = true;  // assume present at boot
  if (!g_battery_config.psu_loss_backup_enabled) {
    if (psu_backup_engaged) {
      SDEBUG_CHARGING("[BAT] PSU backup: feature disabled while engaged — releasing");
      REG_OFF(REG_BATT_FORCE);
      REG_ON(REG_BATT_OFF);

      unsigned long long waitTime = CURRENT_TIME;
      while ((CURRENT_TIME - waitTime <= 1000) && IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
        read_regs();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      REG_OFF(REG_BATT_OFF);

      psu_backup_engaged = false;
    }
    // Keep tracking the live level so re-enabling later doesn't see a stale edge.
    psu_backup_prev_ext_power_ok = IS_REG_IN_ON(REG_IN_PSU_SENSE) || IS_REG_IN_ON(REG_IN_GEN_SENSE);
    return;
  }

  bool ext_power_ok_now = IS_REG_IN_ON(REG_IN_PSU_SENSE) || IS_REG_IN_ON(REG_IN_GEN_SENSE);

  if (psu_backup_prev_ext_power_ok && !ext_power_ok_now) {
    // Falling edge: neither PSU nor genset output is present any more.
    if (IS_REG_OFF(REG_BATT_FORCE)) {
      SDEBUG_CHARGING("[BAT] PSU backup: PSU_SENSE and GEN_SENSE both lost while idle — forcing REG_BATT_FORCE on");
      REG_ON(REG_BATT_FORCE);
      psu_backup_engaged = true;
      // Keep the battery-only shutdown-threshold check (below, in
      // process_STATE_IDLE()) live for the whole outage instead of letting
      // it lapse after g_timing_config.after_run_period_ms of inactivity.
      afterRunPeriod = true;
      afterRunPeriodTime = CURRENT_TIME;
    } else {
      SDEBUG_CHARGING("[BAT] PSU backup: PSU_SENSE and GEN_SENSE both lost but REG_BATT_FORCE already on (not ours) — not touching it");
    }
  } else if (!psu_backup_prev_ext_power_ok && ext_power_ok_now) {
    // Rising edge: PSU or genset output returned.
    if (psu_backup_engaged) {
      SDEBUG_CHARGING("[BAT] PSU backup: external power restored — releasing REG_BATT_FORCE");
      REG_OFF(REG_BATT_FORCE);
      REG_ON(REG_BATT_OFF);

      unsigned long long waitTime = CURRENT_TIME;
      while ((CURRENT_TIME - waitTime <= 1000) && IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
        read_regs();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1));
      }
      REG_OFF(REG_BATT_OFF);

      psu_backup_engaged = false;
    }
  }

  psu_backup_prev_ext_power_ok = ext_power_ok_now;
}

// uptime_ms() when the current engine session started (entered STATE_STARTED);
// 0 = no session. Survives STOPPING->STARTED bounce (stop failed).
static uint32_t flog_session_start_ms = 0;

// Charging session tracking (start time + active flag for duration/reasons)
static bool     flog_charge_active   = false;
static uint32_t flog_charge_start_ms = 0;

float flog_head_temp(void) {
  portENTER_CRITICAL(&g_state_spinlock);
  float t = g_state.sensors.head_temp;
  portEXIT_CRITICAL(&g_state_spinlock);
  return t;
}

// Cached head_temp for the cold/warm-start choke decision (switch_STATE_STARTING
// et al) — same cache SensorTask already fills every PERIOD_SENSOR_MS, no direct
// I2C access from StateMachineTask. Preserves the previous direct-read code's
// "assume warm if sensor absent or bus busy" fail-safe: falls back to 99.0
// (no choke) instead of trusting a stale/never-set 0.0, which would otherwise
// read as "cold" and turn the choke on for the wrong reason.
float flog_head_temp_or_warm(void) {
  portENTER_CRITICAL(&g_state_spinlock);
  bool  valid = g_state.sensors.head_temp_valid;
  float t     = g_state.sensors.head_temp;
  portEXIT_CRITICAL(&g_state_spinlock);
  return valid ? t : 99.0f;
}

// See declaration next to g_limits_config for why this is forward-declared.
bool safety_start_inhibited(void) {
  if (IS_REG_IN_ON(REG_IN_OVERHEAT) || IS_REG_IN_ON(REG_IN_OIL) ||
      IS_REG_IN_ON(REG_IN_OVERLOAD)) {
    return true;
  }
  if (g_limits_config.temp_air_max      > 0.0f && (float)adc_1_1.get() >= g_limits_config.temp_air_max)      return true;
  if (g_limits_config.temp_oil_max      > 0.0f && (float)adc_1_0.get() >= g_limits_config.temp_oil_max)      return true;
  if (g_limits_config.temp_engine_max   > 0.0f && flog_head_temp()     >= g_limits_config.temp_engine_max)   return true;
  if (g_limits_config.temp_inverter_max > 0.0f && (float)adc_1_2.get() >= g_limits_config.temp_inverter_max) return true;
  return false;
}

void flog_charge_started(bool auto_started) {
  flog_charge_active = true;
  flog_charge_start_ms = uptime_ms();
  fram_log_set_charging(true);
  FramLogRecord r = {};
  r.type = FLOG_EV_CHARGE_START;
  r.d.charge_start.bat_v = (float)adc_2_1.get();
  r.d.charge_start.auto_started = auto_started ? 1 : 0;
  fram_log_event(&r);
}

void flog_charge_stopped(uint8_t reason) {
  if (!flog_charge_active) return;
  flog_charge_active = false;
  fram_log_set_charging(false);
  FramLogRecord r = {};
  r.type = FLOG_EV_CHARGE_STOP;
  r.d.charge_stop.bat_v = (float)adc_2_1.get();
  r.d.charge_stop.charge_seconds = (uptime_ms() - flog_charge_start_ms) / 1000;
  r.d.charge_stop.reason = reason;
  fram_log_event(&r);
}

// Called from switch_state() right before the state actually changes.
void flog_on_transition(state old_s, state new_s) {
  FramLogRecord r = {};
  bool was_starting = (old_s == STATE_STARTING || old_s == STATE_STARTING_SINGLE ||
                       old_s == STATE_STARTING_RETRY || old_s == STATE_STARTING_RETRY_TRY);

  switch (new_s) {
  case STATE_STARTED:
    if (flog_session_start_ms != 0) break;  // stop failed, engine never stopped
    flog_session_start_ms = uptime_ms();
    r.type = FLOG_EV_START_OK;
    r.d.start_ok.attempts = was_starting ? (uint8_t)(startTry > 0 ? startTry : 1) : 0;
    r.d.start_ok.detected = was_starting ? 0 : 1;  // found running without a start sequence
    r.d.start_ok.head_temp = flog_head_temp();
    r.d.start_ok.bat_v = (float)adc_2_1.get();
    r.d.start_ok.crank_ms = was_starting ? (uint32_t)(CURRENT_TIME - startTime) : 0;
    fram_log_event(&r);
    break;

  // This is the true entry point of every stop path (see STATE(...) call
  // sites) — STATE_STOPPING itself is now only reached from here, after the
  // load-relay-off delay, so classification must happen on this transition
  // instead, while old_s still reflects the state the engine was actually in.
  case STATE_STOPPING_LOAD_OFF:
    if (old_s == STATE_STARTING || old_s == STATE_STARTING_SINGLE) {
      // Single start attempt timed out
      r.type = FLOG_EV_START_FAIL;
      r.d.start_fail.attempts = 1;
      fram_log_event(&r);
    } else if (old_s == STATE_STOPPED_RETRY) {
      // Retry starts exhausted (startTry >= g_timing_config.max_start_try)
      r.type = FLOG_EV_START_FAIL;
      r.d.start_fail.attempts = (uint8_t)startTry;
      fram_log_event(&r);
    } else if ((old_s == STATE_RUNNING || old_s == STATE_STARTED) &&
               flog_session_start_ms != 0 &&
               uptime_ms() - flog_last_stop_req_ms > 10000) {
      // Engine died without a stop command — classify by protection inputs
      uint8_t inputs;
      portENTER_CRITICAL(&g_state_spinlock);
      inputs = g_state.shift_reg_inputs;
      portEXIT_CRITICAL(&g_state_spinlock);
      r.type = FLOG_EV_EMERGENCY_STOP;
      if      (inputs & (1 << REG_IN_OIL))      r.d.emergency.reason = FLOG_REASON_OIL;
      else if (inputs & (1 << REG_IN_OVERHEAT)) r.d.emergency.reason = FLOG_REASON_OVERHEAT;
      else if (inputs & (1 << REG_IN_OVERLOAD)) r.d.emergency.reason = FLOG_REASON_OVERLOAD;
      else if (flog_pending_temp_limit_stop)    r.d.emergency.reason = FLOG_REASON_TEMP_LIMIT;
      else                                      r.d.emergency.reason = FLOG_REASON_STALLED;
      r.d.emergency.inputs = inputs;
      r.d.emergency.head_temp = flog_head_temp();
      r.d.emergency.run_seconds = (uptime_ms() - flog_session_start_ms) / 1000;
      fram_log_event(&r);
      buzzerAlarm();  // silenced once we actually reach STOPPED/IDLE below — see there
    }
    flog_pending_temp_limit_stop = false;
    break;

  case STATE_STOPPED:
  case STATE_IDLE:
    // Unconditional, not just when a FLOG_EV_STOPPED gets logged below: this
    // is the one place both STATE_STOPPED and STATE_IDLE funnel through, so
    // it's the natural spot to silence any buzzerAlarm() started above once
    // the generator has actually come to rest — cheap no-op if none is active.
    buzzerStop();
    if (flog_session_start_ms != 0) {
      r.type = FLOG_EV_STOPPED;
      r.d.stopped.run_seconds = (uptime_ms() - flog_session_start_ms) / 1000;
      r.d.stopped.head_temp = flog_head_temp();
      r.d.stopped.fuel_l = (float)fuel_level_from_adc(adc_2_0.get());
      fram_log_event(&r);
      flog_session_start_ms = 0;
    }
    break;

  default:
    break;
  }
}

bool switch_STATE_INIT(void) {
  bool result = true;

  SDEBUG("Switch state INIT");
  //    readBatPeriod = g_battery_config.read_period_ms;

  return result;
}

bool process_STATE_INIT(void) {
  bool result = true;
  return result;
}

bool switch_STATE_IDLE(void) {
  bool result = true;

  SDEBUG("Switch state IDLE");

  REG_OFF_STATE(REG_CHOKE);
  REG_OFF_STATE(REG_IGNITION);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_BATT_OFF);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_ON_STATE(REG_LOAD_RELAY);

  REG_SYNC_STATE();

  startTry = 0;
  startTime = CURRENT_TIME;
  startRetryTime = CURRENT_TIME;
  stoppingTime = CURRENT_TIME;
  runningTime = CURRENT_TIME;

  readBatPeriod = g_battery_config.read_after_run_period_ms;

  afterRunPeriodTime = CURRENT_TIME;
  afterRunPeriod = true;

  // If a start happened mid-outage, switch_STATE_STARTED()/process_STATE_RUNNING()
  // already unconditionally cleared REG_BATT_FORCE (see psu_backup_check()'s
  // comment) — nothing physical is left for us to release, so just drop the flag.
  psu_backup_engaged = false;

  return result;
}

bool process_STATE_IDLE(void) {
  bool result = true;

  if (!ina233_present) {
    charging = false;
    return result;
  }

  double bat_v = adc_2_1.get();
  bool psu_ok  = IS_REG_IN_ON(REG_IN_PSU_SENSE);
  bool charger_on = IS_REG_ON(REG_CHARGER);
  // True while the controller is powered purely from the battery (no PSU, no
  // genset output) — used both to keep bat_v readings fresh below and to gate
  // the debounced low-voltage shutdown check further down.
  bool battery_only_supply = IS_REG_IN_OFF(REG_IN_PSU_SENSE) &&
                             IS_REG_IN_OFF(REG_IN_GEN_SENSE) &&
                             IS_REG_IN_ON(REG_IN_BAT_SENSE);

  if (charger_on) {
    if (psu_ok) {
      if (charging) {
        // Periodically measure battery voltage via BATT_FORCE pulse.
        // BATT_FORCE is only asserted for the duration of measurement,
        // then released; BATT_OFF is pulsed to ensure the self-hold relay
        // actually releases (handled inside measure_bat_voltage()).
        if (CURRENT_TIME - readBatTime >= readBatPeriod) {
          SDEBUG_CHARGING("[BAT] IDLE: charging — periodic measurement (elapsed=",
                 (unsigned long)((CURRENT_TIME - readBatTime) / 1000),
                 "s >= period=", (unsigned long)(readBatPeriod / 1000), "s)");
          measure_bat_voltage();
          // Refresh bat_v so the stop check below uses the fresh reading
          bat_v = adc_2_1.get();
          SDEBUG_CHARGING("[BAT] IDLE: measurement done, bat=", dbg_fixed(bat_v, 3), "V");
        }

        SDEBUG_CHARGING("[BAT] IDLE: charging in progress — bat=", dbg_fixed(bat_v, 3),
               "V stop_threshold=", dbg_fixed((double)g_battery_config.threshold_stop_charge_v, 2), "V");
        if (bat_v >= g_battery_config.threshold_stop_charge_v) {
          SDEBUG_CHARGING("[BAT] IDLE: STOP CHARGING — bat=", dbg_fixed(bat_v, 3),
                 "V >= threshold=", dbg_fixed((double)g_battery_config.threshold_stop_charge_v, 2), "V, disabling charger");
          charging = false;
          flog_charge_stopped(FLOG_CHG_THRESHOLD);
          REG_OFF(REG_CHARGER);
          readBatPeriod = g_battery_config.read_after_run_period_ms;
          afterRunPeriodTime = CURRENT_TIME;
          afterRunPeriod = true;
          SDEBUG_CHARGING("[BAT] IDLE: charger OFF, next bat reading in ",
                 (unsigned long)(readBatPeriod / 1000), "s (AFTER_RUN_PERIOD)");
        }
      } else {
        // CHARGER relay is ON but charging flag is false — sync flag
        SDEBUG_CHARGING("[BAT] IDLE: charger relay ON but charging=false — syncing flag to true, bat=", dbg_fixed(bat_v, 3), "V");
        charging = true;
        flog_charge_started(false);  // relay turned on manually (web/MQTT)
        readBatPeriod = g_battery_config.read_charging_period_ms;
        readBatTime = CURRENT_TIME - readBatPeriod;  // force immediate measurement on next cycle
        afterRunPeriod = false;
      }
    } else {
      // PSU disappeared — turn off charger
      SDEBUG_CHARGING("[BAT] IDLE: STOP CHARGING — PSU_SENSE went OFF while charger was running, bat=", dbg_fixed(bat_v, 3), "V");
      charging = false;
      flog_charge_stopped(FLOG_CHG_PSU_LOST);
      REG_OFF(REG_CHARGER);
      readBatPeriod = g_battery_config.read_after_run_period_ms;
      afterRunPeriodTime = CURRENT_TIME;
      afterRunPeriod = true;
      SDEBUG_CHARGING("[BAT] IDLE: charger OFF (no PSU), next bat reading in ",
             (unsigned long)(readBatPeriod / 1000), "s (AFTER_RUN_PERIOD)");
    }
  } else {
    // Charger relay is OFF
    if (charging) {
      // Flag out of sync with relay
      SDEBUG_CHARGING("[BAT] IDLE: charger relay OFF but charging=true — syncing flag to false, bat=", dbg_fixed(bat_v, 3), "V");
      charging = false;
      flog_charge_stopped(FLOG_CHG_MANUAL);  // relay turned off manually (web/MQTT)
      readBatPeriod = g_battery_config.read_after_run_period_ms;
      afterRunPeriodTime = CURRENT_TIME;
      afterRunPeriod = true;
    } else {
      // Periodically measure battery voltage even when not charging
      if (CURRENT_TIME - readBatTime >= readBatPeriod) {
        SDEBUG_CHARGING("[BAT] IDLE: not charging — periodic measurement (elapsed=",
               (unsigned long)((CURRENT_TIME - readBatTime) / 1000),
               "s >= period=", (unsigned long)(readBatPeriod / 1000), "s)");
        measure_bat_voltage();
        bat_v = adc_2_1.get();
        SDEBUG_CHARGING("[BAT] IDLE: measurement done, bat=", dbg_fixed(bat_v, 3), "V");
      }

      if (psu_ok) {
        SDEBUG_CHARGING("[BAT] IDLE: PSU OK, bat=", dbg_fixed(bat_v, 3),
               "V start_threshold=", dbg_fixed((double)g_battery_config.threshold_start_charge_v, 2),
               "V — ", bat_v <= g_battery_config.threshold_start_charge_v ? "START CHARGING" : "no charging needed");
        if (bat_v <= g_battery_config.threshold_start_charge_v) {
          if (charge_pending_since_ms == 0) {
            charge_pending_since_ms = (uint32_t)CURRENT_TIME;
            // Force fast re-measurement while the debounce window is open —
            // otherwise the slow read_period_ms (up to 12h) would leave this
            // check evaluating one stale reading instead of tracking actual
            // persistence, same issue the battery_only_supply fast-cadence
            // fix above addresses for the shutdown debounce.
            readBatPeriod = g_battery_config.read_after_run_period_ms;
            readBatTime = CURRENT_TIME - readBatPeriod;
          }
          uint32_t pending_elapsed_s = (uint32_t)(CURRENT_TIME - charge_pending_since_ms) / 1000;
          if (g_battery_config.start_charge_persist_time_s == 0 ||
              pending_elapsed_s >= g_battery_config.start_charge_persist_time_s) {
            SDEBUG_CHARGING("[BAT] IDLE: START CHARGING — bat=", dbg_fixed(bat_v, 3),
                   "V <= threshold=", dbg_fixed((double)g_battery_config.threshold_start_charge_v, 2),
                   "V for ", pending_elapsed_s, "s, enabling charger");
            charging = true;
            flog_charge_started(true);
            REG_ON(REG_CHARGER);
            readBatPeriod = g_battery_config.read_charging_period_ms;
            readBatTime = CURRENT_TIME - readBatPeriod;  // force immediate measurement on next process_STATE_IDLE cycle
            afterRunPeriod = false;
            charge_pending_since_ms = 0;
            SDEBUG_CHARGING("[BAT] IDLE: charger ON, bat measurement every ",
                   (unsigned long)(readBatPeriod / 1000), "s via BATT_FORCE pulse");
          } else {
            SDEBUG_CHARGING("[BAT] IDLE: bat=", dbg_fixed(bat_v, 3),
                   "V <= start_threshold=", dbg_fixed((double)g_battery_config.threshold_start_charge_v, 2),
                   "V for ", pending_elapsed_s, "s (need ",
                   (g_battery_config.start_charge_persist_time_s), "s before starting charger)");
          }
        } else {
          // bat_v recovered above threshold without ever completing the
          // persist debounce (charging never actually started) — readBatPeriod
          // was forced to read_after_run_period_ms above when the pending
          // debounce armed, but afterRunPeriod was never set, so nothing would
          // otherwise ever return it to the slow read_period_ms. Arm the
          // after-run window here so it does.
          if (charge_pending_since_ms != 0) {
            afterRunPeriod = true;
            afterRunPeriodTime = CURRENT_TIME;
          }
          charge_pending_since_ms = 0;
        }
      } else {
        SDEBUG_CHARGING("[BAT] IDLE: PSU_SENSE=OFF, charging not possible, bat=", dbg_fixed(bat_v, 3), "V");
        if (charge_pending_since_ms != 0) {
          afterRunPeriodTime = CURRENT_TIME;
        }
        charge_pending_since_ms = 0;
      }
    }

    if (afterRunPeriod) {
      unsigned long long elapsed = CURRENT_TIME - afterRunPeriodTime;
      if (elapsed >= g_timing_config.after_run_period_ms) {
        if (battery_only_supply || charge_pending_since_ms != 0) {
          // Still running on battery-only supply, or a charge-start debounce
          // is armed — keep the fast read_after_run_period_ms cadence instead
          // of dropping to the (default 12h) idle period, so the
          // shutdown_persist_time_s / start_charge_persist_time_s debounces
          // keep getting fresh-enough readings to actually measure
          // persistence against. Re-arm afterRunPeriodTime so this branch
          // doesn't re-log every single iteration.
          SDEBUG_CHARGING("[BAT] IDLE: afterRunPeriod expired (", (unsigned long)(elapsed / 1000),
                 "s elapsed) but battery_only_supply or charge-start debounce active — keeping fast read period");
          afterRunPeriodTime = CURRENT_TIME;
        } else {
          SDEBUG_CHARGING("[BAT] IDLE: afterRunPeriod expired (", (unsigned long)(elapsed / 1000),
                 "s elapsed), switching to slow bat reading every ", (unsigned long)(g_battery_config.read_period_ms / 1000), "s");
          afterRunPeriod = false;
          readBatPeriod = g_battery_config.read_period_ms;
        }
      }
    }
  }

  psu_backup_check();

  // Safety: on battery-only supply (no PSU, no genset), shut the battery off
  // once bat_v has stayed <= threshold_shutdown_v continuously for
  // shutdown_persist_time_s, rather than on the first low reading — avoids
  // self-disconnecting the controller over a brief voltage sag. Runs for as
  // long as battery_only_supply holds, not just during afterRunPeriod (see
  // the read-period guard above).
  if (battery_only_supply) {
    if (bat_v <= g_battery_config.threshold_shutdown_v) {
      if (batt_low_since_ms == 0) batt_low_since_ms = (uint32_t)CURRENT_TIME;
      uint32_t low_elapsed_s = (uint32_t)(CURRENT_TIME - batt_low_since_ms) / 1000;
      if (g_battery_config.shutdown_persist_time_s == 0 ||
          low_elapsed_s >= g_battery_config.shutdown_persist_time_s) {
        SDEBUG_CHARGING("[BAT] IDLE: SHUTDOWN — bat-only supply, bat=", dbg_fixed(bat_v, 3),
               "V <= shutdown_threshold=", dbg_fixed((double)g_battery_config.threshold_shutdown_v, 2),
               "V for ", low_elapsed_s, "s, disabling battery");
        REG_OFF(REG_BATT_FORCE); // shutdown bat only supply
        REG_ON(REG_BATT_OFF);
        FramLogRecord r = {};
        r.type = FLOG_EV_ERROR;
        r.d.error.code = 3;  // 3 = battery-only supply shutdown after persist timeout
        r.d.error.value = (float)bat_v;
        fram_log_event(&r);
        batt_low_since_ms = 0;
      } else {
        SDEBUG_CHARGING("[BAT] IDLE: bat-only supply, bat=", dbg_fixed(bat_v, 3),
               "V <= shutdown_threshold=", dbg_fixed((double)g_battery_config.threshold_shutdown_v, 2),
               "V for ", low_elapsed_s, "s (need ",
               (g_battery_config.shutdown_persist_time_s), "s before shutdown)");
      }
    } else {
      batt_low_since_ms = 0;
    }
  } else {
    batt_low_since_ms = 0;
  }

  return result;
}

bool switch_STATE_STOPPING_LOAD_OFF(void) {
  bool result = true;

  SDEBUG("Switch state STOPPING_LOAD_OFF");

  // Remember which stop variant to enter once the delay below elapses.
  // switch_state() hasn't updated CURRENT_STATE yet at this point, so this
  // still reads the state we're leaving.
  stoppingToRetry = IS_CURRENT_STATE(STATE_STARTING_RETRY_TRY);

  // Immediate write (not staged) — this must physically happen now, well
  // before ignition/choke are touched in STATE_STOPPING(_RETRY).
  REG_OFF(REG_LOAD_RELAY);

  // Starter too, same pulse: previously REG_START only dropped in
  // STATE_STOPPING(_RETRY) after load_relay_stop_delay_ms (500 ms default),
  // so a crank-timeout stop kept the bendix spinning for that whole delay
  // after the software had already given up. Load-off-before-ignition-cut
  // still holds; the starter is not part of that sequencing. STOPPING still
  // clears it staged as a backstop. Found 2026-09-28.
  REG_OFF(REG_START);

  // Also immediate, same reasoning: previously ECO only got explicitly
  // cleared once STATE_STOPPED(_RETRY) was reached, leaving it energized
  // through the entire ignition-cut + coast-down window. Cleared here so it
  // drops in the same latch pulse as the load relay, at the very first step
  // of the stop sequence rather than at its very end. STATE_STOPPED(_RETRY)
  // still clears it too (staged, redundant but harmless) — that's the
  // backstop against something re-enabling it mid-stop via a manual
  // /reg or MQTT eco_mode command, which isn't gated by state.
  REG_OFF(REG_ECO_MODE);

  loadOffTime = CURRENT_TIME;

  // Catch-all: release any battery connection the choke forced on, in case
  // we're abandoning a start attempt mid-crank rather than reaching RUNNING
  // (where this is normally released alongside the choke itself). No-op if
  // nothing was forced.
  choke_release_battery();

  return result;
}

bool process_STATE_STOPPING_LOAD_OFF(void) {
  return true;
}

bool switch_STATE_STOPPING(void) {
  bool result = true;

  SDEBUG("Switch state STOPPING");

  REG_ON_STATE(REG_IGNITION);

  REG_OFF_STATE(REG_CHOKE);
  REG_OFF_STATE(REG_START);  // already dropped in LOAD_OFF; staged backstop

  // REG_LOAD_RELAY was already dropped in STATE_STOPPING_LOAD_OFF.

  REG_SYNC_STATE();

  startRetryTime = CURRENT_TIME;
  stoppingTime = CURRENT_TIME;
  runningTime = CURRENT_TIME;

  return result;
}

bool process_STATE_STOPPING(void) {
  return true;
}

bool switch_STATE_STOPPING_RETRY(void) {
  bool result = true;

  SDEBUG("Switch state STOPPING_RETRY");

  REG_ON_STATE(REG_IGNITION);

  REG_OFF_STATE(REG_CHOKE);
  REG_OFF_STATE(REG_START);

  // REG_LOAD_RELAY was already dropped in STATE_STOPPING_LOAD_OFF.

  REG_SYNC_STATE();

  startRetryTime = CURRENT_TIME;
  stoppingTime = CURRENT_TIME;
  runningTime = CURRENT_TIME;

  return result;
}

bool process_STATE_STOPPING_RETRY(void) {
  return true;
}

bool switch_STATE_STOPPED(void) {
  bool result = true;

  SDEBUG("Switch state STOPPED");

  REG_OFF_STATE(REG_IGNITION);

  REG_OFF_STATE(REG_CHOKE);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_LOAD_RELAY);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_SYNC_STATE();

  stoppingTime = CURRENT_TIME;

  return result;
}

bool process_STATE_STOPPED(void) {
  return true;
}

bool switch_STATE_STOPPED_RETRY(void) {
  bool result = true;

  SDEBUG("Switch state STOPPED_RETRY");

  REG_OFF_STATE(REG_IGNITION);

  REG_OFF_STATE(REG_CHOKE);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_LOAD_RELAY);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_SYNC_STATE();

  stoppingTime = CURRENT_TIME;

  return result;
}

bool process_STATE_STOPPED_RETRY(void) {
  return true;
}

bool switch_STATE_STARTING(void) {
  bool result = true;

  SDEBUG("Switch state STARTING");
  SDEBUG_CHARGING("[BAT] switch STARTING: disabling charger (engine start sequence), bat=", dbg_fixed(adc_2_1.get(), 3), "V");

  {
    float head_temp = flog_head_temp_or_warm();
    if (head_temp < g_limits_config.choke_cold_start_temp) {
      SDEBUG("[CHOKE] Cold start: head_temp=", dbg_fixed(head_temp, 1), " < ", dbg_fixed((double)g_limits_config.choke_cold_start_temp, 1), " — choke ON");
      choke_ensure_battery();
      REG_ON_STATE(REG_CHOKE);
    } else {
      SDEBUG("[CHOKE] Warm start: head_temp=", dbg_fixed(head_temp, 1), " >= ", dbg_fixed((double)g_limits_config.choke_cold_start_temp, 1), " — choke OFF");
      REG_OFF_STATE(REG_CHOKE);
    }
  }
  REG_OFF_STATE(REG_IGNITION);
  REG_ON_STATE(REG_START);
  REG_OFF_STATE(REG_CHARGER);
  charging = false;
  flog_charge_stopped(FLOG_CHG_ENGINE_START);
  REG_OFF_STATE(REG_LOAD_RELAY);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_SYNC_STATE();

  startTry = 0;
  startTime = CURRENT_TIME;

  readBatPeriod = g_battery_config.read_starting_period_ms;

  return result;
}

bool process_STATE_STARTING(void) {
  return true;
}

bool switch_STATE_STARTING_SINGLE(void) {
  bool result = true;

  SDEBUG("Switch state STARTING_SINGLE");
  SDEBUG_CHARGING("[BAT] switch STARTING_SINGLE: disabling charger (single-try start), bat=", dbg_fixed(adc_2_1.get(), 3), "V");

  REG_OFF_STATE(REG_IGNITION);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_CHARGER);
  charging = false;
  flog_charge_stopped(FLOG_CHG_ENGINE_START);
  REG_OFF_STATE(REG_LOAD_RELAY);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_SYNC_STATE();

  startTry = 0;
  startTime = CURRENT_TIME;
  readBatPeriod = g_battery_config.read_starting_period_ms;

  return result;
}

bool process_STATE_STARTING_SINGLE(void) {
  return true;
}

bool switch_STATE_STARTING_RETRY(void) {
  bool result = true;

  SDEBUG("Switch state STARTING_RETRY");
  SDEBUG_CHARGING("[BAT] switch STARTING_RETRY: disabling charger (retry start sequence), bat=", dbg_fixed(adc_2_1.get(), 3), "V");

  REG_OFF_STATE(REG_IGNITION);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_CHARGER);
  charging = false;
  flog_charge_stopped(FLOG_CHG_ENGINE_START);
  REG_OFF_STATE(REG_LOAD_RELAY);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_SYNC_STATE();

  startTry = 0;
  startTime = CURRENT_TIME;
  readBatPeriod = g_battery_config.read_starting_period_ms;

  return result;
}

bool process_STATE_STARTING_RETRY(void) {
  return true;
}

bool switch_STATE_STARTING_RETRY_TRY(void) {
  bool result = true;

  SDEBUG("Switch state STATE_STARTING_RETRY_TRY");
  SDEBUG_CHARGING("[BAT] switch STARTING_RETRY_TRY: disabling charger (retry attempt #", startTry + 1, "), bat=", dbg_fixed(adc_2_1.get(), 3), "V");

  {
    float head_temp = flog_head_temp_or_warm();
    if (head_temp < g_limits_config.choke_cold_start_temp) {
      SDEBUG("[CHOKE] Cold retry: head_temp=", dbg_fixed(head_temp, 1), " < ", dbg_fixed((double)g_limits_config.choke_cold_start_temp, 1), " — choke ON");
      choke_ensure_battery();
      REG_ON_STATE(REG_CHOKE);
    } else {
      SDEBUG("[CHOKE] Warm retry: head_temp=", dbg_fixed(head_temp, 1), " >= ", dbg_fixed((double)g_limits_config.choke_cold_start_temp, 1), " — choke OFF");
      REG_OFF_STATE(REG_CHOKE);
    }
  }
  REG_OFF_STATE(REG_IGNITION);
  REG_ON_STATE(REG_START);
  REG_OFF_STATE(REG_CHARGER);
  charging = false;
  flog_charge_stopped(FLOG_CHG_ENGINE_START);
  REG_OFF_STATE(REG_LOAD_RELAY);
  REG_OFF_STATE(REG_ECO_MODE);

  REG_SYNC_STATE();

  startTry++;
  startTime = CURRENT_TIME;

  return result;
}

bool process_STATE_STARTING_RETRY_TRY(void) {
  bool result = true;

  measure_bat_voltage();

  return result;
}

bool switch_STATE_STARTED(void) {
  bool result = true;

  SDEBUG("Switch state STATED");
  SDEBUG_CHARGING("[BAT] switch STARTED: disabling charger (generator engine started), bat=", dbg_fixed(adc_2_1.get(), 3), "V");

  REG_OFF_STATE(REG_IGNITION);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_CHARGER);
  charging = false;
  flog_charge_stopped(FLOG_CHG_ENGINE_START);

  REG_OFF_STATE(REG_LOAD_RELAY);

  // Leave the battery connected if choke_ensure_battery() forced it on for
  // the choke solenoid — released later, alongside the choke itself, in
  // process_STATE_RUNNING(). Same for the general battery-dependency check
  // (manually-toggled choke/fuel pump) — otherwise this is the normal
  // unconditional cleanup (e.g. a stray force left on from periodic voltage
  // measurement).
  if (!choke_forced_battery && !battery_dependency_forced) {
    REG_OFF_STATE(REG_BATT_FORCE);
    REG_ON_STATE(REG_BATT_OFF);
  }

  REG_SYNC_STATE();

  startTime = CURRENT_TIME;
  runningTime = CURRENT_TIME;

  return result;
}

bool process_STATE_STARTED(void) {
  return true;
}

bool switch_STATE_RUNNING(void) {
  bool result = true;

  SDEBUG("Switch state RUNNING");
  SDEBUG_CHARGING("[BAT] switch RUNNING: disabling charger (generator running), bat=", dbg_fixed(adc_2_1.get(), 3),
         "V, read period=", (unsigned long)(g_battery_config.read_running_period_ms / 1000), "s");

  REG_OFF_STATE(REG_IGNITION);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_CHARGER);
  charging = false;
  flog_charge_stopped(FLOG_CHG_ENGINE_START);
  REG_OFF_STATE(REG_BATT_FORCE);
  REG_OFF_STATE(REG_BATT_OFF);
  // Once truly RUNNING, the generator's own output covers choke/fuel_pump —
  // no need to keep forcing the battery on for them. The general
  // battery-dependency safety check (StateMachineTask) also stands down
  // while in this state, so drop its bookkeeping here rather than let it
  // go stale against the unconditional REG_BATT_FORCE-off above.
  battery_dependency_forced = false;
  REG_ON_STATE(REG_ECO_MODE);

  // Configurable via /wifi-config: when disabled, leave REG_LOAD_RELAY
  // exactly as it already is instead of forcing it on.
  if (g_limits_config.auto_load_relay_on_run) {
    REG_ON_STATE(REG_LOAD_RELAY);
  }

  REG_SYNC_STATE();

  startTime = CURRENT_TIME;
  runningTime = CURRENT_TIME;
  readBatPeriod = g_battery_config.read_running_period_ms;

  return result;
}

bool process_STATE_RUNNING(void) {
  if (IS_REG_ON(REG_START))
    REG_OFF(REG_START);
  if (IS_REG_ON(REG_CHARGER)) {
    SDEBUG_CHARGING("[BAT] RUNNING: charger relay still ON — disabling (generator running, charger not needed)");
    REG_OFF(REG_CHARGER);
    charging = false;
    flog_charge_stopped(FLOG_CHG_ENGINE_START);
  }
  // Skip while the choke is still holding the battery forced on — released
  // below, together with the choke, once g_timing_config.start_time_ms has elapsed.
  // (battery_dependency_forced shouldn't be true here — cleared on entry to
  // RUNNING above, and the safety check itself stands down in this state —
  // but check it too for defense-in-depth against a stale flag.)
  if (IS_REG_ON(REG_BATT_FORCE) && !choke_forced_battery && !battery_dependency_forced)
    REG_OFF(REG_BATT_FORCE);

  if (CURRENT_TIME - startTime >= g_timing_config.start_time_ms) {
    if (IS_REG_ON(REG_CHOKE))
      REG_OFF(REG_CHOKE);
    choke_release_battery();
  }

  return true;
}

bool switch_STATE_ERROR(void) {
  bool result = true;

  SDEBUG("Switch state ERROR");

  REG_OFF_STATE(REG_CHOKE);
  REG_ON_STATE(REG_IGNITION);
  REG_OFF_STATE(REG_START);
  REG_OFF_STATE(REG_LOAD_RELAY);

  REG_SYNC_STATE();

  return result;
}

bool process_STATE_ERROR(void) {
  return true;
}

bool switch_state(const state new_state, const bool update_old) {
  bool result = false;

  if (IS_NEW_STATE(new_state)) {
    switch (new_state) {
    case STATE_INIT:
      result = SWITCH_STATE(STATE_INIT);
      break;
    case STATE_IDLE:
      result = SWITCH_STATE(STATE_IDLE);
      break;
    case STATE_STOPPING_LOAD_OFF:
      result = SWITCH_STATE(STATE_STOPPING_LOAD_OFF);
      break;
    case STATE_STOPPING:
      result = SWITCH_STATE(STATE_STOPPING);
      break;
    case STATE_STOPPING_RETRY:
      if (IS_CURRENT_STATE(STATE_STOPPING_LOAD_OFF))
        result = SWITCH_STATE(STATE_STOPPING_RETRY);
      break;
    case STATE_STOPPED:
      result = SWITCH_STATE(STATE_STOPPED);
      break;
    case STATE_STOPPED_RETRY:
      if (IS_CURRENT_STATE(STATE_STOPPING_RETRY))
        result = SWITCH_STATE(STATE_STOPPED_RETRY);
      break;
    case STATE_STARTING_SINGLE:
      if (IS_CURRENT_STATE(STATE_IDLE))
        result = SWITCH_STATE(STATE_STARTING_SINGLE);
      break;
    case STATE_STARTING:
      if (IS_CURRENT_STATE(STATE_STARTING_SINGLE))
        result = SWITCH_STATE(STATE_STARTING);
      break;
    case STATE_STARTING_RETRY:
      if (IS_CURRENT_STATE(STATE_IDLE))
        result = SWITCH_STATE(STATE_STARTING_RETRY);
      break;
    case STATE_STARTING_RETRY_TRY:
      if (IS_CURRENT_STATE(STATE_STARTING_RETRY) ||
          IS_CURRENT_STATE(STATE_STOPPED_RETRY))
        result = SWITCH_STATE(STATE_STARTING_RETRY_TRY);
      break;
    case STATE_STARTED:
      result = SWITCH_STATE(STATE_STARTED);
      break;
    case STATE_RUNNING:
      if (IS_CURRENT_STATE(STATE_STARTED))
        result = SWITCH_STATE(STATE_RUNNING);
      break;
    case STATE_ERROR:
      result = SWITCH_STATE(STATE_ERROR);
      break;
    }

    if (result) {
      flog_on_transition(CURRENT_STATE, new_state);

      if (update_old) {
        SET_PREV_STATE(CURRENT_STATE);
      }

      SET_CURRENT_STATE(new_state);
    }
  }

  return result;
}

bool process_current_state(void) {
  bool result = false;

  // A real switch, not an if-else chain, matching switch_state()'s own
  // dispatch below (14 cases == the full `state` enum, rtos_config.h) —
  // deliberately no default: label, so -Wswitch (-Wall -Wextra) catches a
  // state added to the enum but never given a branch here, instead of
  // silently falling through to result=false the way the if-chain used to.
  switch (CURRENT_STATE) {
  case STATE_INIT:
    result = PROCESS_STATE(STATE_INIT);
    break;
  case STATE_IDLE:
    result = PROCESS_STATE(STATE_IDLE);
    break;
  case STATE_STOPPING_LOAD_OFF:
    result = PROCESS_STATE(STATE_STOPPING_LOAD_OFF);
    break;
  case STATE_STOPPING:
    result = PROCESS_STATE(STATE_STOPPING);
    break;
  case STATE_STOPPING_RETRY:
    result = PROCESS_STATE(STATE_STOPPING_RETRY);
    break;
  case STATE_STOPPED:
    result = PROCESS_STATE(STATE_STOPPED);
    break;
  case STATE_STOPPED_RETRY:
    result = PROCESS_STATE(STATE_STOPPED_RETRY);
    break;
  case STATE_STARTING:
    result = PROCESS_STATE(STATE_STARTING);
    break;
  case STATE_STARTING_SINGLE:
    result = PROCESS_STATE(STATE_STARTING_SINGLE);
    break;
  case STATE_STARTING_RETRY:
    result = PROCESS_STATE(STATE_STARTING_RETRY);
    break;
  case STATE_STARTING_RETRY_TRY:
    result = PROCESS_STATE(STATE_STARTING_RETRY_TRY);
    break;
  case STATE_STARTED:
    result = PROCESS_STATE(STATE_STARTED);
    break;
  case STATE_RUNNING:
    result = PROCESS_STATE(STATE_RUNNING);
    break;
  case STATE_ERROR:
    result = PROCESS_STATE(STATE_ERROR);
    break;
  }

  return result;
}

bool process_state_check(void) {
  bool result = false;

  if (IS_CURRENT_STATE(STATE_INIT)) {
    if (is_running()) {
      result = STATE(STATE_STARTED);
    } else {
      result = STATE(STATE_IDLE);
    }
  }

  else if (IS_CURRENT_STATE(STATE_IDLE)) {
    if (is_running()) {
      result = STATE(STATE_STARTED);
    }
  }

  else if (IS_CURRENT_STATE(STATE_STARTING_SINGLE)) {
    if (is_running()) {
      result = STATE(STATE_STARTED);
    } else {
      result = STATE(STATE_STARTING);
    }
  }

  else if (IS_CURRENT_STATE(STATE_STARTING)) {
    if (CURRENT_TIME - startTime >= g_timing_config.start_time_check_ms) {
      if (is_running()) {
        result = STATE(STATE_STARTED);
      } else if (CURRENT_TIME - startTime >= g_timing_config.start_time_ms) {
        result = STATE(STATE_STOPPING_LOAD_OFF);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_STARTING_RETRY)) {
    if (is_running()) {
      result = STATE(STATE_STARTED);
    } else {
      result = STATE(STATE_STARTING_RETRY_TRY);
    }
  }

  else if (IS_CURRENT_STATE(STATE_STARTING_RETRY_TRY)) {
    if (CURRENT_TIME - startTime >= g_timing_config.start_time_check_ms) {
      if (is_running()) {
        result = STATE(STATE_STARTED);
      } else if (CURRENT_TIME - startTime >= g_timing_config.start_time_ms) {
        result = STATE(STATE_STOPPING_LOAD_OFF);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_STARTED)) {
    if (CURRENT_TIME - runningTime >= g_timing_config.running_time_ms) {
      if (is_running()) {
        result = STATE(STATE_RUNNING);
      } else {
        result = STATE(STATE_STOPPING_LOAD_OFF);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_RUNNING)) {
    if (!is_running()) {
      result = STATE(STATE_STOPPING_LOAD_OFF);
    }
  }

  else if (IS_CURRENT_STATE(STATE_STOPPING_LOAD_OFF)) {
    if (CURRENT_TIME - loadOffTime >= g_timing_config.load_relay_stop_delay_ms) {
      result = STATE(stoppingToRetry ? STATE_STOPPING_RETRY : STATE_STOPPING);
    }
  }

  else if (IS_CURRENT_STATE(STATE_STOPPING)) {
    if (CURRENT_TIME - stoppingTime >= g_timing_config.stopping_time_ms) {
      if (is_stopped()) {
        result = STATE(STATE_STOPPED);
      } else if (is_running()) {
        result = STATE(STATE_STARTED);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_STOPPING_RETRY)) {
    if (CURRENT_TIME - stoppingTime >= g_timing_config.stopping_time_ms) {
      if (is_stopped()) {
        result = STATE(STATE_STOPPED_RETRY);
      } else if (is_running()) {
        result = STATE(STATE_STARTED);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_STOPPED_RETRY)) {
    if (startTry >= g_timing_config.max_start_try) {
      if (CURRENT_TIME - stoppingTime >= g_timing_config.stopping_time_ms) {
        result = STATE(STATE_STOPPING_LOAD_OFF);
      }
    } else if (CURRENT_TIME - startRetryTime >= g_timing_config.start_retry_time_ms) {
      if (is_stopped()) {
        result = STATE(STATE_STARTING_RETRY_TRY);
      } else if (is_running()) {
        result = STATE(STATE_STARTED);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_STOPPED)) {
    if (CURRENT_TIME - stoppingTime >= g_timing_config.stopping_time_ms) {
      if (is_stopped()) {
        result = STATE(STATE_IDLE);
      } else if (is_running()) {
        result = STATE(STATE_STARTED);
      }
    }
  }

  else if (IS_CURRENT_STATE(STATE_ERROR)) {
    if (is_stopped()) {
      result = STATE(STATE_IDLE);
    }
  }

  return result;
}

// Helper: Send output command to shift register task
void sendOutputCommand(OutputCommandType_t type, uint8_t reg, bool state) {
    if (xOutputCommandQueue != NULL) {
        OutputCommand_t cmd = {type, reg, state, 0};
        xQueueSend(xOutputCommandQueue, &cmd, 0);
    }
}

// Helper: Send sync command with full state
void sendSyncCommand(uint16_t full_state) {
    if (xOutputCommandQueue != NULL) {
        OutputCommand_t cmd = {CMD_REG_SYNC, 0, false, full_state};
        xQueueSend(xOutputCommandQueue, &cmd, 0);
    }
}

// RTOS-safe versions of the remaining REG macros for use from tasks.
// RTOS_REG_ON itself moved to shared_defines.h (2026-09-25) — oled_menu.cpp
// needed the identical macro too and used to carry its own hand-duplicated
// copy; OFF/SET have no other caller yet, so they stay local here.
#define RTOS_REG_OFF(REG) sendOutputCommand(CMD_REG_OFF, REG, false)
#define RTOS_REG_SET(REG, STATE) sendOutputCommand(CMD_REG_SET, REG, STATE)

// ----------------------------------------------------------------------------
// StateMachineTask: Generator state machine (highest priority)
// ----------------------------------------------------------------------------
void StateMachineTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset
    WebCommand_t web_cmd;

    SLOG_PRINTLN("StateMachineTask started");

    for (;;) {
        // Wait for events with timeout
        (void)xEventGroupWaitBits(
            xStateEvents,
            EVENT_SENSOR_UPDATE | EVENT_POWER_UPDATE | EVENT_SHIFT_REG_UPDATE |
            EVENT_START_REQUESTED | EVENT_STOP_REQUESTED,
            pdTRUE,   // Clear bits on exit
            pdFALSE,  // Wait for any bit
            pdMS_TO_TICKS(100)  // Timeout
        );

        // Process web commands
        while (xQueueReceive(xWebCommandQueue, &web_cmd, 0) == pdTRUE) {
            switch (web_cmd.type) {
                case WEB_CMD_START:
                    if (is_not_working() && !safety_start_inhibited()) {
                        FramLogRecord r = {};
                        r.type = FLOG_EV_START_REQUEST;
                        r.d.request.source = web_cmd.source;
                        fram_log_event(&r);
                        STATE(STATE_STARTING_SINGLE);
                    }
                    break;
                case WEB_CMD_START_RETRY:
                    if (is_not_working() && !safety_start_inhibited()) {
                        FramLogRecord r = {};
                        r.type = FLOG_EV_START_REQUEST;
                        r.d.request.source = web_cmd.source;
                        r.d.request.retry = 1;
                        fram_log_event(&r);
                        STATE(STATE_STARTING_RETRY);
                    }
                    break;
                case WEB_CMD_STOP:
                    flog_last_stop_req_ms = uptime_ms();
                    if (!is_not_working()) {
                        FramLogRecord r = {};
                        r.type = FLOG_EV_STOP_REQUEST;
                        r.d.request.source = web_cmd.source;
                        fram_log_event(&r);
                    }
                    // A STOP received mid-crank on a retry attempt must cancel the
                    // whole retry campaign, not just this attempt: stoppingToRetry
                    // (set in switch_STATE_STOPPING_LOAD_OFF() from the state we're
                    // leaving) can't distinguish "operator said stop" from "this
                    // attempt's own timeout expired" — both leave from
                    // STATE_STARTING_RETRY(_TRY) the same way, so without this,
                    // STATE_STOPPED_RETRY's own startTry<max_start_try check (which
                    // is correct and untouched here) would auto-resume cranking
                    // right after the "stop". Forcing startTry to the limit here
                    // makes that same check take its already-correct
                    // retries-exhausted branch instead. Found live 2026-09-18.
                    if (IS_CURRENT_STATE(STATE_STARTING_RETRY) || IS_CURRENT_STATE(STATE_STARTING_RETRY_TRY)) {
                        startTry = g_timing_config.max_start_try;
                    }
                    // Always enter the full stop sequence, including from
                    // IDLE: the engine may have been started by hand (or
                    // is_running() missed RPM/OUTPUT) while the controller
                    // still thinks it's idle — Stop must still drop the
                    // load and fire ignition-kill. Do not gate this on
                    // is_not_working().
                    STATE(STATE_STOPPING_LOAD_OFF);
                    break;
                case WEB_CMD_REG_TOGGLE:
                    if (web_cmd.reg < 16 && web_cmd.reg != REG_START) {
                        // Block FUEL_PUMP toggle if it would turn it ON and fuel level >= g_engine_config.fuel_pump_stop_level
                        if (web_cmd.reg == REG_FUEL_PUMP &&
                            IS_REG_OFF(REG_FUEL_PUMP) &&
                            fuel_level_from_adc(adc_2_0.get()) >= g_engine_config.fuel_pump_stop_level) {
                            SDEBUG("[FUEL] FUEL_PUMP toggle blocked — fuel level ",
                                   dbg_fixed((double)fuel_level_from_adc(adc_2_0.get()), 1),
                                   " >= ", dbg_fixed((double)g_engine_config.fuel_pump_stop_level, 1));
                            break;
                        }
                        sendOutputCommand(CMD_REG_TOGGLE, web_cmd.reg, false);
                    } else if (web_cmd.reg == REG_START) {
                        SDEBUG("[SAFETY] REG_START toggle blocked — only switch_STATE_STARTING*() may drive the starter");
                    }
                    break;
                case WEB_CMD_REG_SET:
                    // REG_START is excluded from every manual command surface (HTTP /reg,
                    // raw WebSocket numeric command, MQTT, OLED menu — this switch is the
                    // one funnel point all four go through). Cranking the starter is only
                    // ever safe from inside switch_STATE_STARTING()'s own sequencing (choke
                    // coordination, retry counts, safety-limit checks already done by
                    // process_state_check() before it gets here) — an unauthenticated
                    // GET /reg?reg=11&set=1 or a raw "11" over the dashboard's own WS port
                    // could otherwise crank the starter at any time, including into an
                    // already-spinning flywheel while RUNNING (bendix-into-ring-gear damage,
                    // not something a later state-machine correction can undo the way a
                    // stray REG_IGNITION toggle's stall-detection safety net can). Every
                    // other register here is already either a normal dashboard switch
                    // (Enable_battery/Disable_battery/Ignition/ECO/Load_switch/Charger/Choke/
                    // Fuel_pump — see data/dashboard.json) or inert (buzzer/LEDs/reserved),
                    // so only REG_START needed blocking. Found live 2026-09-18.
                    if (web_cmd.reg < 16 && web_cmd.reg != REG_START) {
                        // Block FUEL_PUMP turn-on if fuel level >= g_engine_config.fuel_pump_stop_level
                        if (web_cmd.reg == REG_FUEL_PUMP && web_cmd.state &&
                            fuel_level_from_adc(adc_2_0.get()) >= g_engine_config.fuel_pump_stop_level) {
                            SDEBUG("[FUEL] FUEL_PUMP set=ON blocked — fuel level ",
                                   dbg_fixed((double)fuel_level_from_adc(adc_2_0.get()), 1),
                                   " >= ", dbg_fixed((double)g_engine_config.fuel_pump_stop_level, 1));
                            break;
                        }
                        sendOutputCommand(CMD_REG_SET, web_cmd.reg, web_cmd.state);
                    } else if (web_cmd.reg == REG_START) {
                        SDEBUG("[SAFETY] REG_START set blocked — only switch_STATE_STARTING*() may drive the starter");
                    }
                    break;
            }
        }

        // Process current state
        process_current_state();

        // Safety: auto-disable FUEL_PUMP if fuel level reached g_engine_config.fuel_pump_stop_level
        if (IS_REG_ON(REG_FUEL_PUMP) &&
            fuel_level_from_adc(adc_2_0.get()) >= g_engine_config.fuel_pump_stop_level) {
            SDEBUG("[FUEL] FUEL_PUMP auto-OFF — fuel level ",
                   dbg_fixed((double)fuel_level_from_adc(adc_2_0.get()), 1),
                   " >= ", dbg_fixed((double)g_engine_config.fuel_pump_stop_level, 1));
            sendOutputCommand(CMD_REG_OFF, REG_FUEL_PUMP, false);
        }

        // Safety: auto-disable FUEL_PUMP if it's been running for
        // g_engine_config.fuel_pump_no_rise_timeout_s with no meaningful rise
        // in fuel level — catches a pump running dry/disconnected (external
        // supply empty, hose off, etc.) that would otherwise never trip the
        // fuel_pump_stop_level check above, since the tank never actually fills.
        // 0 = disabled (same convention as g_limits_config's temp/power limits
        // above) — otherwise uptime_ms()-fuel_pump_last_rise_ms >= 0 is trivially
        // true on the very next iteration, tripping instantly instead of never.
        if (IS_REG_ON(REG_FUEL_PUMP) && g_engine_config.fuel_pump_no_rise_timeout_s > 0) {
            float level = (float)fuel_level_from_adc(adc_2_0.get());
            if (fuel_pump_last_rise_ms == 0 ||
                level >= fuel_pump_last_rise_level + FUEL_PUMP_RISE_HYSTERESIS_L) {
                fuel_pump_last_rise_ms = uptime_ms();
                fuel_pump_last_rise_level = level;
            } else if (uptime_ms() - fuel_pump_last_rise_ms >=
                       g_engine_config.fuel_pump_no_rise_timeout_s * 1000UL) {
                SDEBUG("[FUEL] FUEL_PUMP auto-OFF — no level rise for ",
                       (g_engine_config.fuel_pump_no_rise_timeout_s),
                       "s (stuck at ", dbg_fixed((double)level, 1), "L)");
                sendOutputCommand(CMD_REG_OFF, REG_FUEL_PUMP, false);
                FramLogRecord r = {};
                r.type = FLOG_EV_ERROR;
                r.d.error.code = 2;  // 2 = fuel pump no-rise timeout, auto-off
                r.d.error.value = level;
                fram_log_event(&r);
            }
        } else {
            fuel_pump_last_rise_ms = 0;
        }

        // Safety: CHOKE and FUEL_PUMP both need the battery connected to
        // operate (choke solenoid hold-in power, fuel pump motor power). If
        // either is switched on (typically a manual toggle — e.g. running the
        // pump to refill the tank while stopped) while REG_IN_BAT_SENSE is
        // absent, force the battery on for as long as either needs it, then
        // release once neither does. Skipped entirely while RUNNING: the
        // generator's own output already covers both, so forcing the battery
        // is unnecessary there (see switch_STATE_RUNNING()). Independent of
        // choke_ensure_battery()/choke_release_battery(), which separately
        // covers the automatic cold-start choke sequence with its own
        // settling-time wait — this only handles the general/manual case, and
        // the two are never allowed to release what the other one forced
        // (see switch_STATE_STARTED()/process_STATE_RUNNING()).
        if (!IS_CURRENT_STATE(STATE_RUNNING)) {
            bool battery_dependent_reg_on = IS_REG_ON(REG_CHOKE) || IS_REG_ON(REG_FUEL_PUMP);
            if (battery_dependent_reg_on && IS_REG_IN_OFF(REG_IN_BAT_SENSE) &&
                IS_REG_OFF(REG_BATT_FORCE)) {
                SDEBUG("[BAT] forcing battery on — CHOKE/FUEL_PUMP active, battery not sensed");
                REG_ON(REG_BATT_FORCE);
                battery_dependency_forced = true;
            } else if (battery_dependency_forced && !battery_dependent_reg_on) {
                SDEBUG("[BAT] releasing battery — CHOKE and FUEL_PUMP both off");
                REG_OFF(REG_BATT_FORCE);
                REG_ON(REG_BATT_OFF);
                unsigned long long waitTime = CURRENT_TIME;
                while ((CURRENT_TIME - waitTime <= 1000) && IS_REG_IN_ON(REG_IN_BAT_SENSE)) {
                    read_regs();
                    esp_task_wdt_reset();
                    vTaskDelay(pdMS_TO_TICKS(1));
                }
                REG_OFF(REG_BATT_OFF);
                battery_dependency_forced = false;
            }
        }

        // Safety: hardware protection inputs (overheat/oil/overload) force a
        // stop while the engine is running — these normally kill the engine
        // via their own hardware circuit too, but this guarantees the
        // firmware also reacts (load-off-first) rather than just logging it
        // after the fact once the engine has already died.
        if ((IS_CURRENT_STATE(STATE_STARTED) || IS_CURRENT_STATE(STATE_RUNNING)) &&
            (IS_REG_IN_ON(REG_IN_OVERHEAT) || IS_REG_IN_ON(REG_IN_OIL) ||
             IS_REG_IN_ON(REG_IN_OVERLOAD))) {
            SDEBUG("[SAFETY] protection input active — forcing stop");
            STATE(STATE_STOPPING_LOAD_OFF);
        }

        // Safety: configurable temperature limits (0 = disabled) force a stop
        if (IS_CURRENT_STATE(STATE_STARTED) || IS_CURRENT_STATE(STATE_RUNNING)) {
            bool over_limit =
                (g_limits_config.temp_air_max      > 0.0f && (float)adc_1_1.get() >= g_limits_config.temp_air_max)      ||
                (g_limits_config.temp_oil_max      > 0.0f && (float)adc_1_0.get() >= g_limits_config.temp_oil_max)      ||
                (g_limits_config.temp_engine_max   > 0.0f && flog_head_temp()     >= g_limits_config.temp_engine_max)   ||
                (g_limits_config.temp_inverter_max > 0.0f && (float)adc_1_2.get() >= g_limits_config.temp_inverter_max);
            if (over_limit) {
                SDEBUG("[SAFETY] temperature limit exceeded — forcing stop");
                flog_pending_temp_limit_stop = true;
                STATE(STATE_STOPPING_LOAD_OFF);
            }
        }

        // Safety: configurable active-power limit (0 = disabled) drops only
        // the load relay — the engine keeps running. Level-triggered (like
        // the FUEL_PUMP check above): only acts while the relay is actually
        // on, so it won't fight a deliberate reconnect unless still overloaded.
        if (g_limits_config.power_max > 0.0f && IS_REG_ON(REG_LOAD_RELAY)) {
            portENTER_CRITICAL(&g_state_spinlock);
            float active_power = g_state.power.active_power;
            portEXIT_CRITICAL(&g_state_spinlock);
            if (active_power >= g_limits_config.power_max) {
                SDEBUG("[SAFETY] power limit exceeded — dropping load relay");
                sendOutputCommand(CMD_REG_OFF, REG_LOAD_RELAY, false);
                FramLogRecord r = {};
                r.type = FLOG_EV_ERROR;
                r.d.error.code = 1;  // 1 = active-power limit exceeded, load relay dropped
                r.d.error.value = active_power;
                fram_log_event(&r);
            }
        }

        // Check for state transitions
        process_state_check();

        // Feed engine-hours accumulation in the FRAM log task
        fram_log_set_running(IS_CURRENT_STATE(STATE_STARTED) ||
                             IS_CURRENT_STATE(STATE_RUNNING));

        // Update shared state
        portENTER_CRITICAL(&g_state_spinlock);
        g_state.current_state = runtimeState;
        g_state.previous_state = prevState;
        portEXIT_CRITICAL(&g_state_spinlock);

        // Feed watchdog
        esp_task_wdt_reset();
    }
}

// Human-readable state name for MQTT publish. Non-static: prototype in
// rtos_config.h — oled_menu.cpp's status screen and mqtt.cpp's state
// publish both call this directly.
const char *state_name(state s) {
    switch (s) {
        case STATE_INIT:               return "init";
        case STATE_IDLE:                return "idle";
        case STATE_STOPPING_LOAD_OFF:  return "stopping";
        case STATE_STOPPING:           return "stopping";
        case STATE_STOPPING_RETRY:     return "stopping_retry";
        case STATE_STOPPED:            return "stopped";
        case STATE_STOPPED_RETRY:      return "stopped_retry";
        case STATE_STARTING_SINGLE:    return "starting";
        case STATE_STARTING:           return "starting";
        case STATE_STARTING_RETRY:     return "starting_retry";
        case STATE_STARTING_RETRY_TRY: return "starting_retry";
        case STATE_STARTED:            return "started";
        case STATE_RUNNING:            return "running";
        case STATE_ERROR:              return "error";
        default:                       return "unknown";
    }
}
