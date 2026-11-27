/*
 * config.cpp — timing/battery/engine/time config, loaded from /wifi.json.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code
 * organization" section and [[main-cpp-modularization]] project memory for
 * the pattern this follows). Moved verbatim.
 */

#include "config.h"
#include "rtos_config.h"  // g_limits_config
#include "fram_log.h"      // FLOG_MAINT_TYPES, fram_log_maint_name()

// StateMachineTask's own runtime "current battery read period" cache, still
// defined in the main .cpp (it's read/written from ~20 call sites there,
// none of them config-related) — battery_config_load() below just needs
// write access to resync it after a config change.
extern unsigned long long readBatPeriod;

TimingConfig_t g_timing_config = {
    .max_start_try            = 5,
    .start_time_check_ms      = 2000,
    .start_time_ms            = 4000,
    .start_retry_time_ms      = 15000,
    .stopping_time_ms         = 5000,
    .running_time_ms          = 5000,
    .load_relay_stop_delay_ms = 500,
    .after_run_period_ms      = 1000UL * 60 * 5,
    .cooldown_enabled         = false,
    .cooldown_temp_c          = 140.0f,
    .cooldown_min_s           = 30,
    .cooldown_max_s           = 90,
};

BatteryConfig_t g_battery_config = {
    .threshold_start_charge_v  = 12.3f,
    .threshold_stop_charge_v   = 14.2f,
    .threshold_starting_low_v  = 7.5f,
    .threshold_shutdown_v      = 12.0f,
    .shutdown_persist_time_s   = 300,
    .start_charge_persist_time_s = 0,
    .read_period_ms            = 1000UL * 60 * 60 * 12,
    .read_starting_period_ms   = 1000UL * 5,
    .read_running_period_ms    = 1000UL * 60 * 1,
    .read_after_run_period_ms  = 1000UL * 5,
    .read_charging_period_ms   = 1000UL * 60 * 5,
    .psu_loss_backup_enabled   = false,
    .psu_autostart_enabled     = false,
    .psu_autostart_delay_s     = 10,
    .psu_autostop_delay_s      = 120,
};

EngineConfig_t g_engine_config = {
    .rpm_max                     = 4000,
    .rpm_starting                = 1500,
    .rpm_running                 = 2785,
    .rpm_starting_threshold      = 300,
    .rpm_running_threshold       = 800,
    .fuel_pump_stop_level        = 8.0f,
    .fuel_pump_no_rise_timeout_s = 300,
};

TimeConfig_t g_time_config = {
    .tz_offset_hours   = 3,
    .tz_offset_minutes = 0,
};

// Load safety-limits config from wifi.json document. Missing/0 = disabled.
void limits_config_load(const JsonDocument &doc) {
    JsonObjectConst lim = doc["limits"];
    if (lim.isNull()) return;
    g_limits_config.temp_air_max      = lim["temp_air_max"]      | 0.0f;
    g_limits_config.temp_oil_max      = lim["temp_oil_max"]      | 0.0f;
    g_limits_config.temp_engine_max   = lim["temp_engine_max"]   | 0.0f;
    g_limits_config.temp_inverter_max = lim["temp_inverter_max"] | 0.0f;
    g_limits_config.power_max         = lim["power_max"]         | 0.0f;
    g_limits_config.auto_load_relay_on_run = lim["auto_load_relay_on_run"] | true;
    g_limits_config.choke_cold_start_temp  = lim["choke_cold_start_temp"]  | 5.0f;
    JsonObjectConst maint_int = lim["maint_interval"];
    for (uint8_t i = 0; i < FLOG_MAINT_TYPES; i++) {
        g_limits_config.maint_interval_hours[i] =
            maint_int.isNull() ? 0.0f : (maint_int[fram_log_maint_name(i)] | 0.0f);
    }
}

// Load engine start/stop timing config from wifi.json document.
void timing_config_load(const JsonDocument &doc) {
    JsonObjectConst t = doc["timing"];
    if (t.isNull()) return;
    g_timing_config.max_start_try            = t["max_start_try"]            | 5;
    g_timing_config.start_time_check_ms      = t["start_time_check_ms"]      | 2000;
    g_timing_config.start_time_ms            = t["start_time_ms"]            | 4000;
    g_timing_config.start_retry_time_ms      = t["start_retry_time_ms"]      | 15000;
    g_timing_config.stopping_time_ms         = t["stopping_time_ms"]         | 5000;
    g_timing_config.running_time_ms          = t["running_time_ms"]         | 5000;
    g_timing_config.load_relay_stop_delay_ms = t["load_relay_stop_delay_ms"] | 500;
    g_timing_config.after_run_period_ms      = t["after_run_period_ms"]      | (1000UL * 60 * 5);
    g_timing_config.cooldown_enabled         = t["cooldown_enabled"]         | false;
    g_timing_config.cooldown_temp_c          = t["cooldown_temp_c"]          | 140.0f;
    g_timing_config.cooldown_min_s           = t["cooldown_min_s"]           | 30;
    g_timing_config.cooldown_max_s           = t["cooldown_max_s"]           | 90;
}

// Load battery voltage-threshold / read-period config from wifi.json document.
void battery_config_load(const JsonDocument &doc) {
    JsonObjectConst b = doc["battery"];
    if (b.isNull()) return;
    g_battery_config.threshold_start_charge_v = b["threshold_start_charge_v"] | 12.3f;
    g_battery_config.threshold_stop_charge_v  = b["threshold_stop_charge_v"]  | 14.2f;
    g_battery_config.threshold_starting_low_v = b["threshold_starting_low_v"] | 7.5f;
    g_battery_config.threshold_shutdown_v     = b["threshold_shutdown_v"]     | 12.0f;
    g_battery_config.shutdown_persist_time_s  = b["shutdown_persist_time_s"]  | 300;
    g_battery_config.start_charge_persist_time_s = b["start_charge_persist_time_s"] | 0;
    g_battery_config.read_period_ms           = b["read_period_ms"]           | (1000UL * 60 * 60 * 12);
    g_battery_config.read_starting_period_ms  = b["read_starting_period_ms"]  | (1000UL * 5);
    g_battery_config.read_running_period_ms   = b["read_running_period_ms"]   | (1000UL * 60 * 1);
    g_battery_config.read_after_run_period_ms = b["read_after_run_period_ms"] | (1000UL * 5);
    g_battery_config.read_charging_period_ms  = b["read_charging_period_ms"]  | (1000UL * 60 * 5);
    g_battery_config.psu_loss_backup_enabled  = b["psu_loss_backup_enabled"]  | false;
    g_battery_config.psu_autostart_enabled    = b["psu_autostart_enabled"]    | false;
    g_battery_config.psu_autostart_delay_s    = b["psu_autostart_delay_s"]    | 10;
    g_battery_config.psu_autostop_delay_s     = b["psu_autostop_delay_s"]     | 120;
    // Sync the live read-period variable in case we're loading this after
    // boot has already picked a period based on the old (default) value.
    readBatPeriod = g_battery_config.read_period_ms;
}

// Load RPM-detection thresholds + fuel pump auto-cutoff level from wifi.json.
void engine_config_load(const JsonDocument &doc) {
    JsonObjectConst e = doc["engine"];
    if (e.isNull()) return;
    g_engine_config.rpm_max                = e["rpm_max"]                | 4000;
    g_engine_config.rpm_starting            = e["rpm_starting"]           | 1500;
    g_engine_config.rpm_running             = e["rpm_running"]            | 2785;
    g_engine_config.rpm_starting_threshold  = e["rpm_starting_threshold"] | 300;
    g_engine_config.rpm_running_threshold   = e["rpm_running_threshold"]  | 800;
    g_engine_config.fuel_pump_stop_level    = e["fuel_pump_stop_level"]   | 8.0f;
    g_engine_config.fuel_pump_no_rise_timeout_s = e["fuel_pump_no_rise_timeout_s"] | 300;
}

// Build a POSIX TZ string from the configured fixed UTC offset (no DST) and
// apply it. POSIX offsets are the inverse of the usual "UTC+N" convention:
// the value is what must be ADDED to local time to get UTC.
void apply_timezone(void) {
    char tzbuf[24];
    snprintf(tzbuf, sizeof(tzbuf), "UTC%+d:%02u",
             -(int)g_time_config.tz_offset_hours, g_time_config.tz_offset_minutes);
    setenv("TZ", tzbuf, 1);
    tzset();
}

// Load the fixed UTC offset from wifi.json (falling back to the compiled-in
// default if absent) and (re-)apply it, unconditionally — unlike the other
// *_config_load() functions, this one must run even on a fresh install with
// no "time" key yet, since it replaces the old unconditional setenv/tzset()
// call that used to live in setup_ntp().
void time_config_load(const JsonDocument &doc) {
    JsonObjectConst t = doc["time"];
    if (!t.isNull()) {
        g_time_config.tz_offset_hours   = t["tz_offset_hours"]   | 3;
        g_time_config.tz_offset_minutes = t["tz_offset_minutes"] | 0;
    }
    apply_timezone();
}
