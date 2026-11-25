#pragma once

#include <stdint.h>
#include <stddef.h>
#include <ArduinoJson.h>

// WiFi config persistence: /wifi.json on FFat. Needed by both the main .cpp
// (wifi_config_load()/save(), update_mode()'s own reduced /api/wifi/save)
// and web.cpp (setup_server_routes_ws()'s /api/wifi/config, /api/wifi/save,
// parse_wifi_core_fields()) — moved here 2026-09-24 from the main .cpp so
// both translation units see the same values.
#define WIFI_JSON_PATH       "/wifi.json"
#define WIFI_JSON_SIZE       4096
#define WIFI_NETWORKS_MAX    4

// Configurable-at-runtime settings (timing/battery/engine/time), loaded from
// /wifi.json at boot. Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp
// (see CLAUDE.md's "Code organization" and [[main-cpp-modularization]]
// project memory for the pattern this follows). LimitsConfig_t/g_limits_config
// already live in rtos_config.h (moved earlier, same reason) — limits_config_load()
// is grouped here anyway since all five *_config_load() functions are one
// conceptual unit, always called together from the main .cpp's config-apply path.

// ============================================================================
// Engine Timing Configuration (start/stop sequence durations, retry count)
// ============================================================================
struct TimingConfig_t {
    uint8_t  max_start_try;            // auto-retry crank attempts before giving up (was START_TRY)
    uint32_t start_time_check_ms;      // time into a crank before RPM is first checked (was START_TIME_CHECK)
    uint32_t start_time_ms;            // crank duration / choke+battery-force hold time (was START_TIME)
    uint32_t start_retry_time_ms;      // gap between auto-retry crank attempts (was START_RETRY_TIME)
    uint32_t stopping_time_ms;         // time allotted for the engine to actually stop (was STOPPING_TIME)
    uint32_t running_time_ms;          // time allotted to reach running RPM (was RUNNING_TIME)
    uint32_t load_relay_stop_delay_ms; // load relay settle time before ignition cut (was LOAD_RELAY_STOP_DELAY)
    uint32_t after_run_period_ms;      // fast battery-read window after the engine stops (was AFTER_RUN_PERIOD_TIME)
};
extern TimingConfig_t g_timing_config;

// ============================================================================
// Battery Configuration (voltage thresholds + measurement read periods)
// ============================================================================
struct BatteryConfig_t {
    float    threshold_start_charge_v;   // bat_v at/below which charging starts (was BAT_THRESHOLD_START_CHARGE_VOLTAGE)
    float    threshold_stop_charge_v;    // bat_v at/above which charging stops (was BAT_THRESHOLD_STOP_CHARGE_VOLTAGE)
    float    threshold_starting_low_v;   // reserved/currently unused in logic, kept for parity (was BAT_THRESHOLD_STARTING_LOW_VOLTAGE)
    float    threshold_shutdown_v;       // bat_v at/below which battery-only supply is shut down (was BAT_THRESHOLD_SHUTDOWN_VOLTAGE)
    uint32_t shutdown_persist_time_s;    // seconds bat_v must stay <= threshold_shutdown_v before actually shutting down; 0 = shut down immediately (no debounce)
    uint32_t start_charge_persist_time_s; // seconds bat_v must stay <= threshold_start_charge_v before actually enabling the charger; 0 = start immediately (no debounce)
    uint32_t read_period_ms;             // slow idle read period (was READ_BAT_PERIOD)
    uint32_t read_starting_period_ms;    // read period while cranking (was READ_BAT_STARTING_PERIOD)
    uint32_t read_running_period_ms;     // read period while running (was READ_BAT_RUNNING_PERIOD)
    uint32_t read_after_run_period_ms;   // fast read period just after stopping (was READ_BAT_AFTER_RUN_PERIOD)
    uint32_t read_charging_period_ms;    // read period while actively charging (was READ_BAT_CHARGING_PERIOD)
    bool     psu_loss_backup_enabled;    // force REG_BATT_FORCE on if PSU_SENSE AND GEN_SENSE both drop while idle (opt-in) — name kept for config-file compat, see psu_backup_check()
};
extern BatteryConfig_t g_battery_config;

// ============================================================================
// Engine Detection Configuration (RPM thresholds + fuel pump auto-cutoff)
// ============================================================================
struct EngineConfig_t {
    uint32_t rpm_max;                // sanity clamp before smoothing accepts a reading (was RPM_MAX)
    uint32_t rpm_starting;           // combined with rpm_running_threshold to detect "running" (was RPM_STARTING)
    uint32_t rpm_running;            // currently unused in logic, kept for parity (was RPM_RUNNING)
    uint32_t rpm_starting_threshold; // RPM range considered "cranking"; currently unused in logic since is_starting() (its only reader) was dead code — kept for config-persistence parity (was RPM_STARTING_THRESHOLD)
    uint32_t rpm_running_threshold;  // RPM above which is_running() reports true (was RPM_RUNNING_THRESHOLD)
    float    fuel_pump_stop_level;   // fuel level, L, at/above which REG_FUEL_PUMP is auto-disabled (was FUEL_PUMP_STOP_LEVEL)
    uint32_t fuel_pump_no_rise_timeout_s; // seconds REG_FUEL_PUMP can run with no meaningful fuel-level rise before auto-off (stuck/dry/disconnected pump)
};
extern EngineConfig_t g_engine_config;

// ============================================================================
// Time zone configuration (fixed UTC offset, no DST rules)
// ============================================================================
struct TimeConfig_t {
    int8_t  tz_offset_hours;    // -12..14
    uint8_t tz_offset_minutes;  // 0..59
};
extern TimeConfig_t g_time_config;

// Each parses one top-level key of a wifi.json document into its matching
// g_*_config global above (limits_config_load() into rtos_config.h's
// g_limits_config); called together from the main .cpp's config-apply path
// at boot/config-save. Missing key/field = compiled-in default, not an error.
void limits_config_load(const JsonDocument &doc);
void timing_config_load(const JsonDocument &doc);
void battery_config_load(const JsonDocument &doc);
void engine_config_load(const JsonDocument &doc);
void time_config_load(const JsonDocument &doc);

// Builds a POSIX TZ string from g_time_config and applies it (setenv+tzset).
// Called by time_config_load() itself and once more directly from the main
// .cpp's setup_ntp() (covers the config-portal path, which never reaches
// time_config_load()).
void apply_timezone(void);
