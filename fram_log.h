/*
 * fram_log.h
 * Persistent event log + lifetime metrics on FM24V10 FRAM (128 KB, I2C).
 *
 * Memory map:
 *   0x00000            header (magic / version / layout)
 *   0x00040 / 0x000C0  metrics double buffer A/B (seq + CRC16, tear-proof)
 *   0x00200 - 0x1FFFF  event ring: 4080 fixed 32-byte records
 *
 * Records carry a monotonic seq; slot = (seq-1) % count, so the ring head
 * is recovered at boot by binary search for the seq discontinuity — no
 * stored head pointer to corrupt on power loss. A torn record write is
 * caught by its CRC16.
 *
 * Writers post to xLogQueue; I2CTask (the sole owner of the I2C bus in this
 * firmware) drains it via fram_log_service_tick(), called once per its own
 * tick, and also owns metrics accumulation. Reads (web UI) and admin ops
 * (wipe, set-metrics) go through i2c_run_on_owner() — see fram_log_read()/
 * fram_log_wipe()/fram_metrics_set() — since they need a real
 * synchronous answer instead of the fire-and-forget write path.
 */

#ifndef FRAM_LOG_H
#define FRAM_LOG_H

#include <stdint.h>
#include <stddef.h>
#include "fram_dev.h"

// ---------------------------------------------------------------------------
// Event types
// ---------------------------------------------------------------------------
enum : uint8_t {
    FLOG_EV_NONE           = 0,
    FLOG_EV_BOOT           = 1,
    FLOG_EV_TIME_SYNC      = 2,

    FLOG_EV_START_REQUEST  = 10,
    FLOG_EV_START_OK       = 11,
    FLOG_EV_START_FAIL     = 12,
    FLOG_EV_STOP_REQUEST   = 13,
    FLOG_EV_STOPPED        = 14,
    FLOG_EV_EMERGENCY_STOP = 15,

    FLOG_EV_CHARGE_START   = 20,
    FLOG_EV_CHARGE_STOP    = 21,

    FLOG_EV_ERROR          = 30,
    FLOG_EV_TASK_WDT       = 31,   // task watchdog fired (starved task names)
    FLOG_EV_FW_UPDATE      = 32,   // OTA firmware update completed (see /update handler)
    FLOG_EV_I2C_STUCK      = 33,   // I2CTask's sensor cache gone stale (see ReportTask)
    FLOG_EV_SENSOR_FAIL    = 34,   // a sensor read failed (see FLOG_SENSOR_* / i2c_read_sensors())
    FLOG_EV_PANIC_SUMMARY  = 35,   // esp_core_dump_get_summary() read at boot after a crash reboot

    FLOG_EV_MAINT          = 40,
};

// Command / request sources
enum : uint8_t {
    FLOG_SRC_UNKNOWN = 0,
    FLOG_SRC_HTTP,
    FLOG_SRC_WEBSOCKET,
    FLOG_SRC_MQTT,
    FLOG_SRC_PANEL,      // OLED menu / encoder
    FLOG_SRC_AUTO,
    FLOG_SRC_ESPNOW,     // ESP-NOW fallback link (esp_now_link.cpp)
};

// Emergency stop reasons
enum : uint8_t {
    FLOG_REASON_UNKNOWN = 0,
    FLOG_REASON_OIL,
    FLOG_REASON_OVERHEAT,
    FLOG_REASON_OVERLOAD,
    FLOG_REASON_STALLED,     // output lost, no protection input active (fuel out?)
    FLOG_REASON_TEMP_LIMIT,  // configured software temperature limit exceeded
};

// Charge stop reasons
enum : uint8_t {
    FLOG_CHG_THRESHOLD = 0,  // battery voltage reached stop threshold
    FLOG_CHG_PSU_LOST,
    FLOG_CHG_MANUAL,
    FLOG_CHG_ENGINE_START,
};

// Sensor IDs for FLOG_EV_SENSOR_FAIL (see i2c_read_sensors())
enum : uint8_t {
    FLOG_SENSOR_MCP9600 = 0,
    FLOG_SENSOR_INA233  = 1,
};

// Maintenance subtypes
enum : uint8_t {
    FLOG_MAINT_OIL = 0,
    FLOG_MAINT_SPARK,
    FLOG_MAINT_AIR_FILTER,
    FLOG_MAINT_FUEL_FILTER,
    FLOG_MAINT_VALVES,
    FLOG_MAINT_OTHER,
    FLOG_MAINT_TYPES         // count, keep last
};

#define FLOG_FLAG_TIME_VALID 0x01   // ts is real unix time (RTC/NTP synced)

// ---------------------------------------------------------------------------
// Log record: exactly 32 bytes
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
    uint32_t seq;      // monotonic, starts at 1; 0 = empty slot
    uint32_t ts;       // unix time
    uint8_t  type;     // FLOG_EV_*
    uint8_t  flags;    // FLOG_FLAG_*
    union __attribute__((packed)) {
        uint8_t raw[20];
        struct __attribute__((packed)) { uint8_t reset_reason; } boot;
        struct __attribute__((packed)) { uint32_t old_ts; uint32_t new_ts; } time_sync;
        struct __attribute__((packed)) { uint8_t source; uint8_t retry; } request;      // START/STOP_REQUEST
        struct __attribute__((packed)) { uint8_t attempts; uint8_t detected;            // detected=1: found running w/o start sequence
                                          float head_temp; float bat_v; uint32_t crank_ms; } start_ok;
        struct __attribute__((packed)) { uint8_t attempts; } start_fail;
        struct __attribute__((packed)) { uint32_t run_seconds; float head_temp; float fuel_l; } stopped;
        struct __attribute__((packed)) { uint8_t reason; uint8_t inputs;                 // raw 74HC165 snapshot
                                          float head_temp; uint32_t run_seconds; } emergency;
        struct __attribute__((packed)) { float bat_v; uint8_t auto_started; } charge_start;
        struct __attribute__((packed)) { float bat_v; uint32_t charge_seconds; uint8_t reason; } charge_stop;
        struct __attribute__((packed)) { uint16_t code; float value; } error;
        struct __attribute__((packed)) { uint16_t uptime_s; char tasks[18]; } wdt;  // uptime_s saturates at 65535 (~18h); tasks: no NUL if full
        struct __attribute__((packed)) { uint32_t size; char version[13]; } fw_update;  // version: "yyyymmddhhmm"
        struct __attribute__((packed)) { uint32_t stuck_ms; } i2c_stuck;
        struct __attribute__((packed)) { uint8_t sensor; } sensor_fail;  // FLOG_SENSOR_*
        struct __attribute__((packed)) { uint32_t pc; uint32_t exc_cause; uint32_t exc_vaddr;
                                          char task[8]; } panic;  // exc_cause: Xtensa EXCCAUSE register value; task: no NUL if full
        struct __attribute__((packed)) { uint8_t subtype; uint32_t engine_seconds; char note[13]; } maint;
    } d;
    uint16_t crc;      // CRC16-CCITT over bytes [0..29]
} FramLogRecord;

// ---------------------------------------------------------------------------
// Lifetime metrics (RAM copy; flushed to FRAM double buffer)
// ---------------------------------------------------------------------------
typedef struct {
    uint32_t seq;                 // flush counter, picks newer copy at boot
    uint32_t total_run_seconds;   // engine hours
    uint32_t total_charge_seconds;
    uint32_t start_count;
    uint32_t start_fail_count;
    uint32_t emergency_count;
    uint32_t charge_cycles;
    float    max_head_temp;
    float    max_rpm;
    float    max_power;
    uint32_t maint_seconds[FLOG_MAINT_TYPES];  // engine seconds at last maintenance
    uint32_t maint_ts[FLOG_MAINT_TYPES];       // unix time of last maintenance
    uint16_t crc;
} FramMetrics;

// Bitmask for FramMetricsPatch_t.fields — only set bits are written.
enum {
    FRAM_METRICS_SET_RUN_HOURS     = 1u << 0,
    FRAM_METRICS_SET_CHARGE_HOURS  = 1u << 1,
    FRAM_METRICS_SET_STARTS        = 1u << 2,
    FRAM_METRICS_SET_START_FAILS   = 1u << 3,
    FRAM_METRICS_SET_EMERGENCIES   = 1u << 4,
    FRAM_METRICS_SET_CHARGE_CYCLES = 1u << 5,
    FRAM_METRICS_SET_MAX_HEAD_TEMP = 1u << 6,
    FRAM_METRICS_SET_MAX_RPM       = 1u << 7,
    FRAM_METRICS_SET_MAX_POWER     = 1u << 8,
};

typedef struct {
    uint32_t fields;
    float    run_hours;
    float    charge_hours;
    uint32_t starts;
    uint32_t start_fails;
    uint32_t emergencies;
    uint32_t charge_cycles;
    float    max_head_temp;
    float    max_rpm;
    float    max_power;
} FramMetricsPatch_t;

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// Call once in setup() after fram.begin(). Verifies header (formats the log
// area on first use), locates the ring head, loads metrics. Returns false if
// FRAM is absent — all other calls become no-ops then.
bool fram_log_begin(FRAM32 *dev);

// Create xLogQueue + xLogMqttQueue. Call from setup_rtos(), before I2CTask
// is created (no task of its own any more — see fram_log_service_tick()).
void fram_log_start_task(void);

// Drains xLogQueue (writes + metrics accumulation/flush). Call once per
// I2CTask tick — I2CTask is the sole owner of the I2C bus in this firmware, so
// this replaces what used to be a dedicated LogTask + xI2CMutex pair.
void fram_log_service_tick(void);

// Log an event. Caller fills .type and .d; ts/flags/seq/crc are set here.
// Queued for fram_log_service_tick() once xLogQueue exists, written
// directly during setup() before that. Never blocks.
bool fram_log_event(FramLogRecord *rec);

// Manual maintenance entry: logs FLOG_EV_MAINT with current engine seconds
// and updates the per-type metrics slots.
bool fram_log_maintenance(uint8_t subtype, const char *note);

// Call once in setup() right after the FLOG_EV_BOOT record is logged. If a
// TWDT panic on the previous run left task-starvation info in RTC no-init
// memory (see esp_task_wdt_isr_user_handler in fram_log.cpp), logs it as
// FLOG_EV_TASK_WDT and clears the marker. No-op otherwise.
void fram_log_check_wdt_capture(void);

// Call once in setup(), right after fram_log_check_wdt_capture(). If this
// boot's esp_reset_reason() indicates a real crash (panic/watchdog — not a
// plain esp_restart()/power-on/deep-sleep/brownout), reads the coredump
// ESP-IDF's own coredump-to-flash already wrote via esp_core_dump_get_
// summary() (no .elf needed on-device — unsymbolized PC/cause/faulting
// address only, see decode_crash.sh/crash_archive/ for a real backtrace)
// and logs it as FLOG_EV_PANIC_SUMMARY. No-op if no valid coredump is
// present, or if this boot's reset reason doesn't indicate a crash — the
// coredump partition isn't cleared on read, so gating on THIS boot's own
// reset reason (not "a coredump merely exists") is what stops the same
// stale crash from being re-logged on every subsequent clean reboot.
void fram_log_check_panic_summary(void);

// Ring access (web UI). Seq range [oldest..newest], 0 = empty log.
uint32_t fram_log_newest_seq(void);
uint32_t fram_log_oldest_seq(void);
bool     fram_log_read(uint32_t seq, FramLogRecord *out);

// Erase event ring (and optionally metrics). Blocking, several seconds.
void fram_log_wipe(bool metrics_too);

// Snapshot of metrics (RAM copy, spinlock-protected).
FramMetrics fram_metrics_get(void);

// Engine-hours elapsed since a maintenance subtype was last recorded.
// Clamped to 0 instead of underflowing when maint_seconds[idx] > total_run_seconds
// (happens after fram_metrics_set_run_hours() lowers the lifetime counter below
// a mark that was already recorded, e.g. correcting hours post controller-swap) —
// unsigned subtraction would otherwise wrap to ~1.19M "hours since", falsely
// flagging every maintenance type as DUE everywhere (web banner, OLED, display
// mask). idx must be < FLOG_MAINT_TYPES (unchecked — caller's responsibility,
// same as every other maint_seconds[]/maint_ts[] access in this codebase).
double fram_maint_hours_since(const FramMetrics *m, uint8_t idx);

// Overwrite any subset of lifetime metrics (fields bitmask). Flushes to
// FRAM immediately. Hour/count/peak values must be >= 0; hours are stored
// as rounded seconds. Returns false if FRAM is absent, fields is 0, or a
// supplied value is negative.
bool fram_metrics_set(const FramMetricsPatch_t *patch);

// Overwrite lifetime engine hours only. Wrapper around fram_metrics_set().
bool fram_metrics_set_run_hours(float hours);

// Engine/charger state feeds for runtime accumulation (call on change or
// periodically; cheap).
void fram_log_set_running(bool running);
void fram_log_set_charging(bool charging);

// Peak trackers (call from sensor/power tasks; cheap, RAM only).
void fram_metrics_note_sensors(float head_temp, float rpm);
void fram_metrics_note_power(float watts);

// MQTT mirror: every written record is also pushed to a small ring queue;
// MqttTask drains it. Returns false when empty.
bool fram_log_pop_for_mqtt(FramLogRecord *out);

// JSON helpers
int fram_log_record_json(const FramLogRecord *r, char *buf, size_t n);
// maint_interval_hours: optional FLOG_MAINT_TYPES-sized array of overdue
// thresholds (engine hours; 0 = disabled for that type) — pass NULL to omit
// the "due" field entirely. Caller owns the array (see g_limits_config in
// the .ino); this file has no config-layer dependency of its own.
int fram_metrics_json(char *buf, size_t n, const float *maint_interval_hours = nullptr);
const char *fram_log_type_name(uint8_t type);
// Journal UI filter group for a type: "engine" / "charge" / "alarm" /
// "maint" / "sys" (unknown types → "sys"). Matches data/journal.html.gz's
// EV_META groups and GET /log?group=.
const char *fram_log_group_name(uint8_t type);
const char *fram_log_maint_name(uint8_t subtype);
// Parse maintenance subtype from name or numeric string; -1 if invalid.
int fram_log_maint_parse(const char *s);

#endif // FRAM_LOG_H
