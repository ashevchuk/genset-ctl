/*
 * fram_log.cpp
 * Persistent event log + lifetime metrics on FM24V10 FRAM. See fram_log.h.
 */

#include "fram_log.h"
#include "rtos_config.h"
#include "debug.h"
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <esp_attr.h>
#include <esp_system.h>
#include <esp_core_dump.h>
#include <time.h>

static_assert(sizeof(FramLogRecord) == 32, "FramLogRecord must be 32 bytes");

// ---------------------------------------------------------------------------
// Layout
// ---------------------------------------------------------------------------
#define FLOG_HEADER_ADDR   0x00000UL
#define FLOG_METRICS_A     0x00040UL
#define FLOG_METRICS_B     0x000C0UL
#define FLOG_LOG_START     0x00200UL
#define FLOG_LOG_END       0x20000UL
#define FLOG_REC_SIZE      32UL
#define FLOG_REC_COUNT     ((FLOG_LOG_END - FLOG_LOG_START) / FLOG_REC_SIZE)  // 4080

#define FLOG_MAGIC         0x474C4F31UL  // "GLO1"
#define FLOG_VERSION       1

static_assert(sizeof(FramMetrics) <= (FLOG_METRICS_B - FLOG_METRICS_A),
              "FramMetrics must fit metrics slot");

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t rec_size;
    uint32_t rec_count;
    uint32_t reserved;
} FlogHeader;

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static FRAM32 *s_fram = NULL;
static bool    s_present = false;

static portMUX_TYPE s_flog_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     s_next_seq = 1;       // seq of the NEXT record to write
static FramMetrics  s_metrics  = {};
static bool         s_metrics_dirty = false;
static uint8_t      s_metrics_slot  = 0;  // which slot the NEXT flush goes to

static volatile bool s_running_flag  = false;
static volatile bool s_charging_flag = false;

static QueueHandle_t xLogQueue     = NULL;
static QueueHandle_t xLogMqttQueue = NULL;

#define FLOG_QUEUE_LEN       16
#define FLOG_MQTT_QUEUE_LEN  8
#define FLOG_METRICS_FLUSH_MS 60000

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (uint16_t)(*data++) << 8;
        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint16_t rec_crc(const FramLogRecord *r) {
    return crc16_ccitt((const uint8_t *)r, offsetof(FramLogRecord, crc));
}

static uint16_t metrics_crc(const FramMetrics *m) {
    return crc16_ccitt((const uint8_t *)m, offsetof(FramMetrics, crc));
}

static inline uint32_t slot_addr(uint32_t slot) {
    return FLOG_LOG_START + slot * FLOG_REC_SIZE;
}
static inline uint32_t seq_slot(uint32_t seq) {
    return (seq - 1) % FLOG_REC_COUNT;
}

// Read only the seq field of a slot. Caller holds the bus.
static uint32_t seq_at(uint32_t slot) {
    uint32_t seq = 0;
    s_fram->read(slot_addr(slot), (uint8_t *)&seq, sizeof(seq));
    return seq;
}

static void stamp_time(FramLogRecord *r) {
    time_t t = time(NULL);
    r->ts = (uint32_t)t;
    r->flags = (t > 1600000000) ? FLOG_FLAG_TIME_VALID : 0;  // > Sep 2020 = synced
}

// ---------------------------------------------------------------------------
// Format / boot recovery
// ---------------------------------------------------------------------------
// These are rare, boot-time/admin operations (format-on-first-use, explicit
// wipe) — best-effort retry isn't worth the complexity here, but a failure
// should at least be visible instead of silently discarded.
static void wipe_log_area(void) {
    uint8_t zeros[64] = {};
    uint32_t failures = 0;
    uint32_t chunk = 0;
    // ~2040 chunks (FLOG_LOG area / 64 bytes) run inline on I2CTask when
    // called from job_fram_wipe() — ahead of that task's own
    // esp_task_wdt_reset() at the bottom of its loop, per-chunk I2C cost can
    // add up to a meaningful fraction of the 15s TWDT window on its own, and
    // any bus hiccup pushes it further. Pet it periodically so a slow/flaky
    // bus turns a wipe into "it took longer," not a panic/reboot mid-wipe.
    // Harmless when called from fram_log_begin() during setup() too (no task
    // registered yet at that point — esp_task_wdt_reset() is a no-op there).
    // Found live 2026-09-18.
    for (uint32_t a = FLOG_LOG_START; a < FLOG_LOG_END; a += sizeof(zeros)) {
        if (!s_fram->write(a, zeros, sizeof(zeros))) failures++;
        if (++chunk % 64 == 0) esp_task_wdt_reset();
    }
    if (failures) SLOG_PRINTF("[FLOG] wipe_log_area: %lu chunk(s) failed\n", (unsigned long)failures);
}

static void wipe_metrics_area(void) {
    uint8_t zeros[64] = {};
    uint32_t failures = 0;
    if (!s_fram->write(FLOG_METRICS_A, zeros, sizeof(zeros))) failures++;
    if (!s_fram->write(FLOG_METRICS_A + 64, zeros, sizeof(zeros))) failures++;
    if (!s_fram->write(FLOG_METRICS_B, zeros, sizeof(zeros))) failures++;
    if (!s_fram->write(FLOG_METRICS_B + 64, zeros, sizeof(zeros))) failures++;
    if (failures) SLOG_PRINTF("[FLOG] wipe_metrics_area: %lu chunk(s) failed\n", (unsigned long)failures);
}

static void write_header(void) {
    FlogHeader h = {};
    h.magic = FLOG_MAGIC;
    h.version = FLOG_VERSION;
    h.rec_size = FLOG_REC_SIZE;
    h.rec_count = FLOG_REC_COUNT;
    if (!s_fram->writeObject(FLOG_HEADER_ADDR, h)) {
        SLOG_PRINTLN("[FLOG] write_header failed");
    }
}

// Full scan fallback for a ring whose seq discipline is broken (torn write
// in exactly the wrong place). Returns max valid seq, 0 if empty.
static uint32_t scan_linear(void) {
    uint32_t best = 0;
    for (uint32_t slot = 0; slot < FLOG_REC_COUNT; slot++) {
        uint32_t seq = seq_at(slot);
        if (seq != 0 && seq != 0xFFFFFFFFUL && seq > best && seq_slot(seq) == slot)
            best = seq;
        // Boot-time usually (TWDT not registered → no-op), but also the
        // corrupt-ring fallback — 4080 FRAM reads is enough to matter if
        // somehow reached under a registered task.
        if ((slot & 63) == 63) esp_task_wdt_reset();
    }
    return best;
}

// Locate newest record. Slot layout slot=(seq-1)%N means seq values along the
// ring form one increasing run of step 1 starting at slot 0; binary-search its
// end: slot m belongs to the run iff seq[m] == seq[0] + m.
static uint32_t find_newest_seq(void) {
    uint32_t seq0 = seq_at(0);
    if (seq0 == 0 || seq0 == 0xFFFFFFFFUL) return 0;          // empty log
    if (seq_slot(seq0) != 0) return scan_linear();            // corrupt slot 0

    uint32_t lo = 0, hi = FLOG_REC_COUNT - 1;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo + 1) / 2;
        if (seq_at(mid) == seq0 + mid) lo = mid;
        else hi = mid - 1;
    }
    uint32_t newest = seq0 + lo;
    if (seq_slot(newest) != lo) return scan_linear();         // paranoia
    return newest;
}

static void metrics_load(void) {
    FramMetrics a = {}, b = {};
    s_fram->readObject(FLOG_METRICS_A, a);
    s_fram->readObject(FLOG_METRICS_B, b);
    bool a_ok = (a.seq != 0) && (a.crc == metrics_crc(&a));
    bool b_ok = (b.seq != 0) && (b.crc == metrics_crc(&b));

    if (a_ok && (!b_ok || a.seq >= b.seq)) {
        s_metrics = a;
        s_metrics_slot = 1;              // next flush overwrites the older copy
    } else if (b_ok) {
        s_metrics = b;
        s_metrics_slot = 0;
    } else {
        memset(&s_metrics, 0, sizeof(s_metrics));
        s_metrics_slot = 0;
    }
}

// Flush RAM metrics to the older FRAM slot. Caller holds the bus. Returns
// false (leaving s_metrics_dirty set and s_metrics_slot unrotated) on I2C
// failure, so the next dirty check retries the same slot instead of
// silently losing the update — writeObject() previously wasn't checked at
// all here, and s_metrics_dirty was cleared unconditionally before the
// write even happened.
static bool metrics_flush_locked(void) {
    FramMetrics m;
    portENTER_CRITICAL(&s_flog_mux);
    uint32_t seq = s_metrics.seq + 1;
    m = s_metrics;
    m.seq = seq;
    portEXIT_CRITICAL(&s_flog_mux);
    m.crc = metrics_crc(&m);

    if (!s_fram->writeObject(s_metrics_slot ? FLOG_METRICS_B : FLOG_METRICS_A, m)) {
        SLOG_PRINTLN("[FLOG] Metrics flush failed — will retry");
        return false;
    }

    portENTER_CRITICAL(&s_flog_mux);
    s_metrics.seq = seq;
    s_metrics_dirty = false;
    portEXIT_CRITICAL(&s_flog_mux);
    s_metrics_slot ^= 1;
    return true;
}

// ---------------------------------------------------------------------------
// Write path (LogTask or setup direct)
// ---------------------------------------------------------------------------

// Apply a record's side effects to the metrics counters.
static void metrics_apply_event(const FramLogRecord *r) {
    portENTER_CRITICAL(&s_flog_mux);
    switch (r->type) {
        case FLOG_EV_START_OK:       s_metrics.start_count++;      break;
        case FLOG_EV_START_FAIL:     s_metrics.start_fail_count++; break;
        case FLOG_EV_EMERGENCY_STOP: s_metrics.emergency_count++;  break;
        case FLOG_EV_CHARGE_START:   s_metrics.charge_cycles++;    break;
        case FLOG_EV_MAINT:
            if (r->d.maint.subtype < FLOG_MAINT_TYPES) {
                s_metrics.maint_seconds[r->d.maint.subtype] = r->d.maint.engine_seconds;
                s_metrics.maint_ts[r->d.maint.subtype] = r->ts;
            }
            break;
        default: break;
    }
    s_metrics_dirty = true;
    portEXIT_CRITICAL(&s_flog_mux);
}

// Assign seq, write record to its slot. Caller holds the bus. Returns
// false if the FRAM write itself failed — writeObject()'s result used to
// be discarded entirely here, silently losing the event with no trace
// anywhere (this is the journal used to diagnose exactly this kind of
// failure, so losing entries silently defeated the point). On failure,
// metrics/MQTT mirroring are skipped too, so a later successful retry of
// the same record (see LogTask) doesn't double-count.
//
// s_next_seq is only advanced AFTER a successful write, not before — it
// used to be pre-incremented, which burned a seq number on every failed
// attempt (and fram_log_service_tick()'s retry-same-record path re-derives
// r->seq from s_next_seq on every retry too, so a run of failures burned
// one seq per attempt, not just once). A burned seq leaves a permanent gap
// in the ring: find_newest_seq()'s binary search assumes an unbroken
// seq0+slot run, so a gap makes it stop at the last good slot before it and
// undercounts on the next boot — s_next_seq then gets reset too low, and
// subsequent writes silently overwrite slots that still held real,
// never-actually-lost history. Found live 2026-09-18. Re-deriving seq from
// s_next_seq fresh on every call (rather than trusting a seq the caller may
// already have set from a prior failed attempt) means every retry reuses
// the exact same seq/slot until it actually succeeds — no gap possible.
static bool write_record_locked(FramLogRecord *r) {
    uint32_t seq;
    portENTER_CRITICAL(&s_flog_mux);
    seq = s_next_seq;
    portEXIT_CRITICAL(&s_flog_mux);
    r->seq = seq;
    r->crc = rec_crc(r);
    if (!s_fram->writeObject(slot_addr(seq_slot(r->seq)), *r)) {
        SLOG_PRINTF("[FLOG] Write failed for seq=%lu (type=%u)\n",
                      (unsigned long)r->seq, r->type);
        return false;
    }

    portENTER_CRITICAL(&s_flog_mux);
    s_next_seq = seq + 1;
    portEXIT_CRITICAL(&s_flog_mux);

    metrics_apply_event(r);

    // Mirror for MQTT: ring semantics — drop oldest when full.
    if (xLogMqttQueue != NULL) {
        if (xQueueSend(xLogMqttQueue, r, 0) != pdTRUE) {
            FramLogRecord scrap;
            xQueueReceive(xLogMqttQueue, &scrap, 0);
            xQueueSend(xLogMqttQueue, r, 0);
        }
    }
    return true;
}

bool fram_log_event(FramLogRecord *rec) {
    if (!s_present) return false;
    stamp_time(rec);

    if (xLogQueue != NULL) {
        return xQueueSend(xLogQueue, rec, 0) == pdTRUE;
    }
    // Setup phase: direct write, single-threaded (no lock needed — this only
    // runs before I2CTask/xLogQueue exist at all).
    bool ok = write_record_locked(rec);
    if (ok) metrics_flush_locked();
    return ok;
}

bool fram_log_maintenance(uint8_t subtype, const char *note) {
    if (subtype >= FLOG_MAINT_TYPES) return false;
    FramLogRecord r = {};
    r.type = FLOG_EV_MAINT;
    r.d.maint.subtype = subtype;
    portENTER_CRITICAL(&s_flog_mux);
    r.d.maint.engine_seconds = s_metrics.total_run_seconds;
    portEXIT_CRITICAL(&s_flog_mux);
    if (note != NULL) {
        // Copy printable chars only; drop quote/backslash so the note can be
        // embedded in JSON output verbatim.
        size_t o = 0;
        for (const char *p = note; *p && o < sizeof(r.d.maint.note) - 1; p++) {
            if (*p >= 0x20 && *p != '"' && *p != '\\')
                r.d.maint.note[o++] = *p;
        }
        r.d.maint.note[o] = '\0';
    }
    return fram_log_event(&r);
}

// ---------------------------------------------------------------------------
// Service tick — called once per I2CTask tick (its sole owner). Used to be
// LogTask's own for(;;) loop body on a dedicated task; I2CTask is now the
// only thing that ever touches Wire, so this is just a plain function on
// its schedule instead of a separate task+mutex pair.
// ---------------------------------------------------------------------------
void fram_log_service_tick(void) {
    if (!s_present) return;
    // s_present alone doesn't guarantee xLogQueue exists yet — it's set by
    // fram_log_begin() (setup_fram(), early in setup()) while the queue
    // itself is only created later by fram_log_start_task() (setup_rtos()).
    // I2CTask (the only caller of this function) is expected to only start
    // after fram_log_start_task() has run — see its call site's own
    // comment — but this guard is what actually stops a NULL xQueueReceive()
    // assert/crash-loop if that ordering ever regresses again, instead of
    // just documenting the invariant and hoping.
    if (xLogQueue == NULL) return;

    bool wrote = false;

    // Accumulate engine / charger runtime from wall clock.
    static uint32_t last_tick_ms = 0;
    static uint32_t run_ms_accum = 0, charge_ms_accum = 0;
    uint32_t now_ms = millis();
    if (last_tick_ms == 0) last_tick_ms = now_ms;  // first call
    uint32_t elapsed = now_ms - last_tick_ms;
    last_tick_ms = now_ms;
    if (s_running_flag)  run_ms_accum    += elapsed;
    if (s_charging_flag) charge_ms_accum += elapsed;
    if (run_ms_accum >= 1000 || charge_ms_accum >= 1000) {
        portENTER_CRITICAL(&s_flog_mux);
        s_metrics.total_run_seconds    += run_ms_accum / 1000;
        s_metrics.total_charge_seconds += charge_ms_accum / 1000;
        s_metrics_dirty = true;
        portEXIT_CRITICAL(&s_flog_mux);
        run_ms_accum    %= 1000;
        charge_ms_accum %= 1000;
    }

    // Drain the event queue. No lock needed any more — I2CTask is the sole
    // caller, so a write can only fail because the device itself rejected
    // it, not because someone else holds the bus; requeue-and-retry-next-
    // tick covers that case the same way it always did.
    FramLogRecord rec;
    while (xQueueReceive(xLogQueue, &rec, 0) == pdTRUE) {
        if (write_record_locked(&rec)) {
            wrote = true;
        } else {
            xQueueSendToFront(xLogQueue, &rec, 0);
            break;
        }
    }

    // Flush metrics on event or periodically while dirty.
    static uint32_t last_flush_ms = 0;
    if (last_flush_ms == 0) last_flush_ms = now_ms;  // first call
    if (s_metrics_dirty &&
        (wrote || (now_ms - last_flush_ms >= FLOG_METRICS_FLUSH_MS))) {
        // Only push out last_flush_ms on success — on failure,
        // s_metrics_dirty stays set (see metrics_flush_locked) and this
        // keeps retrying sooner than a full FLOG_METRICS_FLUSH_MS instead
        // of waiting out the periodic timer again.
        if (metrics_flush_locked()) {
            last_flush_ms = now_ms;
        }
    }
}

void fram_log_start_task(void) {
    if (!s_present || xLogQueue != NULL) return;
    xLogQueue     = xQueueCreate(FLOG_QUEUE_LEN, sizeof(FramLogRecord));
    xLogMqttQueue = xQueueCreate(FLOG_MQTT_QUEUE_LEN, sizeof(FramLogRecord));
    if (xLogQueue == NULL || xLogMqttQueue == NULL) {
        SLOG_PRINTLN("ERROR: FRAM log queue creation failed!");
    }
}

// ---------------------------------------------------------------------------
// Task watchdog panic capture
//
// TWDT is configured with trigger_panic=true, so once it fires the chip goes
// down via abort()+reset — there is no safe window to touch the I2C bus
// (FRAM) from the ISR context that detects it. Instead, stash which task(s)
// starved into RTC "no-init" memory (survives the abort+reset, cleared only
// on power loss) and write the FLOG_EV_TASK_WDT record from setup() on the
// next boot, right next to the FLOG_EV_BOOT entry.
//
// This lives here rather than in the .ino because RTC_NOINIT_ATTR expands to
// __attribute__((section(...))), which confuses the Arduino builder's
// ctags-based auto-prototype scanner when it appears in a .ino file; plain
// .cpp files in the sketch folder aren't rewritten that way.
// ---------------------------------------------------------------------------
#define FLOG_WDT_CAPTURE_MAGIC 0x5744544Aul  // "WDTJ"

typedef struct {
    uint32_t magic;
    uint32_t uptime_s;
    char     tasks[18];  // best-effort text from esp_task_wdt_print_triggered_tasks; may not be NUL-terminated if full
} WdtCapture_t;

RTC_NOINIT_ATTR static WdtCapture_t s_wdt_capture;

// esp_task_wdt_print_triggered_tasks() calls the handler once per printed
// line; line 0 is always the fixed ~90-char preamble ("Task watchdog got
// triggered. The following tasks/users did not reset the watchdog in
// time:") which used to be all that fit in the old 16-byte capture buffer —
// making the captured text always identical and useless. Skip it and start
// collecting from line 1, which is the actual "- <task name> (CPU n)" list.
typedef struct { char *buf; size_t cap; size_t pos; int line; } WdtCaptureCtx;

static void wdt_capture_collect(void *opaque, const char *msg) {
    WdtCaptureCtx *ctx = (WdtCaptureCtx *)opaque;
    if (msg == NULL) return;
    if (ctx->line++ == 0) return;  // skip the fixed preamble line
    if (ctx->pos >= ctx->cap) return;
    size_t remain = ctx->cap - ctx->pos;
    size_t mlen = strlen(msg);
    size_t take = (mlen < remain) ? mlen : remain;
    memcpy(ctx->buf + ctx->pos, msg, take);
    ctx->pos += take;
}

// Called by the TWDT ISR before it panics — keep this minimal and ISR-safe.
// Already declared extern "C" by esp_task_wdt.h; no need to repeat it here.
void esp_task_wdt_isr_user_handler(void) {
    s_wdt_capture.magic = 0;  // invalidate until fully populated
    s_wdt_capture.uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    memset(s_wdt_capture.tasks, 0, sizeof(s_wdt_capture.tasks));
    WdtCaptureCtx ctx = { s_wdt_capture.tasks, sizeof(s_wdt_capture.tasks), 0, 0 };
    esp_task_wdt_print_triggered_tasks(&wdt_capture_collect, &ctx, NULL);
    s_wdt_capture.magic = FLOG_WDT_CAPTURE_MAGIC;
}

void fram_log_check_wdt_capture(void) {
    if (s_wdt_capture.magic != FLOG_WDT_CAPTURE_MAGIC) return;
    FramLogRecord r = {};
    r.type = FLOG_EV_TASK_WDT;
    r.d.wdt.uptime_s = (s_wdt_capture.uptime_s > 0xFFFF) ? 0xFFFF : (uint16_t)s_wdt_capture.uptime_s;
    memcpy(r.d.wdt.tasks, s_wdt_capture.tasks, sizeof(r.d.wdt.tasks));
    fram_log_event(&r);
    s_wdt_capture.magic = 0;  // don't log it again next boot
}

// ---------------------------------------------------------------------------
// Panic summary (unsymbolized, on-device — see decode_crash.sh/crash_archive/
// for a real backtrace against the matching build_archive/*.elf)
//
// esp_core_dump_get_summary() parses the coredump ESP-IDF's own coredump-to-
// flash already wrote on any panic (CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH,
// see CLAUDE.md) — no .elf needed on-device, just the crashed task's name,
// PC, and (Xtensa) the exception cause/faulting address straight out of the
// coredump header. The coredump partition is NOT cleared after being read,
// so gating this on THIS boot's own esp_reset_reason() (not "a coredump
// merely exists") is what stops the same stale crash from being re-logged
// on every later clean reboot.
// ---------------------------------------------------------------------------
static bool reset_reason_is_crash(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_PANIC:
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            return true;
        default:
            return false;
    }
}

void fram_log_check_panic_summary(void) {
    if (!reset_reason_is_crash(esp_reset_reason())) return;

    // Zero-initialized: esp_core_dump_get_summary() isn't guaranteed to
    // populate every field for every panic type (confirmed live 2026-09-24 —
    // a TaskWDT panic, not a hardware exception, left ex_info.exc_cause as
    // uninitialized stack garbage, logged as a nonsense value before this
    // fix). Leaving it zeroed on an unpopulated field reads as a real but
    // at least plausible "0 = illegal instruction" rather than garbage.
    esp_core_dump_summary_t summary = {};
    if (esp_core_dump_get_summary(&summary) != ESP_OK) return;

    FramLogRecord r = {};
    r.type = FLOG_EV_PANIC_SUMMARY;
    r.d.panic.pc = summary.exc_pc;
    r.d.panic.exc_cause = summary.ex_info.exc_cause;
    r.d.panic.exc_vaddr = summary.ex_info.exc_vaddr;
    memcpy(r.d.panic.task, summary.exc_task, sizeof(r.d.panic.task));
    fram_log_event(&r);
}

// ---------------------------------------------------------------------------
// Init
// ---------------------------------------------------------------------------
bool fram_log_begin(FRAM32 *dev) {
    s_fram = dev;
    s_present = false;
    if (dev == NULL) return false;

    FlogHeader h = {};
    s_fram->readObject(FLOG_HEADER_ADDR, h);
    if (h.magic != FLOG_MAGIC || h.version != FLOG_VERSION ||
        h.rec_size != FLOG_REC_SIZE || h.rec_count != FLOG_REC_COUNT) {
        SLOG_PRINTLN("[FLOG] No valid header — formatting FRAM log area");
        wipe_log_area();
        wipe_metrics_area();
        write_header();
    }

    metrics_load();
    uint32_t newest = find_newest_seq();
    s_next_seq = newest + 1;
    s_present = true;

    SLOG_PRINTF("[FLOG] Ready: %lu records (newest seq=%lu), engine=%lus starts=%lu\n",
                  (unsigned long)(newest > FLOG_REC_COUNT ? FLOG_REC_COUNT : newest),
                  (unsigned long)newest,
                  (unsigned long)s_metrics.total_run_seconds,
                  (unsigned long)s_metrics.start_count);
    return true;
}

// ---------------------------------------------------------------------------
// Read path
// ---------------------------------------------------------------------------
uint32_t fram_log_newest_seq(void) {
    portENTER_CRITICAL(&s_flog_mux);
    uint32_t v = s_next_seq - 1;
    portEXIT_CRITICAL(&s_flog_mux);
    return v;
}

uint32_t fram_log_oldest_seq(void) {
    uint32_t newest = fram_log_newest_seq();
    if (newest == 0) return 0;
    return (newest > FLOG_REC_COUNT) ? newest - FLOG_REC_COUNT + 1 : 1;
}

// Caller-owned FRAM job args + fn wrappers for i2c_run_on_owner() — these
// run on I2CTask's own context, not the caller's (WebTask for all three
// entry points below). Each entry point reuses one static "done" semaphore
// since calls from a given task are already sequential (a single-request-
// at-a-time WebServer handler, in practice).

// seq/rec are static, not part of a stack-local args struct pointing at the
// caller's own `out` buffer: if i2c_run_on_owner() times out (this job
// queued behind a long one, e.g. a FRAM wipe), fram_log_read()'s caller may
// already have returned — its `out` buffer (typically a local in a WebTask
// HTTP handler) would then be a dangling pointer by the time I2CTask
// actually gets to the job. Reading into a static instead and copying out
// only on confirmed success avoids that regardless of timing.
// `ok` here is what actually reports the read's own success — I2CJob_t
// itself carries no result field (it crosses xI2CJobQueue by value, so
// I2CTask only ever sees its own private copy of the job struct; an
// earlier version relied on that copy's `ok` field and silently always
// reported failure back to the caller — this is what /log's always-empty
// event list was actually pointing at). i2c_run_on_owner() returning true
// only means "the job ran before the timeout", not "the read succeeded".
typedef struct { uint32_t seq; FramLogRecord rec; bool ok; } FramReadJob;
static FramReadJob s_read_job;
static bool job_fram_read(void *arg) {
    FramReadJob *j = (FramReadJob *)arg;
    j->ok = s_fram->readObject(slot_addr(seq_slot(j->seq)), j->rec);
    return j->ok;
}

bool fram_log_read(uint32_t seq, FramLogRecord *out) {
    if (!s_present || seq == 0) return false;
    uint32_t newest = fram_log_newest_seq();
    uint32_t oldest = fram_log_oldest_seq();
    if (seq < oldest || seq > newest) return false;

    static SemaphoreHandle_t s_done = NULL;
    if (s_done == NULL) s_done = xSemaphoreCreateBinary();
    s_read_job.seq = seq;
    if (!i2c_run_on_owner(job_fram_read, &s_read_job, s_done, pdMS_TO_TICKS(500))) return false;
    if (!s_read_job.ok) return false;
    *out = s_read_job.rec;

    return out->seq == seq && out->crc == rec_crc(out);
}

// Static, not a stack-local args struct, for the same reason as
// FramReadJob above — safe regardless of whether fram_log_wipe()'s own
// (already generous) timeout is hit.
static bool s_wipe_metrics_too;
static bool job_fram_wipe(void *arg) {
    (void)arg;
    wipe_log_area();
    portENTER_CRITICAL(&s_flog_mux);
    s_next_seq = 1;
    portEXIT_CRITICAL(&s_flog_mux);
    if (s_wipe_metrics_too) {
        wipe_metrics_area();
        portENTER_CRITICAL(&s_flog_mux);
        memset(&s_metrics, 0, sizeof(s_metrics));
        s_metrics_dirty = false;
        portEXIT_CRITICAL(&s_flog_mux);
        s_metrics_slot = 0;
    }
    return true;
}

void fram_log_wipe(bool metrics_too) {
    if (!s_present) return;
    static SemaphoreHandle_t s_done = NULL;
    if (s_done == NULL) s_done = xSemaphoreCreateBinary();
    s_wipe_metrics_too = metrics_too;
    // This one job runs the entire ~2046-chunk erase loop on I2CTask in a
    // single call — same total blocking cost as today, just executed by
    // I2CTask instead of by WebTask under a mutex. Long timeout to match.
    i2c_run_on_owner(job_fram_wipe, NULL, s_done, pdMS_TO_TICKS(10000));
}

// ---------------------------------------------------------------------------
// Metrics accessors
// ---------------------------------------------------------------------------
FramMetrics fram_metrics_get(void) {
    portENTER_CRITICAL(&s_flog_mux);
    FramMetrics m = s_metrics;
    portEXIT_CRITICAL(&s_flog_mux);
    return m;
}

// Own static result, same reason as FramReadJob.ok above — the flush's
// actual outcome has to be reported through here, not through I2CJob_t.
static bool s_metrics_flush_ok;
static bool job_metrics_flush(void *arg) {
    (void)arg;
    s_metrics_flush_ok = metrics_flush_locked();
    return s_metrics_flush_ok;
}

double fram_maint_hours_since(const FramMetrics *m, uint8_t idx) {
    if (!m->maint_ts[idx]) return (double)m->total_run_seconds / 3600.0;
    if (m->maint_seconds[idx] > m->total_run_seconds) return 0.0;  // see fram_log.h comment
    return (double)(m->total_run_seconds - m->maint_seconds[idx]) / 3600.0;
}

static bool metrics_flush_on_owner(void) {
    static SemaphoreHandle_t s_done = NULL;
    if (s_done == NULL) s_done = xSemaphoreCreateBinary();
    if (!i2c_run_on_owner(job_metrics_flush, NULL, s_done, pdMS_TO_TICKS(500))) return false;
    return s_metrics_flush_ok;
}

bool fram_metrics_set(const FramMetricsPatch_t *patch) {
    if (!s_present || patch == NULL || patch->fields == 0) return false;
    if ((patch->fields & FRAM_METRICS_SET_RUN_HOURS)    && patch->run_hours < 0.0f)    return false;
    if ((patch->fields & FRAM_METRICS_SET_CHARGE_HOURS) && patch->charge_hours < 0.0f) return false;
    if ((patch->fields & FRAM_METRICS_SET_MAX_HEAD_TEMP) && patch->max_head_temp < 0.0f) return false;
    if ((patch->fields & FRAM_METRICS_SET_MAX_RPM)      && patch->max_rpm < 0.0f)      return false;
    if ((patch->fields & FRAM_METRICS_SET_MAX_POWER)    && patch->max_power < 0.0f)    return false;

    portENTER_CRITICAL(&s_flog_mux);
    if (patch->fields & FRAM_METRICS_SET_RUN_HOURS)
        s_metrics.total_run_seconds = (uint32_t)(patch->run_hours * 3600.0f + 0.5f);
    if (patch->fields & FRAM_METRICS_SET_CHARGE_HOURS)
        s_metrics.total_charge_seconds = (uint32_t)(patch->charge_hours * 3600.0f + 0.5f);
    if (patch->fields & FRAM_METRICS_SET_STARTS)
        s_metrics.start_count = patch->starts;
    if (patch->fields & FRAM_METRICS_SET_START_FAILS)
        s_metrics.start_fail_count = patch->start_fails;
    if (patch->fields & FRAM_METRICS_SET_EMERGENCIES)
        s_metrics.emergency_count = patch->emergencies;
    if (patch->fields & FRAM_METRICS_SET_CHARGE_CYCLES)
        s_metrics.charge_cycles = patch->charge_cycles;
    if (patch->fields & FRAM_METRICS_SET_MAX_HEAD_TEMP)
        s_metrics.max_head_temp = patch->max_head_temp;
    if (patch->fields & FRAM_METRICS_SET_MAX_RPM)
        s_metrics.max_rpm = patch->max_rpm;
    if (patch->fields & FRAM_METRICS_SET_MAX_POWER)
        s_metrics.max_power = patch->max_power;
    s_metrics_dirty = true;
    portEXIT_CRITICAL(&s_flog_mux);

    return metrics_flush_on_owner();
}

bool fram_metrics_set_run_hours(float hours) {
    FramMetricsPatch_t p = {};
    p.fields = FRAM_METRICS_SET_RUN_HOURS;
    p.run_hours = hours;
    return fram_metrics_set(&p);
}

void fram_log_set_running(bool running)   { s_running_flag  = running; }
void fram_log_set_charging(bool charging) { s_charging_flag = charging; }

void fram_metrics_note_sensors(float head_temp, float rpm) {
    portENTER_CRITICAL(&s_flog_mux);
    if (head_temp > s_metrics.max_head_temp) { s_metrics.max_head_temp = head_temp; s_metrics_dirty = true; }
    if (rpm > s_metrics.max_rpm)             { s_metrics.max_rpm = rpm;             s_metrics_dirty = true; }
    portEXIT_CRITICAL(&s_flog_mux);
}

void fram_metrics_note_power(float watts) {
    portENTER_CRITICAL(&s_flog_mux);
    if (watts > s_metrics.max_power) { s_metrics.max_power = watts; s_metrics_dirty = true; }
    portEXIT_CRITICAL(&s_flog_mux);
}

bool fram_log_pop_for_mqtt(FramLogRecord *out) {
    if (xLogMqttQueue == NULL) return false;
    return xQueueReceive(xLogMqttQueue, out, 0) == pdTRUE;
}

// ---------------------------------------------------------------------------
// JSON / names
// ---------------------------------------------------------------------------
const char *fram_log_type_name(uint8_t type) {
    switch (type) {
        case FLOG_EV_BOOT:           return "boot";
        case FLOG_EV_TIME_SYNC:      return "time_sync";
        case FLOG_EV_START_REQUEST:  return "start_request";
        case FLOG_EV_START_OK:       return "start_ok";
        case FLOG_EV_START_FAIL:     return "start_fail";
        case FLOG_EV_STOP_REQUEST:   return "stop_request";
        case FLOG_EV_STOPPED:        return "stopped";
        case FLOG_EV_EMERGENCY_STOP: return "emergency_stop";
        case FLOG_EV_CHARGE_START:   return "charge_start";
        case FLOG_EV_CHARGE_STOP:    return "charge_stop";
        case FLOG_EV_ERROR:          return "error";
        case FLOG_EV_TASK_WDT:       return "task_wdt";
        case FLOG_EV_FW_UPDATE:      return "fw_update";
        case FLOG_EV_I2C_STUCK:      return "i2c_stuck";
        case FLOG_EV_SENSOR_FAIL:    return "sensor_fail";
        case FLOG_EV_PANIC_SUMMARY:  return "panic_summary";
        case FLOG_EV_MAINT:          return "maintenance";
        default:                     return "unknown";
    }
}

const char *fram_log_group_name(uint8_t type) {
    switch (type) {
        case FLOG_EV_START_REQUEST:
        case FLOG_EV_START_OK:
        case FLOG_EV_STOP_REQUEST:
        case FLOG_EV_STOPPED:
            return "engine";
        case FLOG_EV_START_FAIL:
        case FLOG_EV_EMERGENCY_STOP:
        case FLOG_EV_ERROR:
        case FLOG_EV_TASK_WDT:
        case FLOG_EV_I2C_STUCK:
        case FLOG_EV_SENSOR_FAIL:
        case FLOG_EV_PANIC_SUMMARY:
            return "alarm";
        case FLOG_EV_CHARGE_START:
        case FLOG_EV_CHARGE_STOP:
            return "charge";
        case FLOG_EV_MAINT:
            return "maint";
        case FLOG_EV_BOOT:
        case FLOG_EV_TIME_SYNC:
        case FLOG_EV_FW_UPDATE:
        default:
            return "sys";
    }
}

const char *fram_log_maint_name(uint8_t subtype) {
    switch (subtype) {
        case FLOG_MAINT_OIL:         return "oil";
        case FLOG_MAINT_SPARK:       return "spark_plug";
        case FLOG_MAINT_AIR_FILTER:  return "air_filter";
        case FLOG_MAINT_FUEL_FILTER: return "fuel_filter";
        case FLOG_MAINT_VALVES:      return "valves";
        case FLOG_MAINT_OTHER:       return "other";
        default:                     return "unknown";
    }
}

int fram_log_maint_parse(const char *s) {
    if (s == NULL || *s == '\0') return -1;
    for (uint8_t i = 0; i < FLOG_MAINT_TYPES; i++)
        if (strcmp(s, fram_log_maint_name(i)) == 0) return i;
    if (isdigit((unsigned char)*s)) {
        int v = atoi(s);
        if (v >= 0 && v < FLOG_MAINT_TYPES) return v;
    }
    return -1;
}

static const char *source_name(uint8_t src) {
    switch (src) {
        case FLOG_SRC_HTTP:      return "http";
        case FLOG_SRC_WEBSOCKET: return "websocket";
        case FLOG_SRC_MQTT:      return "mqtt";
        case FLOG_SRC_PANEL:     return "panel";
        case FLOG_SRC_AUTO:      return "auto";
        case FLOG_SRC_ESPNOW:    return "espnow";
        default:                 return "unknown";
    }
}

static const char *reason_name(uint8_t r) {
    switch (r) {
        case FLOG_REASON_OIL:      return "oil";
        case FLOG_REASON_OVERHEAT: return "overheat";
        case FLOG_REASON_OVERLOAD: return "overload";
        case FLOG_REASON_STALLED:    return "stalled";
        case FLOG_REASON_TEMP_LIMIT: return "temp_limit";
        default:                     return "unknown";
    }
}

static const char *sensor_fail_name(uint8_t s) {
    switch (s) {
        case FLOG_SENSOR_MCP9600: return "mcp9600";
        case FLOG_SENSOR_INA233:  return "ina233";
        default:                  return "unknown";
    }
}

static const char *chg_reason_name(uint8_t r) {
    switch (r) {
        case FLOG_CHG_THRESHOLD:    return "threshold";
        case FLOG_CHG_PSU_LOST:     return "psu_lost";
        case FLOG_CHG_MANUAL:       return "manual";
        case FLOG_CHG_ENGINE_START: return "engine_start";
        default:                    return "unknown";
    }
}

// JSON-escape a NUL-terminated string into dst (bounded by dst_sz, always
// left NUL-terminated). Needed for every fixed-size char[] field emitted
// via a raw "%s" below — `tasks` in particular is filled by
// esp_task_wdt_print_triggered_tasks(), which embeds real \r\n bytes
// between task names (not the two-character escape sequence), and a single
// unescaped control byte in a JSON string breaks every consumer's
// JSON.parse() outright. Confirmed live 2026-08-25: the web journal page's
// "Bad control character in string literal" load failure, traced to exactly
// this field. `note` (user-supplied, already filtered at write time in
// fram_log_maintenance()) and `version` (firmware build-date string) are
// escaped here too on the same "don't trust a raw fixed-size buffer" basis,
// not because either is known to have hit this — cheap insurance.
static void json_escape_into(char *dst, size_t dst_sz, const char *src) {
    size_t o = 0;
    for (const unsigned char *p = (const unsigned char *)src; *p && o + 1 < dst_sz; p++) {
        unsigned char c = *p;
        const char *esc = nullptr;
        switch (c) {
            case '"':  esc = "\\\""; break;
            case '\\': esc = "\\\\"; break;
            case '\n': esc = "\\n";  break;
            case '\r': esc = "\\r";  break;
            case '\t': esc = "\\t";  break;
            default: break;
        }
        if (esc) {
            size_t elen = strlen(esc);
            if (o + elen >= dst_sz) break;
            memcpy(dst + o, esc, elen);
            o += elen;
        } else if (c < 0x20) {
            if (o + 6 >= dst_sz) break;   // "\u00XX"
            snprintf(dst + o, dst_sz - o, "\\u%04x", c);
            o += 6;
        } else {
            dst[o++] = (char)c;
        }
    }
    dst[o] = '\0';
}

int fram_log_record_json(const FramLogRecord *r, char *buf, size_t n) {
    int len = snprintf(buf, n,
        "{\"seq\":%lu,\"ts\":%lu,\"time_valid\":%d,\"type\":\"%s\"",
        (unsigned long)r->seq, (unsigned long)r->ts,
        (r->flags & FLOG_FLAG_TIME_VALID) ? 1 : 0,
        fram_log_type_name(r->type));
    if (len < 0 || (size_t)len >= n) return len;

#define APPEND(...) do { \
        int _a = snprintf(buf + len, n - len, __VA_ARGS__); \
        if (_a > 0) len += _a; \
        if ((size_t)len >= n) { len = (int)n - 1; goto done; } \
    } while (0)

    switch (r->type) {
        case FLOG_EV_BOOT:
            APPEND(",\"reset_reason\":%u", r->d.boot.reset_reason);
            break;
        case FLOG_EV_TIME_SYNC:
            APPEND(",\"old_ts\":%lu,\"new_ts\":%lu",
                   (unsigned long)r->d.time_sync.old_ts,
                   (unsigned long)r->d.time_sync.new_ts);
            break;
        case FLOG_EV_START_REQUEST:
        case FLOG_EV_STOP_REQUEST:
            APPEND(",\"source\":\"%s\",\"retry\":%u",
                   source_name(r->d.request.source), r->d.request.retry);
            break;
        case FLOG_EV_START_OK:
            APPEND(",\"attempts\":%u,\"detected\":%u,\"head_temp\":%.1f,"
                   "\"bat_v\":%.2f,\"crank_ms\":%lu",
                   r->d.start_ok.attempts, r->d.start_ok.detected,
                   (double)r->d.start_ok.head_temp, (double)r->d.start_ok.bat_v,
                   (unsigned long)r->d.start_ok.crank_ms);
            break;
        case FLOG_EV_START_FAIL:
            APPEND(",\"attempts\":%u", r->d.start_fail.attempts);
            break;
        case FLOG_EV_STOPPED:
            APPEND(",\"run_seconds\":%lu,\"head_temp\":%.1f,\"fuel_l\":%.1f",
                   (unsigned long)r->d.stopped.run_seconds,
                   (double)r->d.stopped.head_temp, (double)r->d.stopped.fuel_l);
            break;
        case FLOG_EV_EMERGENCY_STOP:
            APPEND(",\"reason\":\"%s\",\"inputs\":%u,\"head_temp\":%.1f,"
                   "\"run_seconds\":%lu",
                   reason_name(r->d.emergency.reason), r->d.emergency.inputs,
                   (double)r->d.emergency.head_temp,
                   (unsigned long)r->d.emergency.run_seconds);
            break;
        case FLOG_EV_CHARGE_START:
            APPEND(",\"bat_v\":%.2f,\"auto\":%u",
                   (double)r->d.charge_start.bat_v, r->d.charge_start.auto_started);
            break;
        case FLOG_EV_CHARGE_STOP:
            APPEND(",\"bat_v\":%.2f,\"charge_seconds\":%lu,\"reason\":\"%s\"",
                   (double)r->d.charge_stop.bat_v,
                   (unsigned long)r->d.charge_stop.charge_seconds,
                   chg_reason_name(r->d.charge_stop.reason));
            break;
        case FLOG_EV_ERROR:
            APPEND(",\"code\":%u,\"value\":%.2f",
                   r->d.error.code, (double)r->d.error.value);
            break;
        case FLOG_EV_TASK_WDT: {
            char tasks[sizeof(r->d.wdt.tasks) + 1];
            memcpy(tasks, r->d.wdt.tasks, sizeof(r->d.wdt.tasks));
            tasks[sizeof(r->d.wdt.tasks)] = '\0';
            char tasks_esc[sizeof(tasks) * 6 + 1];  // worst case: every byte -> \u00XX
            json_escape_into(tasks_esc, sizeof(tasks_esc), tasks);
            APPEND(",\"uptime_s\":%lu,\"tasks\":\"%s\"",
                   (unsigned long)r->d.wdt.uptime_s, tasks_esc);
            break;
        }
        case FLOG_EV_FW_UPDATE: {
            char version[sizeof(r->d.fw_update.version) + 1];
            memcpy(version, r->d.fw_update.version, sizeof(r->d.fw_update.version));
            version[sizeof(r->d.fw_update.version)] = '\0';
            char version_esc[sizeof(version) * 6 + 1];
            json_escape_into(version_esc, sizeof(version_esc), version);
            APPEND(",\"size\":%lu,\"version\":\"%s\"",
                   (unsigned long)r->d.fw_update.size, version_esc);
            break;
        }
        case FLOG_EV_I2C_STUCK:
            APPEND(",\"stuck_ms\":%lu", (unsigned long)r->d.i2c_stuck.stuck_ms);
            break;
        case FLOG_EV_SENSOR_FAIL:
            APPEND(",\"sensor\":\"%s\"", sensor_fail_name(r->d.sensor_fail.sensor));
            break;
        case FLOG_EV_PANIC_SUMMARY: {
            char task[sizeof(r->d.panic.task) + 1];
            memcpy(task, r->d.panic.task, sizeof(r->d.panic.task));
            task[sizeof(r->d.panic.task)] = '\0';
            char task_esc[sizeof(task) * 6 + 1];
            json_escape_into(task_esc, sizeof(task_esc), task);
            APPEND(",\"pc\":\"0x%08lx\",\"exc_cause\":%lu,\"exc_vaddr\":\"0x%08lx\",\"task\":\"%s\"",
                   (unsigned long)r->d.panic.pc, (unsigned long)r->d.panic.exc_cause,
                   (unsigned long)r->d.panic.exc_vaddr, task_esc);
            break;
        }
        case FLOG_EV_MAINT: {
            char note[sizeof(r->d.maint.note) + 1];
            memcpy(note, r->d.maint.note, sizeof(r->d.maint.note));
            note[sizeof(r->d.maint.note)] = '\0';
            char note_esc[sizeof(note) * 6 + 1];
            json_escape_into(note_esc, sizeof(note_esc), note);
            APPEND(",\"maint\":\"%s\",\"engine_hours\":%.1f,\"note\":\"%s\"",
                   fram_log_maint_name(r->d.maint.subtype),
                   (double)r->d.maint.engine_seconds / 3600.0, note_esc);
            break;
        }
        default: break;
    }
    APPEND("}");
done:
#undef APPEND
    return len;
}

int fram_metrics_json(char *buf, size_t n, const float *maint_interval_hours) {
    FramMetrics m = fram_metrics_get();
    int len = snprintf(buf, n,
        "{\"run_hours\":%.2f,\"run_seconds\":%lu,"
        "\"charge_hours\":%.2f,\"charge_seconds\":%lu,"
        "\"starts\":%lu,\"start_fails\":%lu,\"emergencies\":%lu,"
        "\"charge_cycles\":%lu,"
        "\"max_head_temp\":%.1f,\"max_rpm\":%.0f,\"max_power\":%.0f,"
        "\"maintenance\":{",
        (double)m.total_run_seconds / 3600.0, (unsigned long)m.total_run_seconds,
        (double)m.total_charge_seconds / 3600.0, (unsigned long)m.total_charge_seconds,
        (unsigned long)m.start_count, (unsigned long)m.start_fail_count,
        (unsigned long)m.emergency_count, (unsigned long)m.charge_cycles,
        (double)m.max_head_temp, (double)m.max_rpm, (double)m.max_power);
    if (len < 0 || (size_t)len >= n) return len;

    for (uint8_t i = 0; i < FLOG_MAINT_TYPES; i++) {
        double since_h = fram_maint_hours_since(&m, i);
        int a;
        if (maint_interval_hours && maint_interval_hours[i] > 0.0f) {
            a = snprintf(buf + len, n - len,
                "%s\"%s\":{\"ts\":%lu,\"at_hours\":%.1f,\"hours_since\":%.1f,\"due\":%s}",
                i ? "," : "",
                fram_log_maint_name(i),
                (unsigned long)m.maint_ts[i],
                (double)m.maint_seconds[i] / 3600.0, since_h,
                (since_h >= (double)maint_interval_hours[i]) ? "true" : "false");
        } else {
            a = snprintf(buf + len, n - len,
                "%s\"%s\":{\"ts\":%lu,\"at_hours\":%.1f,\"hours_since\":%.1f}",
                i ? "," : "",
                fram_log_maint_name(i),
                (unsigned long)m.maint_ts[i],
                (double)m.maint_seconds[i] / 3600.0, since_h);
        }
        if (a > 0) len += a;
        if ((size_t)len >= n) return (int)n - 1;
    }
    int a = snprintf(buf + len, n - len, "}}");
    if (a > 0) len += a;
    return len;
}
