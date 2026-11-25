/*
 * report.cpp — ReportTask, broadcasts full state over WebSocket (port 81)
 * ~1x/sec, plus a few housekeeping checks that share its tick.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code
 * organization" section and [[main-cpp-modularization]] project memory for
 * the pattern this follows). Moved verbatim.
 */

#include "report.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "shiftreg.h"
#include "smoothed_safe.h"
#include "debug.h"
#include <stdint.h>
#include <stddef.h>
#include "uptime.h"
#include "wifi_net.h"
#include "http_server.h"
#include <ArduinoJson.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_heap_caps.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "datetime.h"

// ---- externs into esp32-gen-ctl-ai-rtos.cpp's own globals ------------------
extern SmoothedSafe<double> adc_1_0;
extern SmoothedSafe<double> adc_1_1;
extern SmoothedSafe<double> adc_1_2;
extern SmoothedSafe<double> adc_1_3;
extern SmoothedSafe<double> adc_2_0;
extern SmoothedSafe<double> adc_2_1;
extern SmoothedSafe<double> adc_2_2;
double fuel_level_from_adc(double adc_value);
const char *resetReasonName(esp_reset_reason_t r);
bool is_not_working(void);
uint32_t rtc_now_unixtime(void);
extern DateTime start;  // boot-time timestamp, set once in setup()

// Deferring confirmation (above) only covers the FIRST boot attempt of a
// pending image — it does NOT, by itself, protect against a crash before
// that soak completes. Found live 2026-09-25 running an actual crash-loop-
// then-rollback test end to end: a naive "reject if still PENDING_VERIFY
// on a later boot" check never fires, because it's not what ota_state
// actually reads on a repeat attempt. What really happens, read straight
// from bootloader_utility.c: on EVERY boot, the bootloader marks any
// still-PENDING_VERIFY slot ABORTED *before* even picking which partition
// to load — so by the time this app's own code runs on attempt #2+, the
// state is already ESP_OTA_IMG_ABORTED, not PENDING_VERIFY. That marking
// is purely informational, though — partition SELECTION is driven only by
// ota_seq, so marking ABORTED alone does not switch which slot boots next.
// Only an explicit esp_ota_mark_app_invalid_rollback_and_reboot() call
// (esp_ota_ops.c) actually rewrites ota_seq/state so the other slot gets
// picked. Confirmed live: without this, a bad image that crashes before
// confirming just crash-loops on itself forever — 46+ identical cycles on
// the same slot, zero reversion, with a fully valid previous slot sitting
// right there the whole time.
//
// So: check for ABORTED (the state the bootloader itself already set)
// as the very first thing in setup(), and explicitly reject-and-reboot
// when found — see the call site in the main .cpp.
void ota_reject_if_aborted(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) != ESP_OK) {
        return;
    }
    if (ota_state == ESP_OTA_IMG_ABORTED || ota_state == ESP_OTA_IMG_INVALID) {
        esp_ota_mark_app_invalid_rollback_and_reboot();
        // Does not return on success — the reboot happens inside this call.
    }
}

// Delayed OTA-image self-confirmation — see ota_arm_confirm_soak() /
// ReportTask below (also documented in CLAUDE.md's idf.py-build section).
#define OTA_CONFIRM_HEALTHY_AFTER_MS (10UL * 60UL * 1000UL)  // 10 minutes

// Armed only when the running image is PENDING_VERIFY. Timer callback sets
// s_ota_confirm_due; ReportTask alone calls esp_ota_mark_* so the otadata
// flash write never runs on the FreeRTOS timer daemon task.
static volatile bool s_ota_soak_armed = false;
static volatile bool s_ota_confirm_due = false;
static TimerHandle_t s_ota_confirm_timer = nullptr;

static void ota_confirm_timer_cb(TimerHandle_t) {
    s_ota_confirm_due = true;
}

void ota_arm_confirm_soak(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(running, &ota_state) != ESP_OK) {
        return;
    }
    if (ota_state != ESP_OTA_IMG_PENDING_VERIFY) {
        return;
    }

    s_ota_confirm_timer = xTimerCreate(
        "ota_cfm",
        pdMS_TO_TICKS(OTA_CONFIRM_HEALTHY_AFTER_MS),
        pdFALSE,  // one-shot
        nullptr,
        ota_confirm_timer_cb);
    if (s_ota_confirm_timer == nullptr) {
        SLOG_PRINTLN("[OTA] confirm soak timer create failed");
        return;
    }
    s_ota_soak_armed = true;
    if (xTimerStart(s_ota_confirm_timer, 0) != pdPASS) {
        s_ota_soak_armed = false;
        xTimerDelete(s_ota_confirm_timer, 0);
        s_ota_confirm_timer = nullptr;
        SLOG_PRINTLN("[OTA] confirm soak timer start failed");
        return;
    }
    SDEBUG("[OTA] confirm soak armed for ",
           (unsigned long)(OTA_CONFIRM_HEALTHY_AFTER_MS / 60000UL),
           "min (PENDING_VERIFY)");
}

// Formats a float/double into a small reused static buffer instead of
// allocating a temporary String on every ReportTask tick — that per-tick
// String churn (one heap alloc/free per numeric field, ~15 fields, every
// ~1s) was itself part of the fragmentation problem chased throughout this
// session. serialized(char*) makes ArduinoJson copy the bytes into the
// document's own pool immediately on assignment (see setRawString ->
// saveString), so reusing this single buffer sequentially — one field
// formatted and assigned, then the same buffer overwritten for the next —
// is safe as long as it's only ever called from the single-threaded
// ReportTask.
static const char *ws_fmt_num(double value, uint8_t decimals) {
    static char buf[24];
    snprintf(buf, sizeof(buf), "%.*f", decimals, value);
    return buf;
}

// ----------------------------------------------------------------------------
// ReportTask: Send WebSocket updates
// ----------------------------------------------------------------------------
void ReportTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset
    TickType_t xLastWakeTime = xTaskGetTickCount();

    SLOG_PRINTLN("ReportTask started");

    uint8_t heap_tick = 0;

    // Reused across every tick instead of building a fresh document (and a
    // fresh, growing String for its serialized output) each time — that
    // alloc/free-every-~1s churn was itself a fragmentation source, the same
    // class of bug as the bad_alloc crash below. Never call
    // ws_report_doc.clear() here: it tears down and reallocates the whole
    // pool from the heap, undoing the point of reusing it — re-creating the
    // same nested objects/keys on top of the existing document instead lets
    // ArduinoJson v7 recycle slots via its internal free list.
    static JsonDocument ws_report_doc;
    static char ws_report_buf[1536];
    // Filled fresh each tick below, but declared static like build_time_buf
    // just below it — ArduinoJson stores the pointer, not a copy, so the
    // backing memory must outlive this iteration's later serializeJson().
    static char maint_due_buf[128];

    for (;;) {
        // I2CTask health check — replaces the old xI2CMutex-contention
        // polling that lived in loop() (meaningless once I2CTask became the
        // sole I2C owner — nothing left to contend for). g_state.sensors.
        // timestamp is refreshed every i2c_read_sensors() cycle (~500ms);
        // if it hasn't moved in 5+ seconds (well above that cadence),
        // I2CTask itself has stalled — a genuinely wedged device, or a
        // long-running job (e.g. FRAM wipe) — logged once per episode,
        // same edge-triggered idea as sensor_log_fail_once().
        {
            static uint32_t last_sensor_ts_seen = 0;
            static uint32_t stuck_since_ms = 0;
            static bool i2c_stuck_logged = false;
            uint32_t sensor_ts;
            portENTER_CRITICAL(&g_state_spinlock);
            sensor_ts = g_state.sensors.timestamp;
            portEXIT_CRITICAL(&g_state_spinlock);

            if (sensor_ts != last_sensor_ts_seen) {
                last_sensor_ts_seen = sensor_ts;
                stuck_since_ms = 0;
                i2c_stuck_logged = false;
            } else {
                if (stuck_since_ms == 0) stuck_since_ms = uptime_ms();
                uint32_t stuck_ms = uptime_ms() - stuck_since_ms;
                if (stuck_ms >= 5000 && !i2c_stuck_logged) {
                    i2c_stuck_logged = true;
                    FramLogRecord r = {};
                    r.type = FLOG_EV_I2C_STUCK;
                    r.d.i2c_stuck.stuck_ms = stuck_ms;
                    fram_log_event(&r);
                }
            }
        }

        // OTA soak confirm — only while a PENDING_VERIFY image armed the
        // one-shot timer in ota_arm_confirm_soak(). Timer callback only sets
        // s_ota_confirm_due; the otadata write stays here in ReportTask.
        // Already-VALID boots never arm, so this block is a no-op forever.
        if (s_ota_soak_armed && s_ota_confirm_due) {
            s_ota_soak_armed = false;
            s_ota_confirm_due = false;
            esp_ota_mark_app_valid_cancel_rollback();
            if (s_ota_confirm_timer != nullptr) {
                xTimerDelete(s_ota_confirm_timer, 0);
                s_ota_confirm_timer = nullptr;
            }
            SDEBUG("[OTA] image confirmed healthy after ",
                   (unsigned long)(OTA_CONFIRM_HEALTHY_AFTER_MS / 60000UL),
                   "min soak — rollback cancelled");
        }

        // Heap health log every 60 s, for watching the long-term trend
        // before deciding whether the recurring bad_alloc crashes need an
        // architecture change (away from Async*) or are a fixable leak.
        // maxblk (largest free internal-RAM block, MALLOC_CAP_INTERNAL) has repeatedly read a suspiciously
        // constant value (e.g. 36852) across wildly different real heap
        // states and even at the exact moment allocations were failing —
        // it can't be trusted. largest8bit (heap_caps_get_largest_free_block)
        // tracked genuine, varying fragmentation correctly (700-9000 bytes
        // under ordinary runtime conditions, no synthetic stress needed),
        // so that's the number to actually watch trend on. Logging both
        // side by side for now to keep confirming the mismatch.
        if (++heap_tick >= 60) {
            heap_tick = 0;
            unsigned ws_clients_now = (unsigned)http_ws_client_count();
            SLOG_PRINTF("[HEAP] free=%u min=%u maxblk=%u largest8bit=%u ws=%u\n",
                          heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
                          heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                          (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                          ws_clients_now);
            if (!heap_caps_check_integrity_all(true)) {
                SLOG_PRINTLN("[HEAP] CORRUPTION DETECTED (periodic check)");
            }
        }

        http_ws_tick();

        // Only send if there are connected clients
        unsigned ws_clients_for_send = (unsigned)http_ws_client_count();
        if (ws_clients_for_send > 0) {
            // Get current state atomically
            SensorData_t sensors;
            PowerData_t power;
            uint8_t inputs;
            uint16_t outputs;
            state cur_state;

            portENTER_CRITICAL(&g_state_spinlock);
            sensors = g_state.sensors;
            power = g_state.power;
            inputs = g_state.shift_reg_inputs;
            outputs = g_state.shift_reg_outputs;
            cur_state = g_state.current_state;
            portEXIT_CRITICAL(&g_state_spinlock);

            // Build JSON using ArduinoJson
            JsonDocument &doc = ws_report_doc;

            // Switch section. 1/0 instead of true/false - same truthiness in
            // JS (main.js does o[p][k] ? ... : ...), 4-5 bytes shorter per field.
            JsonObject sw = doc["switch"].to<JsonObject>();
            for (int i = 0; i < ShiftRegListLength; i++) {
                sw[ShiftRegs[i].name] = (int)((outputs & (1 << ShiftRegs[i].reg)) != 0);
            }
            for (int i = 0; i < ShiftRegInListLength; i++) {
                sw[ShiftRegsIn[i].name] = (int)((inputs & (1 << ShiftRegsIn[i].reg)) != 0);
            }
            bool _running = is_not_working();
            sw["run"] = (int)!_running;
            sw["stop"] = (int)_running;

            // Value section. STATE is state_name() (idle/starting/started/
            // running/stopping/…), the same string MQTT already publishes.
            // switch.run/stop stay as the idle-vs-everything-else pair the
            // web UI reads; the display uses STATE for the finer label.
            JsonObject val = doc["value"].to<JsonObject>();
            val["STATE"] = state_name(cur_state);
            val["ADC0_0"] = serialized(ws_fmt_num(adc_1_0.get(), 1));
            val["ADC0_1"] = serialized(ws_fmt_num(adc_1_1.get(), 1));
            val["ADC0_2"] = serialized(ws_fmt_num(adc_1_2.get(), 1));
            val["ADC0_3"] = serialized(ws_fmt_num(adc_1_3.get(), 1));
            val["ADC1_0"] = serialized(ws_fmt_num(fuel_level_from_adc(adc_2_0.get()), 1));
            val["ADC1_1"] = serialized(ws_fmt_num(adc_2_1.get(), 1));
            val["ADC1_2"] = serialized(ws_fmt_num(adc_2_2.get() / 1000, 2));
            val["HEAD_TEMP"] = serialized(ws_fmt_num(sensors.head_temp, 1));
            val["RPM"] = sensors.rpm;
            val["RSSI"] = wifi_net_rssi();
            val["VRMS"] = serialized(ws_fmt_num(power.rms_voltage, 2));
            val["IRMS"] = serialized(ws_fmt_num(power.rms_current, 4));
            val["POWER_ACTIVE"] = serialized(ws_fmt_num(power.active_power, 3));
            val["POWER_REACTIVE"] = serialized(ws_fmt_num(power.reactive_power, 3));
            val["POWER_APPARENT"] = serialized(ws_fmt_num(power.apparent_power, 3));
            val["FREQUENCY"] = serialized(ws_fmt_num(power.frequency, 2));
            val["POWER_FACTOR"] = serialized(ws_fmt_num(power.power_factor, 4));
            // Lifetime energy accumulators, mWh -> kWh (refreshed every
            // MCP39F511A_ENERGY_READ_PERIOD_MS in PowerMeterTask, not every tick).
            val["ENERGY_IMPORT_ACTIVE"] = serialized(ws_fmt_num(
                power.energy_valid ? (double)power.import_active_energy_mwh / 1000000.0 : 0.0, 3));
            val["ENERGY_EXPORT_ACTIVE"] = serialized(ws_fmt_num(
                power.energy_valid ? (double)power.export_active_energy_mwh / 1000000.0 : 0.0, 3));
            val["ENERGY_IMPORT_REACTIVE"] = serialized(ws_fmt_num(
                power.energy_valid ? (double)power.import_reactive_energy_mwh / 1000000.0 : 0.0, 3));
            val["ENERGY_EXPORT_REACTIVE"] = serialized(ws_fmt_num(
                power.energy_valid ? (double)power.export_reactive_energy_mwh / 1000000.0 : 0.0, 3));
            val["RESET_REASON"] = resetReasonName(esp_reset_reason());
            // Must read the running image's own esp_app_desc (date/time),
            // never this translation unit's own __DATE__/__TIME__: those are
            // fixed at whenever report.cpp itself was last actually
            // recompiled, which ninja's incremental build skips on any
            // build that doesn't touch this file — stale for the exact same
            // reason idf/main/esp32-gen-ctl-ai-rtos.cpp needs
            // bump_fw_version.sh to force a fresh stamp on every build (see
            // CLAUDE.md). esp_app_desc.version[]/date[]/time[] (set from
            // THAT file's own __DATE__/__TIME__, in k_fw_app_desc) is the
            // one place already guaranteed fresh every build — same value
            // archive filenames and /log's fw_update event already show.
            // Found live 2026-10-03 (dashboard Build: card frozen at a date
            // weeks old despite several real rebuilds/reflashes since).
            static char build_time_buf[32] = {0};
            if (build_time_buf[0] == 0) {
                const esp_app_desc_t *desc = esp_app_get_description();
                snprintf(build_time_buf, sizeof(build_time_buf), "%s %s", desc->date, desc->time);
            }
            val["BUILD_TIME"] = build_time_buf;
            FramMetrics fm_snapshot = fram_metrics_get();
            val["ENGINE_HOURS"] = serialized(ws_fmt_num(
                (double)fm_snapshot.total_run_seconds / 3600.0, 1));
            // Overdue-maintenance banner (data-id="MAINT_DUE" in index.html).
            // Labels come from MAINT_TYPE_LABELS (shared with the OLED menu).
            {
                char list_buf[100];
                int list_len = 0;
                list_buf[0] = '\0';
                // MAINT_DUE_MASK: same bits, machine-readable — bit i set means
                // FLOG_MAINT_* index i is overdue. Consumed by display/'s touch
                // UI (Controls-tab Record button) instead of re-parsing the
                // human-readable label list above, which is free text and not
                // safe to match back to a subtype index.
                uint8_t due_mask = 0;
                for (uint8_t i = 0; i < FLOG_MAINT_TYPES; i++) {
                    float thr = g_limits_config.maint_interval_hours[i];
                    if (thr <= 0.0f) continue;
                    double since_h = fram_maint_hours_since(&fm_snapshot, i);
                    if (since_h < (double)thr) continue;
                    due_mask |= (1u << i);
                    int a = snprintf(list_buf + list_len, sizeof(list_buf) - list_len,
                                      "%s%s", list_len ? ", " : "", MAINT_TYPE_LABELS[i]);
                    if (a > 0) list_len += a;
                    if ((size_t)list_len >= sizeof(list_buf)) break;
                }
                // Empty when nothing's overdue — index.html hides the banner
                // via CSS ":empty", no separate visibility flag needed.
                if (list_len > 0) {
                    snprintf(maint_due_buf, sizeof(maint_due_buf),
                             "Maintenance overdue: %s", list_buf);
                } else {
                    maint_due_buf[0] = '\0';
                }
                val["MAINT_DUE"] = maint_due_buf;
                val["MAINT_DUE_MASK"] = due_mask;
            }
            val["FREE_HEAP"] = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
            val["MIN_HEAP"]  = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);

            // Date section
            JsonObject date = doc["date"].to<JsonObject>();
            uint32_t now_ts = rtc_now_unixtime();
            date["TIME"] = now_ts;
            date["UPTIME"] = now_ts - start.unixtime();

            // Serialize into a fixed buffer instead of a growing String —
            // String's grow-by-doubling reallocations are themselves a
            // classic ESP32 fragmentation source (each regrow can leave a
            // stale, oddly-sized hole behind). serializeJson() bounds its
            // writes to the buffer and returns the byte count actually
            // written; treat a return >= sizeof(buf) as truncation, since
            // that's the only case where the library skips the trailing
            // null terminator.
            size_t written = serializeJson(doc, ws_report_buf, sizeof(ws_report_buf));
            if (written >= sizeof(ws_report_buf)) {
                SLOG_PRINTLN("[HEAP] WS report JSON truncated - buffer too small, broadcast skipped");
            } else {
                // Copies the report and sends it from the httpd task, so a
                // stalled client can't block ReportTask (TWDT history).
                if (!http_ws_broadcast(ws_report_buf, written)) {
                    SLOG_PRINTLN("[WS] previous broadcast still in progress - report skipped");
                }
            }
        }

        // Feed watchdog
        esp_task_wdt_reset();

        // Wait for next period
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PERIOD_REPORT_MS));
    }
}
