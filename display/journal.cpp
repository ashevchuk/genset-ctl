/*
 * journal.cpp — fetch generator event journal + metrics over HTTP.
 * Runs from loop() (core 1); hands parsed rows to ui.cpp under display lock.
 */
#include "journal.h"
#include "config.h"
#include "ui.h"
#include "esp_bsp.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

static volatile bool s_refresh_req = false;
static volatile int8_t s_maint_pending = -1;  // -1 = nothing queued

void journal_request_refresh(void) { s_refresh_req = true; }

void journal_request_maintenance(uint8_t subtype) {
    if (subtype >= MAINT_TYPES) return;
    s_maint_pending = (int8_t)subtype;
    s_refresh_req = true;  // ride along on the existing poll gate
}

const char *const MAINT_TYPE_NAMES[MAINT_TYPES] = {
    "oil", "spark_plug", "air_filter", "fuel_filter", "valves", "other"
};
const char *const MAINT_TYPE_LABELS[MAINT_TYPES] = {
    "Oil change", "Spark plug", "Air filter", "Fuel filter", "Valve clearance", "Other"
};

// ---------------------------------------------------------------------------
// Formatting helpers
// ---------------------------------------------------------------------------
static const char *RESET_REASONS[] = {
    "Unknown", "PowerOn", "ExtPin", "Software", "Panic",
    "IntWDT", "TaskWDT", "OtherWDT", "DeepSleep", "Brownout", "SDIO"
};

static void fmt_age(uint32_t now_ts, uint32_t ts, bool time_valid,
                    char *out, size_t n) {
    if (!time_valid || now_ts == 0 || ts == 0 || ts > now_ts) {
        strlcpy(out, "?", n);
        return;
    }
    uint32_t d = now_ts - ts;
    if      (d < 60)          snprintf(out, n, "%lus", (unsigned long)d);
    else if (d < 3600)        snprintf(out, n, "%lum", (unsigned long)(d / 60));
    else if (d < 86400)       snprintf(out, n, "%luh", (unsigned long)(d / 3600));
    else                      snprintf(out, n, "%lud", (unsigned long)(d / 86400));
}

static void fmt_dur(uint32_t sec, char *out, size_t n) {
    if (sec >= 3600) snprintf(out, n, "%luh%02lum",
                              (unsigned long)(sec / 3600),
                              (unsigned long)((sec % 3600) / 60));
    else if (sec >= 60) snprintf(out, n, "%lum%02lus",
                                 (unsigned long)(sec / 60),
                                 (unsigned long)(sec % 60));
    else snprintf(out, n, "%lus", (unsigned long)sec);
}

// Fill one entry from a /log JSON event object.
static void fill_entry(JsonObjectConst ev, uint32_t now_ts, JournalEntry *e) {
    const char *type = ev["type"] | "unknown";
    fmt_age(now_ts, ev["ts"] | 0UL, (ev["time_valid"] | 0) != 0,
            e->age, sizeof(e->age));

    char dur[16];

    if (strcmp(type, "boot") == 0) {
        strlcpy(e->type, "BOOT", sizeof(e->type));
        e->sev = JSEV_SYS;
        int rr = ev["reset_reason"] | 0;
        snprintf(e->text, sizeof(e->text), "reset: %s",
                 (rr >= 0 && rr <= 10) ? RESET_REASONS[rr] : "?");
    } else if (strcmp(type, "time_sync") == 0) {
        strlcpy(e->type, "TIME", sizeof(e->type));
        e->sev = JSEV_SYS;
        long dlt = (long)((ev["new_ts"] | 0UL) - (ev["old_ts"] | 0UL));
        snprintf(e->text, sizeof(e->text), "clock adjusted %+ld s", dlt);
    } else if (strcmp(type, "start_request") == 0) {
        strlcpy(e->type, "START RQ", sizeof(e->type));
        e->sev = JSEV_START;
        snprintf(e->text, sizeof(e->text), "from %s%s",
                 (const char *)(ev["source"] | "?"),
                 (ev["retry"] | 0) ? " (retry)" : "");
    } else if (strcmp(type, "start_ok") == 0) {
        strlcpy(e->type, "STARTED", sizeof(e->type));
        e->sev = JSEV_START;
        if (ev["detected"] | 0) {
            snprintf(e->text, sizeof(e->text), "detected running, bat %.1fV",
                     (double)(ev["bat_v"] | 0.0f));
        } else {
            snprintf(e->text, sizeof(e->text),
                     "try %d, crank %.1fs, %.0f°C, %.1fV",
                     ev["attempts"] | 0,
                     (double)(ev["crank_ms"] | 0UL) / 1000.0,
                     (double)(ev["head_temp"] | 0.0f),
                     (double)(ev["bat_v"] | 0.0f));
        }
    } else if (strcmp(type, "start_fail") == 0) {
        strlcpy(e->type, "ST FAIL", sizeof(e->type));
        e->sev = JSEV_ALARM;
        snprintf(e->text, sizeof(e->text), "gave up after %d attempt(s)",
                 ev["attempts"] | 0);
    } else if (strcmp(type, "stop_request") == 0) {
        strlcpy(e->type, "STOP RQ", sizeof(e->type));
        e->sev = JSEV_STOP;
        snprintf(e->text, sizeof(e->text), "from %s",
                 (const char *)(ev["source"] | "?"));
    } else if (strcmp(type, "stopped") == 0) {
        strlcpy(e->type, "STOPPED", sizeof(e->type));
        e->sev = JSEV_STOP;
        fmt_dur(ev["run_seconds"] | 0UL, dur, sizeof(dur));
        snprintf(e->text, sizeof(e->text), "ran %s, %.0f°C, %.1fL left",
                 dur, (double)(ev["head_temp"] | 0.0f),
                 (double)(ev["fuel_l"] | 0.0f));
    } else if (strcmp(type, "emergency_stop") == 0) {
        strlcpy(e->type, "EMERG", sizeof(e->type));
        e->sev = JSEV_ALARM;
        fmt_dur(ev["run_seconds"] | 0UL, dur, sizeof(dur));
        char reason[16];
        strlcpy(reason, ev["reason"] | "?", sizeof(reason));
        for (char *p = reason; *p; p++) *p = toupper((unsigned char)*p);
        snprintf(e->text, sizeof(e->text), "%s! ran %s, %.0f°C",
                 reason, dur, (double)(ev["head_temp"] | 0.0f));
    } else if (strcmp(type, "charge_start") == 0) {
        strlcpy(e->type, "CHG ON", sizeof(e->type));
        e->sev = JSEV_CHARGE;
        snprintf(e->text, sizeof(e->text), "bat %.2fV (%s)",
                 (double)(ev["bat_v"] | 0.0f),
                 (ev["auto"] | 0) ? "auto" : "manual");
    } else if (strcmp(type, "charge_stop") == 0) {
        strlcpy(e->type, "CHG OFF", sizeof(e->type));
        e->sev = JSEV_CHARGE;
        fmt_dur(ev["charge_seconds"] | 0UL, dur, sizeof(dur));
        snprintf(e->text, sizeof(e->text), "bat %.2fV, %s, %s",
                 (double)(ev["bat_v"] | 0.0f), dur,
                 (const char *)(ev["reason"] | "?"));
    } else if (strcmp(type, "error") == 0) {
        strlcpy(e->type, "ERROR", sizeof(e->type));
        e->sev = JSEV_ALARM;
        snprintf(e->text, sizeof(e->text), "code %d, value %.1f",
                 ev["code"] | 0, (double)(ev["value"] | 0.0f));
    } else if (strcmp(type, "task_wdt") == 0) {
        strlcpy(e->type, "WDT", sizeof(e->type));
        e->sev = JSEV_ALARM;
        fmt_dur(ev["uptime_s"] | 0UL, dur, sizeof(dur));
        snprintf(e->text, sizeof(e->text), "starved after %s uptime: %s",
                 dur, (const char *)(ev["tasks"] | "?"));
    } else if (strcmp(type, "fw_update") == 0) {
        strlcpy(e->type, "FW UPD", sizeof(e->type));
        e->sev = JSEV_SYS;
        const char *ver = ev["version"] | "";
        if (ver[0]) {
            snprintf(e->text, sizeof(e->text), "OTA update, %lu bytes, v%s",
                     (unsigned long)(ev["size"] | 0UL), ver);
        } else {
            snprintf(e->text, sizeof(e->text), "OTA update, %lu bytes",
                     (unsigned long)(ev["size"] | 0UL));
        }
    } else if (strcmp(type, "i2c_stuck") == 0) {
        strlcpy(e->type, "I2C", sizeof(e->type));
        e->sev = JSEV_ALARM;
        snprintf(e->text, sizeof(e->text), "I2CTask stalled %lu ms",
                 (unsigned long)(ev["stuck_ms"] | 0UL));
    } else if (strcmp(type, "sensor_fail") == 0) {
        strlcpy(e->type, "SENSOR", sizeof(e->type));
        e->sev = JSEV_ALARM;
        char sensor[16];
        strlcpy(sensor, ev["sensor"] | "?", sizeof(sensor));
        for (char *p = sensor; *p; p++) *p = toupper((unsigned char)*p);
        snprintf(e->text, sizeof(e->text), "read failed: %s", sensor);
    } else if (strcmp(type, "maintenance") == 0) {
        strlcpy(e->type, "MAINT", sizeof(e->type));
        e->sev = JSEV_MAINT;
        const char *note = ev["note"] | "";
        snprintf(e->text, sizeof(e->text), "%s @ %.1fh%s%s",
                 (const char *)(ev["maint"] | "?"),
                 (double)(ev["engine_hours"] | 0.0f),
                 note[0] ? " — " : "", note);
    } else {
        strlcpy(e->type, "?", sizeof(e->type));
        e->sev = JSEV_SYS;
        strlcpy(e->text, type, sizeof(e->text));
    }
}

// ---------------------------------------------------------------------------
// Fetch
// ---------------------------------------------------------------------------
static bool http_get_json(HTTPClient &http, const char *path,
                          JsonDocument &doc, char *err, size_t errlen) {
    char url[160];
    snprintf(url, sizeof(url), "http://%s:%u%s",
             g_config.gen_host, GEN_HTTP_PORT, path);
    if (!http.begin(url)) {
        snprintf(err, errlen, "bad URL");
        return false;
    }
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        snprintf(err, errlen, "HTTP %d", code);
        http.end();
        return false;
    }
    DeserializationError jerr = deserializeJson(doc, http.getStream());
    http.end();
    if (jerr != DeserializationError::Ok) {
        snprintf(err, errlen, "JSON: %s", jerr.c_str());
        return false;
    }
    return true;
}

static void show_status(const char *msg) {
    if (bsp_display_lock(100)) {
        ui_journal_set_status(msg);
        bsp_display_unlock();
    }
}

void journal_poll(bool allowed) {
    if (!s_refresh_req) return;
    if (!allowed) return;   // keep the request pending until WiFi is back
    if (strlen(g_config.gen_host) == 0) {
        s_refresh_req = false;
        s_maint_pending = -1;
        show_status("generator host not configured");
        return;
    }
    s_refresh_req = false;

    static JournalEntry entries[JOURNAL_MAX_EVENTS];
    char metrics_line[144] = "";
    char err[48];
    int n = 0;

    // Two sequential requests (/metrics then /log) share this loop() call —
    // journal_poll() blocks the Arduino loop task for its whole duration, so
    // worst case (host unreachable mid-fetch) must stay well under the 10 s
    // LVGL watchdog window even after the false-positive fix above (which
    // still requires the LVGL task to be idle enough between the two
    // requests to service a couple of 300 ms UI ticks; a single uninterrupted
    // block near 10 s leaves no room for that safety margin).
    HTTPClient http;
    http.setConnectTimeout(1500);
    http.setTimeout(2000);

    // A pending Record tap (Controls-tab maintenance bar) rides this same
    // poll — fire it first so /metrics right after already reflects the
    // just-recorded hours-since-last-service.
    if (s_maint_pending >= 0) {
        uint8_t sub = (uint8_t)s_maint_pending;
        s_maint_pending = -1;
        char path[64];
        snprintf(path, sizeof(path), "/maintenance?type=%s&note=touch",
                 MAINT_TYPE_NAMES[sub]);
        DynamicJsonDocument doc(128);
        if (http_get_json(http, path, doc, err, sizeof(err))) {
            char msg[64];
            snprintf(msg, sizeof(msg), "Recorded: %s", MAINT_TYPE_LABELS[sub]);
            show_status(msg);
        } else {
            Serial.printf("[JRN] /maintenance failed: %s\n", err);
            show_status(err);
        }
    }

    // Lifetime metrics → one summary line
    {
        DynamicJsonDocument doc(2048);
        if (http_get_json(http, "/metrics", doc, err, sizeof(err))) {
            snprintf(metrics_line, sizeof(metrics_line),
                     "%.1f h  •  %lu starts (%lu fail)  •  %lu emerg  •  "
                     "%lux charge %.1f h",
                     (double)(doc["run_hours"] | 0.0f),
                     (unsigned long)(doc["starts"] | 0UL),
                     (unsigned long)(doc["start_fails"] | 0UL),
                     (unsigned long)(doc["emergencies"] | 0UL),
                     (unsigned long)(doc["charge_cycles"] | 0UL),
                     (double)(doc["charge_hours"] | 0.0f));
        } else {
            Serial.printf("[JRN] /metrics failed: %s\n", err);
        }
    }

    // Event log
    {
        char path[32];
        snprintf(path, sizeof(path), "/log?count=%d", JOURNAL_MAX_EVENTS);
        DynamicJsonDocument doc(16384);
        if (!http_get_json(http, path, doc, err, sizeof(err))) {
            Serial.printf("[JRN] /log failed: %s\n", err);
            show_status(err);
            return;
        }
        uint32_t now_ts = doc["ts_now"] | 0UL;
        for (JsonObjectConst ev : doc["events"].as<JsonArrayConst>()) {
            if (n >= JOURNAL_MAX_EVENTS) break;
            fill_entry(ev, now_ts, &entries[n]);
            n++;
        }
    }

    Serial.printf("[JRN] Loaded %d events\n", n);
    if (bsp_display_lock(100)) {
        ui_journal_show(entries, n, metrics_line);
        bsp_display_unlock();
    }
}
