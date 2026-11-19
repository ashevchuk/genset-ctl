/*
 * journal.h — FRAM event journal viewer (data side).
 * Fetches /log and /metrics from the generator controller over HTTP,
 * formats entries into plain structs; ui.cpp renders them.
 */
#pragma once
#include <Arduino.h>

#define JOURNAL_MAX_EVENTS 25

// Maintenance subtypes — index-aligned with the controller's FLOG_MAINT_*
// enum (fram_log.h) and its MAINT_TYPE_LABELS (esp32-gen-ctl-ai-rtos.ino).
// Nothing here is fetched over the wire from the controller — if the enum
// order ever changes on that side, update these two tables by hand.
#define MAINT_TYPES 6
extern const char *const MAINT_TYPE_NAMES[MAINT_TYPES];   // wire name for /maintenance?type=
extern const char *const MAINT_TYPE_LABELS[MAINT_TYPES];  // UI label

// Row severity → UI color
enum : uint8_t {
    JSEV_SYS = 0,    // boot / time sync — dim
    JSEV_START,      // green
    JSEV_STOP,       // normal text
    JSEV_ALARM,      // red
    JSEV_CHARGE,     // blue
    JSEV_MAINT,      // orange
};

typedef struct {
    char    age[10];   // "3h", "12d", "?" when clock not synced
    char    type[10];  // short label, e.g. "STARTED"
    char    text[80];  // details line
    uint8_t sev;       // JSEV_*
} JournalEntry;

// Ask for a (re)fetch — safe to call from LVGL callbacks.
void journal_request_refresh(void);

// Ask to record a completed maintenance action (fires GET /maintenance on
// the next journal_poll(), then forces a refresh so the Journal tab's
// figures update too). Safe to call from LVGL callbacks.
void journal_request_maintenance(uint8_t subtype);

// Perform the pending fetch if any. Call from loop() (may block for the
// duration of two HTTP GETs to the controller). `allowed` = WiFi up and
// no OTA in progress.
void journal_poll(bool allowed);
