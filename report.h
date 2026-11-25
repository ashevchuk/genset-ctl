#pragma once

// ReportTask: periodic full-state broadcast to the /ws clients
// (http_ws_broadcast(), http_server.h), plus a few
// once-per-boot/periodic housekeeping checks that
// happened to live in the same task loop (I2CTask stall detection, delayed
// OTA-image health confirmation when a soak timer was armed at boot, heap
// trend logging).
// Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's
// "Code organization" and [[main-cpp-modularization]] project memory for
// the pattern this follows).
void ReportTask(void *pvParameters);

// Explicitly rejects (and reboots away from) an OTA image the bootloader
// has already marked ABORTED/INVALID — i.e. a previous boot attempt of
// this same image crashed before ever confirming itself healthy. Call
// once, as early as possible in setup() — see report.cpp for why this is
// needed and can't just be left to the bootloader's own passive marking.
void ota_reject_if_aborted(void);

// If the running image is PENDING_VERIFY, arms a FreeRTOS one-shot timer
// for OTA_CONFIRM_HEALTHY_AFTER_MS; the timer only sets a flag, and
// ReportTask performs esp_ota_mark_app_valid_cancel_rollback() so the
// flash write stays off the timer daemon. No-op when the image is already
// VALID (or any non-pending state). Call once from setup() immediately
// after ota_reject_if_aborted().
void ota_arm_confirm_soak(void);
