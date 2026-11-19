/*
 * remote_io.h — hardware remote start/stop buttons, an unattended autostart
 * trigger, and two status outputs, wired directly to the display
 * controller's own free GPIOs.
 *
 * All six are a second, unattended command source into the exact same
 * xWebCommandQueue / start_retry / stop path the Controls tab's touchscreen
 * buttons already use (ui.cpp's btn_start_cb/btn_retry_cb/btn_stop_cb — same
 * ws_send() calls, same text commands) — the generator controller's own
 * state-machine safety gating (STATE_STOPPING_LOAD_OFF, REG_START refusal,
 * etc.) applies identically regardless of which of these sent the command.
 *
 * Pin map (ESP32-S3, JC3248W535EN board — see esp_bsp.h/display.ino for the
 * board's own reserved pins; these six are free and unused elsewhere):
 *   IO6  — momentary push button, start. Debounced falling edge (grounded)
 *          -> ws_send("start_retry") once.
 *   IO7  — momentary push button, stop.  Debounced falling edge (grounded)
 *          -> ws_send("stop") once.
 *   IO9  — autostart-armed level switch. Grounded = autostart permitted.
 *          Gates IO14's falling-edge action only; otherwise read-only, no
 *          command of its own.
 *   IO14 — autostart trigger (level). Debounced falling edge (grounded)
 *          while IO9 reads armed -> ws_send("start_retry") once. Debounced
 *          rising edge (released) -> ws_send("stop") once, UNCONDITIONALLY —
 *          regardless of IO9's state at that moment and regardless of
 *          whether this trigger or something else (touchscreen, web, MQTT)
 *          started the generator. If there is no WS link at the moment of
 *          either edge, nothing is sent at all — no queueing, no retry once
 *          the link returns, matching ws_send()'s own fire-and-forget
 *          contract. Retrying a failed crank is the controller's own
 *          start_retry state machine's job, not this code's.
 *   IO15 — output, HIGH while ws_client_connected() && gen.valid (the same
 *          expression ui_set_ws_status() already uses). Re-evaluated every
 *          poll — drops to LOW immediately on link loss, since that is
 *          this pin's entire job.
 *   IO16 — output, HIGH while gen.running, driven only while gen.valid is
 *          true. When gen.valid goes false (stale data / lost link), the
 *          pin is deliberately left untouched at its last driven level
 *          rather than forced low: gen.running itself is never cleared on
 *          staleness (see display.ino's stale-data block, which only
 *          clears gen.valid), so writing this pin through unconditionally
 *          would flip a "confirmed running" output to "confirmed stopped"
 *          on a lost link, when the truth is just "unknown."
 *
 * All four inputs use INPUT_PULLUP with active-low wiring ("притягнуто до
 * землі" = active), debounced in software (millis()-based, no delay()).
 */
#pragma once
#include <Arduino.h>
#include "ws_client.h"

void remote_io_init(void);

// Call once per loop() iteration, unconditionally (not gated behind the UI's
// own 300ms refresh tick — button latency and debounce timing both want the
// tighter cadence). Pass the current g_gen snapshot and the same link-ok
// expression ui_set_ws_status() uses.
void remote_io_poll(const GenData &gen, bool link_ok);
