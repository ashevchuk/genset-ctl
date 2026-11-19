#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// WiFi bring-up: connection-state event logging, SNTP/mDNS kickoff on first
// IP, wifi.json load/save/apply, and the first-boot WiFiManager captive
// portal. Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp — see
// CLAUDE.md's "Code organization" and [[main-cpp-modularization]] project
// memory.

// Registered via WiFi.onEvent() in the main .cpp's setup() — logs every
// WiFi/Ethernet event to Serial, no other side effects except re-asserting
// WiFi.setSleep(false) on STA_CONNECTED (see the function body for why).
void WiFiEvent(arduino_event_id_t event);

// WARNING: called from a separate FreeRTOS task (thread)! Registered via
// WiFi.onEvent() for ARDUINO_EVENT_WIFI_STA_GOT_IP specifically. Starts
// SNTP and (once, on first STA connect) mDNS.
void WiFiGotIP(arduino_event_id_t event, arduino_event_info_t info);

// Raw wifi.json load/save — no parsing/validation, just the FFat file I/O.
// save() overlays `doc`'s top-level keys onto the existing file (if any)
// rather than truncating it, so a WiFi-only writer cannot wipe mqtt/limits/
// timing/battery/engine/time. Non-static: also called directly by
// oled_menu.cpp's WiFi-provisioning menu and update_mode.cpp's own reduced
// /api/wifi/save route.
bool wifi_config_load(JsonDocument &doc);
bool wifi_config_save(const JsonDocument &doc);

// Applies an already-loaded/parsed wifi.json document: sets wifiModeIsAP/
// wifiHostname, brings up AP or STA (asyncWifiMulti) accordingly. Called
// from the main .cpp's setup() (after wifi_config_load()) and from this
// file's own run_wifi_config_portal() (after a fresh save).
bool wifi_config_apply(const JsonDocument &doc);

// First-boot / no-wifi.json setup: a synchronous WiFiManager (tzapu)
// captive portal (AP "Gen-Ctl-Setup"), builds a wifi.json document
// from the portal's fields + WiFiManager's own scanned STA credentials,
// saves it, and applies it. Called from the main .cpp's setup() when
// wifi_config_load() fails.
void run_wifi_config_portal(void);

// Accessor for oled_menu.cpp's WiFi-scan-restore flow (menu_process_encoder(),
// MENU_WIFI case 1) — kept as a function rather than exposing the backing
// buffers directly, so they stay this file's own encapsulated storage.
void wifi_get_last_sta_creds(char *ssid_out, size_t ssid_len, char *pass_out, size_t pass_len);
