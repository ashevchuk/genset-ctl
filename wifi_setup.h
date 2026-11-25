#pragma once

#include <stdint.h>
#include <stddef.h>
#include <ArduinoJson.h>

// WiFi bring-up: connection-state event logging, SNTP/mDNS kickoff on first
// IP, wifi.json load/save/apply, and the open-AP fallback used when there
// is no wifi.json. Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp — see
// CLAUDE.md's "Code organization" and [[main-cpp-modularization]] project
// memory.

// Brings up the WiFi driver (wifi_net_init()) and hooks SNTP start + mDNS
// (once) onto the first STA IP. Call once, early in setup().
void wifi_setup_begin(void);

// mDNS <hostname>.local + _http._tcp:80. Called on the first STA IP, and
// directly in AP mode.
bool wifi_mdns_start(void);

// Raw wifi.json load/save — no parsing/validation, just the FFat file I/O.
// save() overlays `doc`'s top-level keys onto the existing file (if any)
// rather than truncating it, so a WiFi-only writer cannot wipe mqtt/limits/
// timing/battery/engine/time. Non-static: also called directly by
// oled_menu.cpp's WiFi-provisioning menu and update_mode.cpp's own reduced
// /api/wifi/save route.
bool wifi_config_load(JsonDocument &doc);
bool wifi_config_save(const JsonDocument &doc);

// Applies an already-loaded/parsed wifi.json document: sets wifiModeIsAP/
// wifiHostname, brings up AP or STA (wifi_net multi-network) accordingly. Called
// from the main .cpp's setup() after wifi_config_load().
bool wifi_config_apply(const JsonDocument &doc);

// No-wifi.json fallback: an open (passwordless) AP "Gen-Ctl" at
// 192.168.4.1; the normal HTTP server's /wifi-config then writes the first
// wifi.json. Called from the main .cpp's setup() when wifi_config_load()
// fails.
void wifi_start_open_ap(void);

// Accessor for oled_menu.cpp's WiFi-scan-restore flow (menu_process_encoder(),
// MENU_WIFI case 1) — kept as a function rather than exposing the backing
// buffers directly, so they stay this file's own encapsulated storage.
void wifi_get_last_sta_creds(char *ssid_out, size_t ssid_len, char *pass_out, size_t pass_len);
