#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

// ESP-NOW fallback link to the display panel(s) (display/), protocol v2.
// One fleet-wide key (used as both PMK and LMK) is shared by every
// controller and display of the installation. The controller broadcasts a
// beacon with its name (wifi.json "hostname"); a display picks a generator
// from the beacons it hears and joins with a hello, after which state and
// commands flow unicast and encrypted. Commands reuse xWebCommandQueue
// (FLOG_SRC_ESPNOW) — same StateMachineTask gating as HTTP/WS/MQTT.
// ESPNOW.md has the wire format and the channel/coexistence handling.

#define ESPNOW_KEY_LEN 16  // matches ESP_NOW_KEY_LEN (esp_now.h)

struct EspNowConfig_t {
    bool    enabled;
    uint8_t key[ESPNOW_KEY_LEN];  // fleet key, used as PMK and per-peer LMK
};

extern EspNowConfig_t g_espnow_config;

// Parses wifi.json's "espnow" object ({"enabled", "key": 32 hex chars}) into
// g_espnow_config. Called from the main .cpp's config-apply path like every
// other *_config_load(). Missing/malformed key = disabled, not a guess.
void espnow_config_load(const JsonDocument &doc);

// Keeps ESP-NOW reachable while the STA has no AP (ESPNOW.md "Channel
// handling"). Call every loop() pass in STA mode. `connected`: STA has an
// IP; `reconnect_idle`: nothing else (AsyncWiFiMulti, OLED menu scan) is
// using the radio. Returns true when the home channel shows our SSID again
// and the caller should start a full reconnect right away. No-op (false)
// when ESP-NOW is disabled.
bool espnow_wifi_supervise(bool connected, bool reconnect_idle);

// True while espnow_wifi_supervise()'s own single-channel scan is in flight —
// the caller must not start another scan (AsyncWiFiMulti) until it ends.
bool espnow_wifi_probe_running(void);

// Channel of the last AP association (0 = unknown or ESP-NOW disabled).
uint8_t espnow_home_channel(void);

// EspNowTask: esp_now_init(), beacon, joining displays from hellos, 1 Hz
// encrypted state push to every joined display. Received commands go
// straight from the ESP-NOW receive callback (WiFi driver task, not an ISR)
// into xWebCommandQueue.
void EspNowTask(void *pvParameters);
