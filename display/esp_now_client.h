#pragma once
#include <Arduino.h>

// ESP-NOW fallback link to a generator controller, protocol v2 (../ESPNOW.md;
// the packet structs in esp_now_client.cpp must match ../esp_now_link.cpp
// byte for byte). One fleet key (g_config.espnow_key) for the installation;
// generators announce themselves by name in a broadcast beacon, and the
// panel talks to the one selected in g_config.espnow_gen_mac/_name.
//
// WS stays the primary path: received state only writes g_gen, and
// ws_send() only falls back to espnow_send_text(), while WS is silent
// (!ws_client_fresh()).

#define ESPNOW_NAME_LEN 24

// Reads g_config.espnow_*; a missing/malformed key leaves the link disabled.
// Call after WiFi.mode(WIFI_STA). Safe to call again after a radio restart
// (WiFi.mode(WIFI_OFF) -> WIFI_STA) or a config change — deinitializes first.
void espnow_client_init(void);

bool espnow_client_enabled(void);

// RSSI of the last state packet from the selected generator, as heard here.
int espnow_client_rssi(void);

// True while state packets from the selected generator keep arriving.
bool espnow_client_active(void);

// Same text commands ws_send() takes: "start", "start_retry", "stop", or a
// register index ("0".."15") for a toggle. Returns false if not sent.
bool espnow_send_text(const char *msg);

// Generators heard recently, plus the selected one even if not heard.
struct EspNowGenInfo {
    uint8_t mac[6];
    char    name[ESPNOW_NAME_LEN + 1];
    int     rssi;      // 0 when not heard
    bool    seen;
    bool    selected;
};
int espnow_list_generators(EspNowGenInfo *out, int max);

// Select a generator (mac == nullptr: none). Applied, persisted and joined
// on the next espnow_client_poll(); safe to call from the LVGL task.
void espnow_select_generator(const uint8_t *mac, const char *name);

// The selected generator's STA IP, learned from its encrypted state packets.
// Returns true once each time it changes.
bool espnow_take_gen_ip(char *buf, size_t len);

// Keeps ESP-NOW reachable while the STA has no AP (../ESPNOW.md "Channel
// handling"). Call every loop() pass. `may_probe`: STA is not connected and
// no connect attempt or UI scan is using the radio. Returns true when the
// home channel shows the configured SSID again and the caller should
// WiFi.begin(ssid, pass, espnow_home_channel()) now. A no-op while
// espnow_wifi_join_pinned() holds the radio (see below).
bool espnow_wifi_supervise(bool connected, bool may_probe);
bool espnow_wifi_probe_running(void);
uint8_t espnow_home_channel(void);   // 0 = unknown (never associated)

// True right after a fresh selection (espnow_select_generator()) while its
// channel is known from a recent beacon and it hasn't gone active yet — the
// radio is pinned there and the caller must not run its own AP full-sweep
// reconnect either, or the two fights would starve whichever channel the
// radio should actually be sitting on. Times out on its own
// (ESPNOW_PIN_TIMEOUT_MS) so a generator that never answers can't block the
// panel's own WiFi reconnection forever.
bool espnow_wifi_join_pinned(void);

// Pending selection, hello/keepalive, start/stop resends. Call every loop().
void espnow_client_poll(void);

// JSON status for the OTA server's GET /espnow (no key material).
void espnow_client_status_json(char *buf, size_t len);
