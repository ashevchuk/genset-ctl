#pragma once
#include <lvgl.h>
#include "ws_client.h"
#include "journal.h"

// Display size after 90° rotation
#define UI_W 480
#define UI_H 320

typedef enum {
    WIFI_ST_OFF,         // not connected
    WIFI_ST_CONNECTING,  // connecting / reconnecting
    WIFI_ST_CONNECTED,   // connected
} wifi_ui_state_t;

void ui_init(void);
void ui_update(const GenData &d);
void ui_set_wifi_status(wifi_ui_state_t st, int rssi);
typedef enum {
    LINK_UI_NONE,    // no fresh data from the generator
    LINK_UI_WS,      // WebSocket via the router
    LINK_UI_ESPNOW,  // ESP-NOW fallback
} link_ui_state_t;

// Call after ui_set_wifi_status(): with LINK_UI_ESPNOW the signal bars switch
// to espnow_rssi (the ESP-NOW link as heard here).
void ui_set_link_status(link_ui_state_t link, int espnow_rssi);
// Config tab "Generator Host" field, when gen_host changes outside the tab.
void ui_set_gen_host(const char *host);
// Config tab generator dropdown — merges the ESP-NOW list (esp_now_client.h)
// and the LAN UDP discovery list (udp_discovery.h), tagged by source;
// rate-limited internally, call every UI tick.
void ui_gen_list_refresh(void);
// True once after the operator picks a LAN-sourced (not ESP-NOW) entry from
// that dropdown — unlike an ESP-NOW pick, there is no radio join involved,
// it is just a ready-made IP for the caller to point ws_client_init() at.
bool ui_take_lan_pick(char *buf, size_t len);
// Panel Li-ion (GPIO5), not the generator battery. Hidden when absent.
// Call with bsp_display_lock held.
void ui_set_panel_battery(bool present, float volts);
// Called from Config tab: populate WiFi network list
void ui_wifi_scan_results(int count);  // call after WiFi.scanComplete() >= 0
// Called when user taps "Connect" in config tab
void ui_on_connect_request(void (*cb)(const char *ssid, const char *pass,
                                      const char *host));

// OTA progress overlay (call with bsp_display_lock held)
void ui_ota_show(void);
void ui_ota_set_progress(int percent);
void ui_ota_hide(void);

// Journal tab (call with bsp_display_lock held)
void ui_journal_show(const JournalEntry *entries, int count,
                     const char *metrics_line);
void ui_journal_set_status(const char *msg);
