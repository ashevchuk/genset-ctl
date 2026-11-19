#pragma once
#include <Arduino.h>

#define CONFIG_FILE "/config.json"

// Fixed, not user-configurable: the main controller's HTTP/WS ports never
// change (WebSocketsServer listens one port up from HTTP — see display.ino).
#define GEN_HTTP_PORT 80

struct Config {
    char wifi_ssid[64];
    char wifi_pass[64];
    char gen_host[64];
    // ESP-NOW fallback link (esp_now_client.cpp, ../ESPNOW.md). Fleet key as
    // 32 hex chars (empty = link disabled); the selected generator's STA MAC
    // ("aa:bb:cc:dd:ee:ff", empty = none) and beacon name.
    char espnow_key[33];
    char espnow_gen_mac[18];
    char espnow_gen_name[25];
};

extern Config g_config;

void config_init(void);
void config_save(void);
bool config_load(void);
