#include "config.h"
#include <SPIFFS.h>
#include <ArduinoJson.h>

Config g_config = {
    .wifi_ssid = "",
    .wifi_pass = "",
    .gen_host  = "192.168.1.100",
    .espnow_key      = "",
    .espnow_gen_mac  = "",
    .espnow_gen_name = "",
};

void config_init(void) {
    if (!SPIFFS.begin(true)) {
        Serial.println("[CFG] SPIFFS mount failed");
        return;
    }
    config_load();
}

bool config_load(void) {
    if (!SPIFFS.exists(CONFIG_FILE)) return false;
    File f = SPIFFS.open(CONFIG_FILE, "r");
    if (!f) return false;

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (err) return false;

    strlcpy(g_config.wifi_ssid, doc["wifi_ssid"] | "",              sizeof(g_config.wifi_ssid));
    strlcpy(g_config.wifi_pass, doc["wifi_pass"] | "",              sizeof(g_config.wifi_pass));
    strlcpy(g_config.gen_host,  doc["gen_host"]  | "192.168.1.100", sizeof(g_config.gen_host));
    strlcpy(g_config.espnow_key,      doc["espnow_key"]      | "", sizeof(g_config.espnow_key));
    strlcpy(g_config.espnow_gen_mac,  doc["espnow_gen_mac"]  | "", sizeof(g_config.espnow_gen_mac));
    strlcpy(g_config.espnow_gen_name, doc["espnow_gen_name"] | "", sizeof(g_config.espnow_gen_name));

    Serial.printf("[CFG] Loaded: ssid=%s host=%s\n",
                  g_config.wifi_ssid, g_config.gen_host);
    return true;
}

void config_save(void) {
    File f = SPIFFS.open(CONFIG_FILE, "w");
    if (!f) { Serial.println("[CFG] Save failed"); return; }

    StaticJsonDocument<512> doc;
    doc["wifi_ssid"] = g_config.wifi_ssid;
    doc["wifi_pass"] = g_config.wifi_pass;
    doc["gen_host"]  = g_config.gen_host;
    doc["espnow_key"]      = g_config.espnow_key;
    doc["espnow_gen_mac"]  = g_config.espnow_gen_mac;
    doc["espnow_gen_name"] = g_config.espnow_gen_name;
    serializeJson(doc, f);
    f.close();
    Serial.println("[CFG] Saved");
}
