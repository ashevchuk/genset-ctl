#include "wifi_setup.h"

#include "wifi_net.h"
#include "ffat.h"
#include <mdns.h>
#include <stdio.h>
#include <string>

#include "esp_netif_sntp.h"

#include "config.h"
#include "debug.h"
#include "shared_defines.h" // WIFI_HOSTNAME_DEF

#define WIFI_OPEN_AP_SSID "Gen-Ctl"

// Still defined (non-static) in the main .cpp — read by setup()/loop() too,
// not just this file's own WiFi bring-up logic.
extern bool wifiModeIsAP;
extern char wifiHostname[33];

// First STA network credentials — saved for menu scan restore after
// wifi_net_multi_clear(). Only written here (wifi_config_apply()), only
// read via wifi_get_last_sta_creds() below — genuinely private to this file.
static char s_wifi_ssid[33] = {};
static char s_wifi_pass[65] = {};

void wifi_get_last_sta_creds(char *ssid_out, size_t ssid_len, char *pass_out, size_t pass_len) {
  strlcpy(ssid_out, s_wifi_ssid, ssid_len);
  strlcpy(pass_out, s_wifi_pass, pass_len);
}

// Runs on the event task on every STA got-IP.
static void on_got_ip(uint32_t ip) {
  (void)ip;
  // Start SNTP service now that we have an IP address
  esp_err_t ret = esp_netif_sntp_start();
  if (ret == ESP_OK) {
    SLOG_PRINTLN("SNTP started");
  } else if (ret == ESP_ERR_INVALID_STATE) {
    SLOG_PRINTLN("SNTP already started");
  } else {
    SLOG_PRINTF("Failed to start SNTP: %s\n", esp_err_to_name(ret));
  }
  // Start mDNS on first STA connect
  static bool mdns_started = false;
  if (!mdns_started) {
    mdns_started = true;
    wifi_mdns_start();
  }
}

void wifi_setup_begin(void) {
  wifi_net_init();
  wifi_net_on_got_ip(on_got_ip);
}

bool wifi_mdns_start(void) {
  if (mdns_init() != ESP_OK) return false;
  mdns_hostname_set(wifiHostname);
  mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);
  SLOG_PRINTF("mDNS: http://%s.local\n", wifiHostname);
  return true;
}

// ---- WiFi config (wifi.json) ----

// Non-static: oled_menu.cpp's WiFi-provisioning menu calls both directly.
bool wifi_config_load(JsonDocument &doc) {
  FILE *f = fopen(FFAT_MOUNT_POINT WIFI_JSON_PATH, "rb");
  if (f == nullptr) return false;
  std::string text;
  char buf[256];
  size_t n;
  while ((n = fread(buf, 1, sizeof(buf), f)) > 0) text.append(buf, n);
  fclose(f);
  return deserializeJson(doc, text) == DeserializationError::Ok;
}

bool wifi_config_save(const JsonDocument &doc) {
  // Overlay onto the on-disk document so a WiFi-only writer (OLED menu,
  // update_mode /api/wifi/save, first-boot portal) cannot wipe mqtt/limits/
  // timing/battery/engine/time. Top-level keys present in `doc` replace the
  // matching keys in full; keys `doc` does not mention stay as they were.
  // Found 2026-09-28: both the OLED save and update_mode built a WiFi-only
  // JsonDocument and this function then truncated /wifi.json to just that.
  JsonDocument out;
  const JsonDocument *to_write = &doc;
  if (wifi_config_load(out)) {
    for (JsonPairConst p : doc.as<JsonObjectConst>()) {
      out[p.key()] = p.value();
    }
    to_write = &out;
  }

  std::string text;
  serializeJson(*to_write, text);
  FILE *f = fopen(FFAT_MOUNT_POINT WIFI_JSON_PATH, "wb");
  if (f == nullptr) return false;
  bool ok = fwrite(text.data(), 1, text.size(), f) == text.size();
  return (fclose(f) == 0) && ok;
}

bool wifi_config_apply(const JsonDocument &doc) {
  const char *mode = doc["mode"] | "sta";
  wifiModeIsAP = (strcmp(mode, "ap") == 0);
  strlcpy(wifiHostname, doc["hostname"] | WIFI_HOSTNAME_DEF, sizeof(wifiHostname));
  wifi_net_multi_clear();
  wifi_net_set_hostname(wifiHostname);

  if (wifiModeIsAP) {
    JsonObjectConst ap = doc["ap"];
    if (ap.isNull()) return false;
    const char *ap_ssid = ap["ssid"] | "Gen-Ctl";
    const char *ap_pass = ap["password"] | "12345678";
    uint8_t ch = ap["channel"] | 1;
    bool hidden = ap["hidden"] | false;
    uint8_t max_conn = ap["max_conn"] | 4;
    if (!wifi_net_start_ap(ap_ssid, ap_pass, ch, hidden, max_conn, ap["ip"] | "192.168.4.1",
                           ap["gw"] | "192.168.4.1", ap["sn"] | "255.255.255.0"))
      return false;
    SLOG_PRINTF("AP: %s\n", ap_ssid);
    return true;
  }

  JsonObjectConst sta = doc["sta"];
  if (sta.isNull()) return false;
  JsonArrayConst nets = sta["networks"];
  int n_added = 0;
  for (JsonObjectConst n : nets) {
    const char *s = n["ssid"];
    const char *p = n["password"] | "";
    if (s && strlen(s) > 0) {
      wifi_net_multi_add(s, p);
      if (n_added == 0) {           // save first network for menu-scan restore
        strlcpy(s_wifi_ssid, s, sizeof(s_wifi_ssid));
        strlcpy(s_wifi_pass, p, sizeof(s_wifi_pass));
      }
      n_added++;
    }
  }
  if (n_added == 0) return false;

  bool dhcp = sta["dhcp"] | true;
  if (!dhcp) {
    wifi_net_set_static_ip(sta["ip"] | "", sta["gw"] | "", sta["sn"] | "", sta["dns1"] | "", sta["dns2"] | "");
  }
  return wifi_net_start_sta();
}

void wifi_start_open_ap(void) {
  wifiModeIsAP = true;
  strlcpy(wifiHostname, WIFI_HOSTNAME_DEF, sizeof(wifiHostname));
  wifi_net_multi_clear();
  wifi_net_set_hostname(wifiHostname);
  // No passphrase: this only runs without a wifi.json, so there is nothing
  // to protect yet and nobody has a password to type.
  if (!wifi_net_start_ap(WIFI_OPEN_AP_SSID, nullptr, 1, false, 4, "192.168.4.1", "192.168.4.1", "255.255.255.0")) {
    SLOG_PRINTLN("Open AP start failed");
    return;
  }
  SLOG_PRINTF("No wifi.json: open AP \"%s\", configure at http://192.168.4.1/wifi-config\n", WIFI_OPEN_AP_SSID);
}
