#include "wifi_setup.h"

#include <WiFiManager.h>
#include <AsyncWiFiMulti.h>
#include <ESPmDNS.h>
#include <FS.h>

#include "esp_netif_sntp.h"

#include "config.h"
#include "debug.h"
#include "shared_defines.h" // WIFI_HOSTNAME_DEF

#define WIFI_PORTAL_SSID  "Gen-Ctl-Setup"
#define WIFI_PORTAL_PASS  "fkujhbnv666"

// Still defined (non-static) in the main .cpp — read by setup()/loop() too,
// not just this file's own WiFi bring-up logic.
extern bool wifiModeIsAP;
extern String wifiHostname;
extern GuLinux::AsyncWiFiMulti asyncWifiMulti;
extern FS *filesystem;

// First STA network credentials — saved for menu scan restore after
// asyncWifiMulti.clear(). Only written here (wifi_config_apply()), only
// read via wifi_get_last_sta_creds() below — genuinely private to this file.
static char s_wifi_ssid[33] = {};
static char s_wifi_pass[65] = {};

void wifi_get_last_sta_creds(char *ssid_out, size_t ssid_len, char *pass_out, size_t pass_len) {
  strlcpy(ssid_out, s_wifi_ssid, ssid_len);
  strlcpy(pass_out, s_wifi_pass, pass_len);
}

void WiFiEvent(WiFiEvent_t event) {
  SLOG_PRINTF("[WiFi-event] event: %d\n", event);

  switch (event) {
  case ARDUINO_EVENT_WIFI_READY:
    SLOG_PRINTLN("WiFi interface ready");
    break;
  case ARDUINO_EVENT_WIFI_SCAN_DONE:
    SLOG_PRINTLN("Completed scan for access points");
    break;
  case ARDUINO_EVENT_WIFI_STA_START:
    SLOG_PRINTLN("WiFi client started");
    break;
  case ARDUINO_EVENT_WIFI_STA_STOP:
    SLOG_PRINTLN("WiFi clients stopped");
    break;
  case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    SLOG_PRINTLN("Connected to access point");
    // wifi_config_apply() sets this once at boot, before the very first
    // WiFi.begin() — every later reconnect (AsyncWiFiMulti::tryNextAP()'s
    // own WiFi.begin() calls) doesn't repeat it. Re-assert here, right after
    // L2 association and before DHCP runs, as a defensive measure against
    // modem-sleep possibly missing the AP's buffered DHCP offer/ack in that
    // window — matches a hostapd log showing full WPA handshake success
    // followed by total station silence until the AP's inactivity deauth.
    WiFi.setSleep(false);
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    SLOG_PRINTLN("Disconnected from WiFi access point");
    break;
  case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
    SLOG_PRINTLN("Authentication mode of access point has changed");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    SLOG_PRINT("Obtained IP address: ");
    SLOG_PRINTLN(WiFi.localIP());
    break;
  case ARDUINO_EVENT_WIFI_STA_LOST_IP:
    SLOG_PRINTLN("Lost IP address and IP address is reset to 0");
    break;
  case ARDUINO_EVENT_WPS_ER_SUCCESS:
    SLOG_PRINTLN("WiFi Protected Setup (WPS): succeeded in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_FAILED:
    SLOG_PRINTLN("WiFi Protected Setup (WPS): failed in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_TIMEOUT:
    SLOG_PRINTLN("WiFi Protected Setup (WPS): timeout in enrollee mode");
    break;
  case ARDUINO_EVENT_WPS_ER_PIN:
    SLOG_PRINTLN("WiFi Protected Setup (WPS): pin code in enrollee mode");
    break;
  case ARDUINO_EVENT_WIFI_AP_START:
    SLOG_PRINTLN("WiFi access point started");
    break;
  case ARDUINO_EVENT_WIFI_AP_STOP:
    SLOG_PRINTLN("WiFi access point  stopped");
    break;
  case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
    SLOG_PRINTLN("Client connected");
    break;
  case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
    SLOG_PRINTLN("Client disconnected");
    break;
  case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
    SLOG_PRINTLN("Assigned IP address to client");
    break;
  case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
    SLOG_PRINTLN("Received probe request");
    break;
  case ARDUINO_EVENT_WIFI_AP_GOT_IP6:
    SLOG_PRINTLN("AP IPv6 is preferred");
    break;
  case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
    SLOG_PRINTLN("STA IPv6 is preferred");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP6:
    SLOG_PRINTLN("Ethernet IPv6 is preferred");
    break;
  case ARDUINO_EVENT_ETH_START:
    SLOG_PRINTLN("Ethernet started");
    break;
  case ARDUINO_EVENT_ETH_STOP:
    SLOG_PRINTLN("Ethernet stopped");
    break;
  case ARDUINO_EVENT_ETH_CONNECTED:
    SLOG_PRINTLN("Ethernet connected");
    break;
  case ARDUINO_EVENT_ETH_DISCONNECTED:
    SLOG_PRINTLN("Ethernet disconnected");
    break;
  case ARDUINO_EVENT_ETH_GOT_IP:
    SLOG_PRINTLN("Obtained IP address");
    break;
  default:
    break;
  }
}

// WARNING: This function is called from a separate FreeRTOS task (thread)!
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info) {
  SLOG_PRINT("WiFi connected, IP: ");
  SLOG_PRINTLN(IPAddress(info.got_ip.ip_info.ip.addr));

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
    if (MDNS.begin(wifiHostname.c_str())) {
      MDNS.addService("http", "tcp", 80);
      SLOG_PRINTF("mDNS: http://%s.local\n", wifiHostname.c_str());
    }
  }

}

// ---- WiFi config (wifi.json) ----

// Non-static: oled_menu.cpp's WiFi-provisioning menu calls both directly.
bool wifi_config_load(JsonDocument &doc) {
  File f = filesystem->open(WIFI_JSON_PATH, "r");
  if (!f || f.isDirectory()) {
    if (f) f.close();
    return false;
  }
  DeserializationError err = deserializeJson(doc, f);
  f.close();
  return (err == DeserializationError::Ok);
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

  File f = filesystem->open(WIFI_JSON_PATH, "w");
  if (!f) return false;
  serializeJson(*to_write, f);
  f.close();
  return true;
}

bool wifi_config_apply(const JsonDocument &doc) {
  const char *mode = doc["mode"] | "sta";
  wifiModeIsAP = (strcmp(mode, "ap") == 0);
  wifiHostname = doc["hostname"] | WIFI_HOSTNAME_DEF;
  asyncWifiMulti.clear();

  if (wifiModeIsAP) {
    JsonObjectConst ap = doc["ap"];
    if (ap.isNull()) return false;
    const char *ap_ssid = ap["ssid"] | "Gen-Ctl";
    const char *ap_pass = ap["password"] | "12345678";
    uint8_t ch = ap["channel"] | 1;
    bool hidden = ap["hidden"] | false;
    uint8_t max_conn = ap["max_conn"] | 4;
    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(ap_ssid, ap_pass, ch, hidden, max_conn)) return false;
    IPAddress ip, gw, sn;
    if (ip.fromString(ap["ip"] | "192.168.4.1") &&
        gw.fromString(ap["gw"] | "192.168.4.1") &&
        sn.fromString(ap["sn"] | "255.255.255.0"))
      WiFi.softAPConfig(ip, gw, sn);
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
      asyncWifiMulti.addAP(s, p);
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
    IPAddress ip, gw, sn, d1, d2;
    if (ip.fromString(sta["ip"] | "") && gw.fromString(sta["gw"] | "") &&
        sn.fromString(sta["sn"] | "")) {
      bool has_d1 = d1.fromString(sta["dns1"] | "");
      bool has_d2 = d2.fromString(sta["dns2"] | "");
      if (has_d1 && has_d2)
        WiFi.config(ip, gw, sn, d1, d2);
      else if (has_d1)
        WiFi.config(ip, gw, sn, d1);
      else
        WiFi.config(ip, gw, sn);
    }
  }
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setHostname(wifiHostname.c_str());
  return true;
}

void run_wifi_config_portal(void) {
  // Sync WiFiManager (tzapu) creates and owns its own internal WebServer +
  // DNSServer — no server/dns objects to construct or pass in here, unlike
  // ESPAsync_WiFiManager. startConfigPortal() calls shutdownConfigPortal()
  // internally before returning, so its internal HTTP server is already
  // torn down by the time this function returns — no port-80 conflict with
  // the main webServer.begin() called later in setup().
  WiFiManager wm;

  // No built-in multi-network API in this (sync) library — the async one's
  // wm.getSSID(i)/getPW(i) (i=0..3) doesn't exist here. Network slot 1 is
  // whatever the user actually picks/enters through the portal's own
  // standard scan-and-connect flow — that's captured via
  // getWiFiSSID()/getWiFiPass() below, NOT a custom parameter (confirmed
  // live: a first attempt using a blank custom "sta_ssid_1" text field
  // instead saved an empty networks array, since nobody fills in a
  // redundant duplicate field when the normal picker already did the job).
  // Slots 2-4 are additional/backup networks with no portal-flow
  // equivalent, so those stay custom parameters.
  WiFiManagerParameter p_mode("mode", "Mode (ap/sta)", "sta", 4);
  WiFiManagerParameter p_hostname("hostname", "Hostname", wifiHostname.c_str(), 33);
  WiFiManagerParameter p_ap_ssid("ap_ssid", "AP SSID", "Gen-Ctl", 33);
  WiFiManagerParameter p_ap_pass("ap_pass", "AP Password", "12345678", 65);
  WiFiManagerParameter p_ap_ch("ap_ch", "AP Channel (1-13)", "1", 4);
  WiFiManagerParameter p_ap_hidden("ap_hidden", "AP Hidden (0/1)", "0", 2);
  WiFiManagerParameter p_sta_dhcp("sta_dhcp", "STA DHCP (0/1)", "1", 2);
  WiFiManagerParameter p_sta_ip("sta_ip", "STA Static IP", "", 16);
  WiFiManagerParameter p_sta_gw("sta_gw", "STA Gateway", "", 16);
  WiFiManagerParameter p_sta_sn("sta_sn", "STA Subnet", "255.255.255.0", 16);
  WiFiManagerParameter p_sta_dns1("sta_dns1", "STA DNS1", "", 16);
  WiFiManagerParameter p_sta_dns2("sta_dns2", "STA DNS2", "", 16);
  WiFiManagerParameter p_sta_ssid_2("sta_ssid_2", "Backup STA SSID 2", "", 33);
  WiFiManagerParameter p_sta_pass_2("sta_pass_2", "Backup STA Password 2", "", 65);
  WiFiManagerParameter p_sta_ssid_3("sta_ssid_3", "Backup STA SSID 3", "", 33);
  WiFiManagerParameter p_sta_pass_3("sta_pass_3", "Backup STA Password 3", "", 65);
  WiFiManagerParameter p_sta_ssid_4("sta_ssid_4", "Backup STA SSID 4", "", 33);
  WiFiManagerParameter p_sta_pass_4("sta_pass_4", "Backup STA Password 4", "", 65);
  wm.addParameter(&p_mode);
  wm.addParameter(&p_hostname);
  wm.addParameter(&p_ap_ssid);
  wm.addParameter(&p_ap_pass);
  wm.addParameter(&p_ap_ch);
  wm.addParameter(&p_ap_hidden);
  wm.addParameter(&p_sta_dhcp);
  wm.addParameter(&p_sta_ip);
  wm.addParameter(&p_sta_gw);
  wm.addParameter(&p_sta_sn);
  wm.addParameter(&p_sta_dns1);
  wm.addParameter(&p_sta_dns2);
  wm.addParameter(&p_sta_ssid_2);
  wm.addParameter(&p_sta_pass_2);
  wm.addParameter(&p_sta_ssid_3);
  wm.addParameter(&p_sta_pass_3);
  wm.addParameter(&p_sta_ssid_4);
  wm.addParameter(&p_sta_pass_4);

  wm.setConfigPortalTimeout(180);
  wm.setMinimumSignalQuality(-1);
  WiFi.mode(WIFI_AP_STA);
  if (!wm.startConfigPortal(WIFI_PORTAL_SSID, WIFI_PORTAL_PASS)) {
    SLOG_PRINTLN("Config portal failed");
    return;
  }

  JsonDocument doc;
  doc["mode"] = p_mode.getValue();
  doc["hostname"] = p_hostname.getValue();
  JsonObject ap = doc["ap"].to<JsonObject>();
  ap["ssid"] = p_ap_ssid.getValue();
  ap["password"] = p_ap_pass.getValue();
  ap["channel"] = atoi(p_ap_ch.getValue());
  ap["hidden"] = (atoi(p_ap_hidden.getValue()) != 0);
  ap["max_conn"] = 4;
  ap["ip"] = "192.168.4.1";
  ap["gw"] = "192.168.4.1";
  ap["sn"] = "255.255.255.0";
  JsonObject sta = doc["sta"].to<JsonObject>();
  sta["dhcp"] = (atoi(p_sta_dhcp.getValue()) != 0);
  sta["ip"] = p_sta_ip.getValue();
  sta["gw"] = p_sta_gw.getValue();
  sta["sn"] = p_sta_sn.getValue();
  sta["dns1"] = p_sta_dns1.getValue();
  sta["dns2"] = p_sta_dns2.getValue();
  JsonArray nets = sta["networks"].to<JsonArray>();
  // Slot 1: whatever WiFiManager's own scan-and-connect flow captured this
  // session (WiFi.begin() persists it, so this reads back correctly even
  // though startConfigPortal() doesn't return the credentials directly).
  String primarySsid = wm.getWiFiSSID();
  String primaryPass = wm.getWiFiPass();
  if (primarySsid.length() > 0 && primaryPass.length() >= 8) {
    JsonObject n = nets.add<JsonObject>();
    n["ssid"] = primarySsid;
    n["password"] = primaryPass;
  }
  const WiFiManagerParameter *ssidParams[3] = {&p_sta_ssid_2, &p_sta_ssid_3, &p_sta_ssid_4};
  const WiFiManagerParameter *passParams[3] = {&p_sta_pass_2, &p_sta_pass_3, &p_sta_pass_4};
  for (uint8_t i = 0; i < 3; i++) {
    String s = ssidParams[i]->getValue();
    String p = passParams[i]->getValue();
    if (s.length() > 0 && p.length() >= 8) {
      JsonObject n = nets.add<JsonObject>();
      n["ssid"] = s;
      n["password"] = p;
    }
  }
  if (nets.size() == 0 && strcmp(p_mode.getValue(), "sta") == 0) {
    SLOG_PRINTLN("No STA credentials; keeping portal config in memory only.");
  }
  if (wifi_config_save(doc)) {
    SLOG_PRINTLN("wifi.json saved");
  } else {
    SLOG_PRINTLN("wifi.json save failed");
  }
  wifi_config_apply(doc);
}
