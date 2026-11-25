/*
 * wifi_net.h — the WiFi radio on the native esp_wifi/esp_netif API.
 *
 * Sole owner of the driver. Reproduces what the code previously got from
 * Arduino's WiFi class plus AsyncWiFiMulti: STA multi-network selection
 * (scan, strongest configured SSID first, next one on failure), Arduino's
 * STA auto-reconnect rules, AP mode, one shared async scan, and the status
 * queries the rest of the firmware reads. See
 * .notes/wifi-migration-plan.md for the exact behaviour being matched.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

enum WifiMultiStatus { WIFI_MULTI_IDLE, WIFI_MULTI_RUNNING, WIFI_MULTI_CONNECTED };

// Netifs, driver, event handlers. Radio stays off until a mode is started.
void wifi_net_init(void);

void wifi_net_set_hostname(const char *hostname);
const char *wifi_net_hostname(void);

// Called (from the event task) on every STA got-IP event, with the IPv4
// address in network byte order.
void wifi_net_on_got_ip(void (*cb)(uint32_t ip));

// AP mode. `pass` NULL or "" = open network. ip/gw/sn as dotted strings.
bool wifi_net_start_ap(const char *ssid, const char *pass, uint8_t channel, bool hidden, uint8_t max_conn,
                       const char *ip, const char *gw, const char *sn);

// STA mode, power save off.
bool wifi_net_start_sta(void);

// Radio off (invalid wifi.json). A later scan turns it back on in STA mode.
void wifi_net_radio_off(void);

// Static STA addressing (call before connecting). dns1/dns2 may be "".
bool wifi_net_set_static_ip(const char *ip, const char *gw, const char *sn, const char *dns1, const char *dns2);

// Arduino-style reconnect of the current network after a drop.
void wifi_net_set_auto_reconnect(bool on);

// Multi-network connection (was AsyncWiFiMulti).
void wifi_net_multi_clear(void);
bool wifi_net_multi_add(const char *ssid, const char *pass);
// Scans `channel` (0 = all) and connects to the strongest configured SSID.
// False if already running or connected.
bool wifi_net_multi_start(uint8_t channel = 0);
WifiMultiStatus wifi_net_multi_status(void);

// Single STA connection without a scan (update mode).
bool wifi_net_connect(const char *ssid, const char *pass);
bool wifi_net_wait_connected(uint32_t timeout_ms);

// Status. "Connected" means the STA has an IP, as WiFi.status() == WL_CONNECTED did.
bool wifi_net_connected(void);
int8_t wifi_net_rssi(void);                    // 0 when not associated
bool wifi_net_ssid(char *out, size_t len);     // associated AP's SSID; "" when not associated
uint8_t wifi_net_channel(void);                // primary channel (the scan channel mid-scan)
uint32_t wifi_net_sta_ip(void);                // network byte order, 0 when none
uint32_t wifi_net_ap_ip(void);
void wifi_net_sta_mac_str(char *out, size_t len);  // "AA:BB:CC:DD:EE:FF"

// One shared scan at a time.
#define WIFI_NET_SCAN_RUNNING (-1)
#define WIFI_NET_SCAN_FAILED  (-2)
bool wifi_net_scan_start(uint8_t channel, uint32_t max_ms_per_chan, bool show_hidden);
int wifi_net_scan_blocking(void);              // all channels, 300 ms each, no hidden
int wifi_net_scan_result(void);                // count, or RUNNING / FAILED (none)
bool wifi_net_scan_ssid(int index, char *out, size_t len);
void wifi_net_scan_clear(void);
