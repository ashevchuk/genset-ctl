#include "wifi_net.h"
#include "debug.h"
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/semphr.h>
#include <lwip/inet.h>
#include <lwip/ip4_addr.h>
#include <string.h>
#include <string>
#include <vector>
#include <algorithm>

#define WIFI_SCAN_MAX_RECORDS     32
#define WIFI_MULTI_SCAN_MS        300   // per channel, as AsyncWiFiMulti scanned
#define WIFI_SCAN_ACTIVE_MIN_MS   100   // Arduino's active-scan minimum dwell
#define WIFI_BLOCKING_SCAN_WAIT_MS 60000
#define WIFI_SCAN_DONE_BIT        BIT0
#define WIFI_GOT_IP_BIT           BIT1

enum ScanState { SCAN_IDLE, SCAN_RUNNING, SCAN_DONE, SCAN_FAILED };

struct WifiCandidate {
    std::string ssid;
    std::string pass;
    int8_t rssi;
    uint8_t channel;
};

static esp_netif_t *s_sta = nullptr;
static esp_netif_t *s_ap = nullptr;
static SemaphoreHandle_t s_lock = nullptr;
static EventGroupHandle_t s_events = nullptr;
static char s_hostname[33] = "";
static void (*s_got_ip_cb)(uint32_t) = nullptr;
static volatile bool s_has_ip = false;
static bool s_auto_reconnect = true;
static bool s_first_disconnect = true;  // Arduino retries the very first drop for any reason
static bool s_static_ip = false;
static bool s_started = false;  // esp_wifi_start() called and not stopped

static ScanState s_scan_state = SCAN_IDLE;
static wifi_ap_record_t s_scan_records[WIFI_SCAN_MAX_RECORDS];
static int s_scan_count = 0;

static WifiMultiStatus s_multi_status = WIFI_MULTI_IDLE;
static std::vector<WifiCandidate> s_configured;
static std::vector<WifiCandidate> s_candidates;
static size_t s_candidate_index = 0;
static bool s_multi_trying = false;  // a candidate connect is in flight

struct Lock {
    Lock() { xSemaphoreTakeRecursive(s_lock, portMAX_DELAY); }
    ~Lock() { xSemaphoreGiveRecursive(s_lock); }
};

// ---- helpers ------------------------------------------------------------------

static bool parse_ip(const char *s, esp_ip4_addr_t *out) {
    if (s == nullptr || *s == '\0') return false;
    ip4_addr_t a;
    if (!ip4addr_aton(s, &a)) return false;
    out->addr = a.addr;
    return true;
}

static bool is_reconnectable_reason(uint8_t reason) {
    switch (reason) {
        case WIFI_REASON_UNSPECIFIED:
        case WIFI_REASON_AUTH_EXPIRE:
        case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
        case WIFI_REASON_802_1X_AUTH_FAILED:
        case WIFI_REASON_HANDSHAKE_TIMEOUT:
        case WIFI_REASON_AUTH_LEAVE:
        case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:
        case WIFI_REASON_ASSOC_TOOMANY:
        case WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA:
        case WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA:
        case WIFI_REASON_ASSOC_NOT_AUTHED:
        case WIFI_REASON_MIC_FAILURE:
        case WIFI_REASON_IE_IN_4WAY_DIFFERS:
        case WIFI_REASON_INVALID_PMKID:
        case WIFI_REASON_BEACON_TIMEOUT:
        case WIFI_REASON_NO_AP_FOUND:
        case WIFI_REASON_ASSOC_FAIL:
        case WIFI_REASON_CONNECTION_FAIL:
        case WIFI_REASON_AP_TSF_RESET:
        case WIFI_REASON_ROAMING:
            return true;
        default:
            return false;
    }
}

// Same wifi_config_t Arduino's WiFi.begin(ssid, pass, channel) built.
static bool sta_connect(const char *ssid, const char *pass, uint8_t channel) {
    wifi_config_t conf = {};
    conf.sta.channel = channel;
    conf.sta.scan_method = WIFI_FAST_SCAN;
    conf.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    conf.sta.threshold.rssi = -127;
    conf.sta.pmf_cfg.capable = true;
    strlcpy((char *)conf.sta.ssid, ssid, sizeof(conf.sta.ssid));
    if (pass != nullptr && pass[0] != '\0') {
        conf.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        strlcpy((char *)conf.sta.password, pass, sizeof(conf.sta.password));
    }
    if (esp_wifi_set_config(WIFI_IF_STA, &conf) != ESP_OK) return false;
    if (!s_static_ip) esp_netif_dhcpc_start(s_sta);
    return esp_wifi_connect() == ESP_OK;
}

static void multi_try_next(void) {
    while (s_candidate_index < s_candidates.size()) {
        const WifiCandidate &c = s_candidates[s_candidate_index];
        if (sta_connect(c.ssid.c_str(), c.pass.c_str(), c.channel)) {
            s_multi_trying = true;
            return;
        }
        s_candidate_index++;
    }
    s_multi_trying = false;
    s_multi_status = WIFI_MULTI_IDLE;
}

// Strongest-first, one entry per SSID, configured networks only.
static void multi_on_scan_done(void) {
    std::vector<wifi_ap_record_t> found(s_scan_records, s_scan_records + s_scan_count);
    std::stable_sort(found.begin(), found.end(),
                     [](const wifi_ap_record_t &a, const wifi_ap_record_t &b) { return a.rssi > b.rssi; });
    s_candidates.clear();
    for (const wifi_ap_record_t &r : found) {
        std::string ssid((const char *)r.ssid);
        if (ssid.empty() || ssid.size() > 31) continue;
        bool seen = std::any_of(s_candidates.begin(), s_candidates.end(),
                                [&](const WifiCandidate &c) { return c.ssid == ssid; });
        if (seen) continue;
        for (const WifiCandidate &cfg : s_configured) {
            if (cfg.ssid == ssid) {
                s_candidates.push_back({ssid, cfg.pass, r.rssi, r.primary});
                break;
            }
        }
    }
    s_candidate_index = 0;
    multi_try_next();
}

// ---- events -----------------------------------------------------------------

static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    (void)base;
    Lock lock;
    switch (id) {
        case WIFI_EVENT_SCAN_DONE: {
            const wifi_event_sta_scan_done_t *ev = (const wifi_event_sta_scan_done_t *)data;
            uint16_t n = WIFI_SCAN_MAX_RECORDS;
            if (ev->status != 0 || esp_wifi_scan_get_ap_records(&n, s_scan_records) != ESP_OK) {
                esp_wifi_clear_ap_list();
                s_scan_count = 0;
                s_scan_state = SCAN_FAILED;
            } else {
                s_scan_count = n;
                s_scan_state = SCAN_DONE;
            }
            SLOG_PRINTLN("Completed scan for access points");
            xEventGroupSetBits(s_events, WIFI_SCAN_DONE_BIT);
            if (s_multi_status == WIFI_MULTI_RUNNING && !s_multi_trying) {
                if (s_scan_state == SCAN_DONE) {
                    multi_on_scan_done();
                } else {
                    s_multi_status = WIFI_MULTI_IDLE;
                }
            }
            break;
        }
        case WIFI_EVENT_STA_START:
            SLOG_PRINTLN("WiFi client started");
            break;
        case WIFI_EVENT_STA_CONNECTED:
            SLOG_PRINTLN("Connected to access point");
            // Defensive: modem sleep could miss the AP's buffered DHCP
            // offer/ack right after association.
            esp_wifi_set_ps(WIFI_PS_NONE);
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            const wifi_event_sta_disconnected_t *ev = (const wifi_event_sta_disconnected_t *)data;
            uint8_t reason = ev->reason ? ev->reason : (uint8_t)WIFI_REASON_UNSPECIFIED;
            SLOG_PRINTF("Disconnected from WiFi access point, reason=%u\n", reason);
            s_has_ip = false;
            xEventGroupClearBits(s_events, WIFI_GOT_IP_BIT);
            if (s_multi_status == WIFI_MULTI_CONNECTED) {
                s_multi_status = WIFI_MULTI_IDLE;
            } else if (s_multi_status == WIFI_MULTI_RUNNING && s_multi_trying) {
                s_candidate_index++;
                multi_try_next();
                break;
            }
            bool retry = false;
            if (reason == WIFI_REASON_ASSOC_LEAVE) {
                retry = false;  // voluntary disconnect
            } else if (s_first_disconnect) {
                s_first_disconnect = false;
                retry = true;
            } else if (s_auto_reconnect && is_reconnectable_reason(reason)) {
                retry = true;
            }
            if (retry && s_multi_status != WIFI_MULTI_RUNNING) esp_wifi_connect();
            break;
        }
        case WIFI_EVENT_AP_START:
            SLOG_PRINTLN("WiFi access point started");
            break;
        case WIFI_EVENT_AP_STOP:
            SLOG_PRINTLN("WiFi access point stopped");
            break;
        case WIFI_EVENT_AP_STACONNECTED:
            SLOG_PRINTLN("Client connected");
            break;
        case WIFI_EVENT_AP_STADISCONNECTED:
            SLOG_PRINTLN("Client disconnected");
            break;
        default:
            break;
    }
}

static void on_ip_event(void *arg, esp_event_base_t base, int32_t id, void *data) {
    (void)arg;
    (void)base;
    void (*cb)(uint32_t) = nullptr;
    uint32_t ip = 0;
    {
        Lock lock;
        if (id == IP_EVENT_STA_GOT_IP) {
            const ip_event_got_ip_t *ev = (const ip_event_got_ip_t *)data;
            ip = ev->ip_info.ip.addr;
            s_has_ip = true;
            xEventGroupSetBits(s_events, WIFI_GOT_IP_BIT);
            s_multi_status = WIFI_MULTI_CONNECTED;
            s_multi_trying = false;
            cb = s_got_ip_cb;
            char ipstr[16];
            esp_ip4addr_ntoa(&ev->ip_info.ip, ipstr, sizeof(ipstr));
            SLOG_PRINTF("Obtained IP address: %s\n", ipstr);
        } else if (id == IP_EVENT_STA_LOST_IP) {
            SLOG_PRINTLN("Lost IP address and IP address is reset to 0");
            s_has_ip = false;
            xEventGroupClearBits(s_events, WIFI_GOT_IP_BIT);
            if (s_multi_status == WIFI_MULTI_CONNECTED) {
                s_multi_status = WIFI_MULTI_IDLE;
            } else if (s_multi_status == WIFI_MULTI_RUNNING && s_multi_trying) {
                s_candidate_index++;
                multi_try_next();
            }
        }
    }
    if (cb) cb(ip);
}

// ---- public -----------------------------------------------------------------

void wifi_net_init(void) {
    if (s_lock != nullptr) return;
    s_lock = xSemaphoreCreateRecursiveMutex();
    s_events = xEventGroupCreate();
    esp_netif_init();
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) SLOG_PRINTF("event loop: %s\n", esp_err_to_name(err));
    s_sta = esp_netif_create_default_wifi_sta();
    s_ap = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    // Every config is set explicitly at boot; nothing needs to survive in NVS.
    esp_wifi_set_storage(WIFI_STORAGE_RAM);
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_ip_event, nullptr);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_LOST_IP, on_ip_event, nullptr);
    SLOG_PRINTLN("WiFi interface ready");
}

void wifi_net_set_hostname(const char *hostname) {
    strlcpy(s_hostname, hostname, sizeof(s_hostname));
    esp_netif_set_hostname(s_sta, s_hostname);
    esp_netif_set_hostname(s_ap, s_hostname);
}

const char *wifi_net_hostname(void) {
    return s_hostname;
}

void wifi_net_on_got_ip(void (*cb)(uint32_t ip)) {
    s_got_ip_cb = cb;
}

bool wifi_net_start_ap(const char *ssid, const char *pass, uint8_t channel, bool hidden, uint8_t max_conn,
                       const char *ip, const char *gw, const char *sn) {
    esp_wifi_set_mode(WIFI_MODE_AP);
    wifi_config_t conf = {};
    strlcpy((char *)conf.ap.ssid, ssid, sizeof(conf.ap.ssid));
    conf.ap.ssid_len = strlen((const char *)conf.ap.ssid);
    conf.ap.channel = channel;
    conf.ap.ssid_hidden = hidden ? 1 : 0;
    conf.ap.max_connection = max_conn;
    conf.ap.beacon_interval = 100;
    if (pass != nullptr && pass[0] != '\0') {
        strlcpy((char *)conf.ap.password, pass, sizeof(conf.ap.password));
        conf.ap.authmode = WIFI_AUTH_WPA2_PSK;
    } else {
        conf.ap.authmode = WIFI_AUTH_OPEN;
    }
    if (esp_wifi_set_config(WIFI_IF_AP, &conf) != ESP_OK) return false;
    if (esp_wifi_start() != ESP_OK) return false;
    s_started = true;

    esp_netif_ip_info_t info = {};
    if (parse_ip(ip, &info.ip) && parse_ip(gw, &info.gw) && parse_ip(sn, &info.netmask)) {
        esp_netif_dhcps_stop(s_ap);
        esp_netif_set_ip_info(s_ap, &info);
        esp_netif_dhcps_start(s_ap);
    }
    return true;
}

bool wifi_net_start_sta(void) {
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) return false;
    if (esp_wifi_start() != ESP_OK) return false;
    s_started = true;
    esp_wifi_set_ps(WIFI_PS_NONE);
    return true;
}

void wifi_net_radio_off(void) {
    esp_wifi_stop();
    s_started = false;
}

bool wifi_net_set_static_ip(const char *ip, const char *gw, const char *sn, const char *dns1, const char *dns2) {
    esp_netif_ip_info_t info = {};
    if (!parse_ip(ip, &info.ip) || !parse_ip(gw, &info.gw) || !parse_ip(sn, &info.netmask)) return false;
    esp_netif_dhcpc_stop(s_sta);
    if (esp_netif_set_ip_info(s_sta, &info) != ESP_OK) return false;
    esp_netif_dns_info_t dns = {};
    if (parse_ip(dns1, &dns.ip.u_addr.ip4)) {
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_dns_info(s_sta, ESP_NETIF_DNS_MAIN, &dns);
    }
    if (parse_ip(dns2, &dns.ip.u_addr.ip4)) {
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        esp_netif_set_dns_info(s_sta, ESP_NETIF_DNS_BACKUP, &dns);
    }
    s_static_ip = true;
    return true;
}

void wifi_net_set_auto_reconnect(bool on) {
    Lock lock;
    s_auto_reconnect = on;
}

void wifi_net_multi_clear(void) {
    Lock lock;
    s_configured.clear();
    s_candidates.clear();
    s_candidate_index = 0;
    s_multi_trying = false;
    s_multi_status = WIFI_MULTI_IDLE;
}

bool wifi_net_multi_add(const char *ssid, const char *pass) {
    Lock lock;
    std::string s = ssid ? ssid : "";
    std::string p = pass ? pass : "";
    if (s.empty() || s.size() > 31 || p.size() > 64) return false;
    for (const WifiCandidate &c : s_configured) {
        if (c.ssid == s) return false;
    }
    s_configured.push_back({s, p, 0, 0});
    return true;
}

bool wifi_net_multi_start(uint8_t channel) {
    {
        Lock lock;
        if (s_multi_status != WIFI_MULTI_IDLE) return false;
        s_multi_trying = false;
    }
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(10));
    esp_wifi_set_mode(WIFI_MODE_STA);
    {
        Lock lock;
        s_multi_status = WIFI_MULTI_RUNNING;
    }
    // A scan already in progress counts: its results are used when it ends.
    wifi_net_scan_start(channel, WIFI_MULTI_SCAN_MS, true);
    return true;
}

WifiMultiStatus wifi_net_multi_status(void) {
    Lock lock;
    return s_multi_status;
}

bool wifi_net_connect(const char *ssid, const char *pass) {
    Lock lock;
    return sta_connect(ssid, pass, 0);
}

bool wifi_net_wait_connected(uint32_t timeout_ms) {
    return (xEventGroupWaitBits(s_events, WIFI_GOT_IP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(timeout_ms)) &
            WIFI_GOT_IP_BIT) != 0;
}

bool wifi_net_connected(void) {
    return s_has_ip;
}

int8_t wifi_net_rssi(void) {
    wifi_ap_record_t info;
    return esp_wifi_sta_get_ap_info(&info) == ESP_OK ? info.rssi : 0;
}

bool wifi_net_ssid(char *out, size_t len) {
    wifi_ap_record_t info;
    if (esp_wifi_sta_get_ap_info(&info) != ESP_OK) {
        if (len) out[0] = '\0';
        return false;
    }
    strlcpy(out, (const char *)info.ssid, len);
    return true;
}

uint8_t wifi_net_channel(void) {
    uint8_t primary = 0;
    wifi_second_chan_t second = WIFI_SECOND_CHAN_NONE;
    esp_wifi_get_channel(&primary, &second);
    return primary;
}

uint32_t wifi_net_sta_ip(void) {
    esp_netif_ip_info_t info = {};
    if (!s_has_ip || esp_netif_get_ip_info(s_sta, &info) != ESP_OK) return 0;
    return info.ip.addr;
}

uint32_t wifi_net_ap_ip(void) {
    esp_netif_ip_info_t info = {};
    return esp_netif_get_ip_info(s_ap, &info) == ESP_OK ? info.ip.addr : 0;
}

void wifi_net_sta_mac_str(char *out, size_t len) {
    uint8_t mac[6] = {};
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(out, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

bool wifi_net_scan_start(uint8_t channel, uint32_t max_ms_per_chan, bool show_hidden) {
    Lock lock;
    if (s_scan_state == SCAN_RUNNING) return true;  // results of the running scan will do
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_wifi_get_mode(&mode);
    if (mode == WIFI_MODE_AP) {
        esp_wifi_set_mode(WIFI_MODE_APSTA);
    } else if (mode != WIFI_MODE_APSTA && mode != WIFI_MODE_STA) {
        esp_wifi_set_mode(WIFI_MODE_STA);
    }
    // A scan needs a running driver (e.g. the OLED menu scan with the radio off).
    if (!s_started && esp_wifi_start() == ESP_OK) s_started = true;
    esp_wifi_clear_ap_list();
    s_scan_count = 0;
    wifi_scan_config_t cfg = {};
    cfg.channel = channel;
    cfg.show_hidden = show_hidden;
    cfg.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    cfg.scan_time.active.min = WIFI_SCAN_ACTIVE_MIN_MS;
    cfg.scan_time.active.max = max_ms_per_chan;
    xEventGroupClearBits(s_events, WIFI_SCAN_DONE_BIT);
    if (esp_wifi_scan_start(&cfg, false) != ESP_OK) {
        s_scan_state = SCAN_FAILED;
        if (s_multi_status == WIFI_MULTI_RUNNING && !s_multi_trying) s_multi_status = WIFI_MULTI_IDLE;
        return false;
    }
    s_scan_state = SCAN_RUNNING;
    return true;
}

int wifi_net_scan_blocking(void) {
    if (!wifi_net_scan_start(0, WIFI_MULTI_SCAN_MS, false)) return WIFI_NET_SCAN_FAILED;
    xEventGroupWaitBits(s_events, WIFI_SCAN_DONE_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(WIFI_BLOCKING_SCAN_WAIT_MS));
    return wifi_net_scan_result();
}

int wifi_net_scan_result(void) {
    Lock lock;
    if (s_scan_state == SCAN_RUNNING) return WIFI_NET_SCAN_RUNNING;
    if (s_scan_state == SCAN_DONE) return s_scan_count;
    return WIFI_NET_SCAN_FAILED;
}

bool wifi_net_scan_ssid(int index, char *out, size_t len) {
    Lock lock;
    if (s_scan_state != SCAN_DONE || index < 0 || index >= s_scan_count) {
        if (len) out[0] = '\0';
        return false;
    }
    strlcpy(out, (const char *)s_scan_records[index].ssid, len);
    return true;
}

void wifi_net_scan_clear(void) {
    Lock lock;
    if (s_scan_state == SCAN_RUNNING) return;
    s_scan_count = 0;
    s_scan_state = SCAN_IDLE;
}
