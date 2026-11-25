/*
 * esp_now_link.cpp — ESP-NOW fallback link to the display panel(s).
 *
 * Protocol v2: one fleet key, broadcast discovery beacon, displays join by
 * sending a hello. See esp_now_link.h and ESPNOW.md for the design and the
 * wire format.
 */

#include "esp_now_link.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "debug.h"
#include "smoothed_safe.h"
#include "statemachine.h"
#include "wifi_net.h"
#include <nvs.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include "uptime.h"

extern SmoothedSafe<double> adc_1_0;
extern SmoothedSafe<double> adc_1_1;
extern SmoothedSafe<double> adc_1_2;
extern SmoothedSafe<double> adc_1_3;
extern SmoothedSafe<double> adc_2_0;
extern SmoothedSafe<double> adc_2_1;
extern SmoothedSafe<double> adc_2_2;
double fuel_level_from_adc(double adc_value);
extern char wifiHostname[33];

EspNowConfig_t g_espnow_config = {
    .enabled = false,
    .key     = {0},
};

// ---- wire format (ESPNOW.md is the source of truth — keep both in sync) --

#define ESPNOW_PROTO_VERSION   3
#define ESPNOW_MAGIC_STATE     0xE5
#define ESPNOW_MAGIC_CMD       0xEC
#define ESPNOW_MAGIC_BEACON    0xEB
#define ESPNOW_MAGIC_HELLO     0xE1
#define ESPNOW_MAGIC_JRN_REQ   0xE7
#define ESPNOW_MAGIC_JRN_EVENT 0xE8
#define ESPNOW_MAGIC_JRN_END   0xE9
#define ESPNOW_NAME_LEN        24
#define ESPNOW_MAX_PAYLOAD     250  // ESP-NOW v1 frame limit
#define ESPNOW_JRN_MAX_EVENTS  25

#define ESPNOW_FLAG_WORKING  0x01  // !is_not_working() — same as WS switch.run

static const uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// Controller -> broadcast, unencrypted, every ESPNOW_BEACON_MS. Only lets a
// display list this generator; nothing in it is trusted for control.
struct __attribute__((packed)) EspNowBeaconPkt {
    uint8_t magic;
    uint8_t proto_version;
    char    name[ESPNOW_NAME_LEN];  // wifi.json "hostname", NUL-padded
};

// Display -> broadcast, unencrypted. Asks the controller whose STA MAC is
// `target` to add the sender as an encrypted peer. Anyone can send one; it
// only costs a peer slot — without the fleet key the sender can neither
// decrypt state nor produce a command that survives decryption.
struct __attribute__((packed)) EspNowHelloPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t target[6];
};

// Controller -> each joined display, unicast, encrypted, 1 Hz. Values
// mirror ReportTask's WS "value" section (same smoothed sources, units).
struct __attribute__((packed)) EspNowStatePkt {
    uint8_t  magic;
    uint8_t  proto_version;
    uint8_t  state_id;      // `state` enum value (rtos_config.h)
    uint8_t  flags;         // ESPNOW_FLAG_*
    uint8_t  reg_inputs;    // raw shift_reg_in bitmask
    uint16_t reg_outputs;   // raw shift_reg_outputs bitmask
    uint32_t rpm;
    float    freq_hz;
    float    bat_v;         // WS ADC1_1
    float    fuel_l;        // WS ADC1_0
    float    core_v;        // WS ADC1_2
    float    head_temp_c;
    float    oil_temp_c;    // WS ADC0_0
    float    ambient_temp_c;    // WS ADC0_1
    float    inverter_temp_c;   // WS ADC0_2
    float    alternator_temp_c; // WS ADC0_3
    float    vrms;
    float    irms;
    float    active_power_w;
    uint32_t sta_ip;        // controller's STA IPv4 (network order), 0 if none
    uint8_t  maint_due_mask;  // WS MAINT_DUE_MASK
};

// Display -> controller, unicast, encrypted. Mirrors WebCommandType_t
// (rtos_config.h) — StateMachineTask's gating applies unchanged.
enum EspNowCmd_t : uint8_t {
    ESPNOW_CMD_START = 0,
    ESPNOW_CMD_START_RETRY,
    ESPNOW_CMD_STOP,
    ESPNOW_CMD_REG_SET,
    ESPNOW_CMD_REG_TOGGLE,
    ESPNOW_CMD_MAINT,       // record maintenance; `reg` = FLOG_MAINT_* subtype
};

struct __attribute__((packed)) EspNowCmdPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t cmd;    // EspNowCmd_t
    uint8_t reg;    // for REG_SET/REG_TOGGLE; subtype for MAINT
    uint8_t state;  // for REG_SET: desired state (0/1)
};

// Display -> controller, unicast, encrypted: send the newest `count` journal
// events. Answered with JrnEvent packets (one event each), then one JrnEnd.
struct __attribute__((packed)) EspNowJrnReqPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t req_id;  // echoed in the answer so a late one can be told apart
    uint8_t count;
};

// Controller -> display: one event as the same JSON object GET /log returns.
// Variable length; the JSON is not NUL-terminated.
struct __attribute__((packed)) EspNowJrnEventHdr {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t req_id;
    uint8_t idx;     // 0 = newest
};

// Controller -> display: closes an answer; also carries the lifetime metrics
// the panel's journal tab shows (GET /metrics equivalents).
struct __attribute__((packed)) EspNowJrnEndPkt {
    uint8_t  magic;
    uint8_t  proto_version;
    uint8_t  req_id;
    uint8_t  count;           // JrnEvent packets sent for this req_id
    uint32_t ts_now;          // /log ts_now
    uint32_t run_seconds;
    uint32_t charge_seconds;
    uint32_t starts;
    uint32_t start_fails;
    uint32_t emergencies;
    uint32_t charge_cycles;
};

// Sizes documented in ESPNOW.md; the panel's copies assert the same values.
static_assert(sizeof(EspNowBeaconPkt) == 26, "wire format");
static_assert(sizeof(EspNowHelloPkt) == 8, "wire format");
static_assert(sizeof(EspNowStatePkt) == 64, "wire format");
static_assert(sizeof(EspNowCmdPkt) == 5, "wire format");
static_assert(sizeof(EspNowJrnReqPkt) == 4, "wire format");
static_assert(sizeof(EspNowJrnEventHdr) == 4, "wire format");
static_assert(sizeof(EspNowJrnEndPkt) == 32, "wire format");

// ---- config -----------------------------------------------------------

// 32 hex chars -> 16 bytes. Fails closed on any malformed input.
static bool parse_key_hex(const char *s, uint8_t key[ESPNOW_KEY_LEN]) {
    if (s == NULL || strlen(s) != ESPNOW_KEY_LEN * 2) return false;
    for (int i = 0; i < ESPNOW_KEY_LEN; i++) {
        unsigned b;
        if (sscanf(s + i * 2, "%2x", &b) != 1) return false;
        key[i] = (uint8_t)b;
    }
    return true;
}

void espnow_config_load(const JsonDocument &doc) {
    JsonObjectConst espnow = doc["espnow"];
    if (espnow.isNull()) return;

    bool enabled = espnow["enabled"] | false;
    uint8_t key[ESPNOW_KEY_LEN];
    if (!enabled || !parse_key_hex(espnow["key"] | "", key)) {
        // A missing/malformed key is a hard disable — never fall back to an
        // all-zero key nobody chose.
        if (enabled) SLOG_PRINTLN("ESP-NOW: \"espnow.key\" missing or malformed — disabled");
        g_espnow_config.enabled = false;
        return;
    }
    g_espnow_config.enabled = true;
    memcpy(g_espnow_config.key, key, ESPNOW_KEY_LEN);
}

// ---- joined displays --------------------------------------------------

// ESP_NOW_MAX_ENCRYPT_PEER_NUM is 6; leave one slot spare.
#define ESPNOW_MAX_DISPLAYS     5
#define ESPNOW_PEER_TIMEOUT_MS  120000  // displays re-hello every 30 s
#define ESPNOW_BEACON_MS        2000
#define ESPNOW_STATE_MS         1000

struct DisplayPeer {
    bool     used;
    uint8_t  mac[6];
    uint32_t last_hello_ms;
};

static DisplayPeer   s_peers[ESPNOW_MAX_DISPLAYS];
static QueueHandle_t s_hello_q = NULL;   // 6-byte MACs from the recv callback

// Requests that need FRAM (journal, maintenance record) are handed from the
// recv callback, which runs in the WiFi driver task, to EspNowTask.
enum : uint8_t { REQ_JOURNAL, REQ_MAINT };
struct EspNowReq {
    uint8_t mac[6];
    uint8_t kind;
    uint8_t req_id;
    uint8_t arg;  // REQ_JOURNAL: event count; REQ_MAINT: subtype
};
static QueueHandle_t s_req_q = NULL;
static uint8_t       s_own_mac[6];

static bool add_encrypted_peer(const uint8_t mac[6]) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    memcpy(peer.lmk, g_espnow_config.key, ESPNOW_KEY_LEN);
    peer.channel = 0;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = true;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        SLOG_PRINTF("ESP-NOW: add_peer failed: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

// Join or refresh a display. A full table evicts the least recently heard.
static void peer_touch(const uint8_t mac[6]) {
    int free_slot = -1, oldest = 0;
    for (int i = 0; i < ESPNOW_MAX_DISPLAYS; i++) {
        if (s_peers[i].used && memcmp(s_peers[i].mac, mac, 6) == 0) {
            s_peers[i].last_hello_ms = uptime_ms();
            return;
        }
        if (!s_peers[i].used && free_slot < 0) free_slot = i;
        if (s_peers[i].used && s_peers[i].last_hello_ms < s_peers[oldest].last_hello_ms) oldest = i;
    }
    int slot = free_slot;
    if (slot < 0) {
        slot = oldest;
        esp_now_del_peer(s_peers[slot].mac);
        s_peers[slot].used = false;
    }
    if (!add_encrypted_peer(mac)) return;
    s_peers[slot].used = true;
    memcpy(s_peers[slot].mac, mac, 6);
    s_peers[slot].last_hello_ms = uptime_ms();
    SLOG_PRINTF("ESP-NOW: display %02x:%02x:%02x:%02x:%02x:%02x joined\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void peers_expire(void) {
    for (int i = 0; i < ESPNOW_MAX_DISPLAYS; i++) {
        if (s_peers[i].used && uptime_ms() - s_peers[i].last_hello_ms > ESPNOW_PEER_TIMEOUT_MS) {
            esp_now_del_peer(s_peers[i].mac);
            s_peers[i].used = false;
            SLOG_PRINTLN("ESP-NOW: display peer timed out");
        }
    }
}

// ---- receive callback ---------------------------------------------------

// Runs in the WiFi driver's own task context (not an ISR), so plain
// xQueueSend() is safe. Commands are accepted only from MACs in the peer
// list, all of which are encrypted peers; a frame that fails decryption
// never reaches this callback. See ESPNOW.md for what was verified about
// unencrypted frames spoofing a peer's address.
static void espnow_on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!g_espnow_config.enabled || len < 2 || data[1] != ESPNOW_PROTO_VERSION) return;
    const uint8_t magic = data[0];

    if (magic == ESPNOW_MAGIC_HELLO && len == sizeof(EspNowHelloPkt)) {
        EspNowHelloPkt h;
        memcpy(&h, data, sizeof(h));
        if (memcmp(h.target, s_own_mac, 6) == 0 && s_hello_q != NULL)
            xQueueSend(s_hello_q, info->src_addr, 0);
        return;
    }

    if (memcmp(info->src_addr, BROADCAST_MAC, 6) == 0) return;
    if (!esp_now_is_peer_exist(info->src_addr)) return;

    if (magic == ESPNOW_MAGIC_JRN_REQ && len == sizeof(EspNowJrnReqPkt)) {
        EspNowJrnReqPkt r;
        memcpy(&r, data, sizeof(r));
        EspNowReq req = {};
        memcpy(req.mac, info->src_addr, 6);
        req.kind = REQ_JOURNAL;
        req.req_id = r.req_id;
        req.arg = r.count;
        if (s_req_q != NULL) xQueueSend(s_req_q, &req, 0);
        return;
    }

    if (magic != ESPNOW_MAGIC_CMD || len != sizeof(EspNowCmdPkt) || xWebCommandQueue == NULL) return;
    EspNowCmdPkt pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.cmd == ESPNOW_CMD_MAINT) {
        EspNowReq req = {};
        memcpy(req.mac, info->src_addr, 6);
        req.kind = REQ_MAINT;
        req.arg = pkt.reg;
        if (s_req_q != NULL) xQueueSend(s_req_q, &req, 0);
        return;
    }

    WebCommand_t web_cmd = {};
    web_cmd.source = FLOG_SRC_ESPNOW;

    switch (pkt.cmd) {
        case ESPNOW_CMD_START:       web_cmd.type = WEB_CMD_START; break;
        case ESPNOW_CMD_START_RETRY: web_cmd.type = WEB_CMD_START_RETRY; break;
        case ESPNOW_CMD_STOP:        web_cmd.type = WEB_CMD_STOP; break;
        case ESPNOW_CMD_REG_SET:
            web_cmd.type  = WEB_CMD_REG_SET;
            web_cmd.reg   = pkt.reg;
            web_cmd.state = pkt.state != 0;
            break;
        case ESPNOW_CMD_REG_TOGGLE:
            web_cmd.type = WEB_CMD_REG_TOGGLE;
            web_cmd.reg  = pkt.reg;
            break;
        default:
            return;
    }
    xQueueSend(xWebCommandQueue, &web_cmd, 0);
}

// ---- channel handling while the STA has no AP (ESPNOW.md) --------------
//
// Any STA connect attempt with the AP absent sweeps every channel, even with
// a channel hint (the hint only sets where the sweep starts), so the radio
// leaves the shared channel and ESP-NOW goes deaf. Instead: remember the
// channel/SSID of the last AP association ("home"), park the radio there
// while disconnected, and look for the AP with a single-channel scan of that
// channel only. Full all-channel rescans still happen on the caller's own
// backoff, in case the AP moved.

#define ESPNOW_PROBE_INTERVAL_MS  5000
#define ESPNOW_PROBE_MS_PER_CHAN  120

static uint8_t  s_home_channel = 0;
static char     s_home_ssid[33] = "";
static bool     s_home_loaded = false;
static bool     s_probe_running = false;
static uint32_t s_last_probe_ms = 0;

// NVS lives in SPI flash: same flash/I2C collision hazard as WebTask's FFat
// reads (rtos_config.h, xFlashI2CGuard), so take the same guard.
static void home_load(void) {
    bool held = xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE;
    nvs_handle_t nvs;
    if (nvs_open("espnow", NVS_READONLY, &nvs) == ESP_OK) {
        uint8_t ch = 0;
        nvs_get_u8(nvs, "ch", &ch);  // a missing key leaves the default
        s_home_channel = ch;
        size_t len = sizeof(s_home_ssid);
        if (nvs_get_str(nvs, "ssid", s_home_ssid, &len) != ESP_OK) s_home_ssid[0] = '\0';
        nvs_close(nvs);
    }
    if (held) xSemaphoreGive(xFlashI2CGuard);
    s_home_loaded = true;
    SLOG_PRINTF("ESP-NOW: home channel %u, ssid \"%s\"\n", s_home_channel, s_home_ssid);
}

static void home_save(uint8_t ch, const char *ssid) {
    s_home_channel = ch;
    strlcpy(s_home_ssid, ssid, sizeof(s_home_ssid));
    bool held = xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE;
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("espnow", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_u8(nvs, "ch", ch);
        if (err == ESP_OK) err = nvs_set_str(nvs, "ssid", s_home_ssid);
        if (err == ESP_OK) err = nvs_commit(nvs);
        nvs_close(nvs);
    }
    if (err != ESP_OK) SLOG_PRINTF("ESP-NOW: home channel save failed: %s\n", esp_err_to_name(err));
    if (held) xSemaphoreGive(xFlashI2CGuard);
    SLOG_PRINTF("ESP-NOW: home channel saved: %u, ssid \"%s\"\n", ch, s_home_ssid);
}

bool espnow_wifi_supervise(bool connected, bool reconnect_idle) {
    if (!g_espnow_config.enabled) return false;
    if (!s_home_loaded) home_load();

    if (connected) {
        uint8_t ch = wifi_net_channel();
        char ssid[33];
        // The associated SSID is already empty in the loop pass that sees the drop.
        wifi_net_ssid(ssid, sizeof(ssid));
        if (ch != 0 && ssid[0] != '\0' &&
            (ch != s_home_channel || strcmp(ssid, s_home_ssid) != 0))
            home_save(ch, ssid);
        return false;
    }
    if (s_home_channel == 0 || s_home_ssid[0] == '\0') return false;

    if (s_probe_running) {
        int n = wifi_net_scan_result();
        if (n == WIFI_NET_SCAN_RUNNING) return false;
        s_probe_running = false;
        bool seen = false;
        char found[33];
        for (int i = 0; i < n; i++) {
            if (wifi_net_scan_ssid(i, found, sizeof(found)) && strcmp(found, s_home_ssid) == 0) seen = true;
        }
        wifi_net_scan_clear();
        if (seen) {
            SLOG_PRINTF("ESP-NOW: \"%s\" back on channel %u — reconnecting\n", s_home_ssid, s_home_channel);
            return true;
        }
    }

    if (!reconnect_idle || wifi_net_scan_result() == WIFI_NET_SCAN_RUNNING) return false;

    uint8_t primary = 0;
    wifi_second_chan_t second;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK && primary != s_home_channel)
        esp_wifi_set_channel(s_home_channel, WIFI_SECOND_CHAN_NONE);

    if (uptime_ms() - s_last_probe_ms >= ESPNOW_PROBE_INTERVAL_MS) {
        s_last_probe_ms = uptime_ms();
        if (wifi_net_scan_start(s_home_channel, ESPNOW_PROBE_MS_PER_CHAN, false))
            s_probe_running = true;
    }
    return false;
}

bool espnow_wifi_probe_running(void) {
    return s_probe_running;
}

uint8_t espnow_home_channel(void) {
    return (g_espnow_config.enabled && s_home_ssid[0] != '\0') ? s_home_channel : 0;
}

// ----------------------------------------------------------------------------
// EspNowTask: init, beacon, join handling, periodic state push. Commands
// arrive through the receive callback above.
// ----------------------------------------------------------------------------
static void idle_forever(void) {
    for (;;) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

static void fill_state(EspNowStatePkt &pkt) {
    state cur_state;
    SensorData_t sensors;
    PowerData_t power;
    uint8_t inputs;
    uint16_t outputs;
    portENTER_CRITICAL(&g_state_spinlock);
    cur_state = g_state.current_state;
    sensors   = g_state.sensors;
    power     = g_state.power;
    inputs    = g_state.shift_reg_inputs;
    outputs   = g_state.shift_reg_outputs;
    portEXIT_CRITICAL(&g_state_spinlock);

    pkt = {};
    pkt.magic         = ESPNOW_MAGIC_STATE;
    pkt.proto_version = ESPNOW_PROTO_VERSION;
    pkt.state_id      = (uint8_t)cur_state;
    pkt.flags         = is_not_working() ? 0 : ESPNOW_FLAG_WORKING;
    pkt.reg_inputs    = inputs;
    pkt.reg_outputs   = outputs;
    pkt.rpm           = sensors.rpm;
    pkt.freq_hz       = power.frequency;
    pkt.bat_v         = adc_2_1.get();
    pkt.fuel_l        = fuel_level_from_adc(adc_2_0.get());
    pkt.core_v        = adc_2_2.get() / 1000;
    pkt.head_temp_c   = sensors.head_temp;
    pkt.oil_temp_c        = adc_1_0.get();
    pkt.ambient_temp_c    = adc_1_1.get();
    pkt.inverter_temp_c   = adc_1_2.get();
    pkt.alternator_temp_c = adc_1_3.get();
    pkt.vrms          = power.rms_voltage;
    pkt.irms          = power.rms_current;
    pkt.active_power_w = power.active_power;
    pkt.sta_ip        = wifi_net_sta_ip();
    FramMetrics metrics = fram_metrics_get();
    pkt.maint_due_mask = fram_maint_due_mask(&metrics, g_limits_config.maint_interval_hours);
}

// The ESP-NOW send queue is short; a burst of journal packets can hit NO_MEM.
static esp_err_t send_paced(const uint8_t *mac, const uint8_t *data, size_t len) {
    esp_err_t err = ESP_FAIL;
    for (int attempt = 0; attempt < 10; attempt++) {
        err = esp_now_send(mac, data, len);
        if (err != ESP_ERR_ESPNOW_NO_MEM) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return err;
}

// Static: EspNowTask is the only caller, and its stack is small.
static FramLogRecord s_jrn_batch[FLOG_READ_BATCH_MAX];
static bool          s_jrn_ok[FLOG_READ_BATCH_MAX];
static uint8_t       s_jrn_pkt[ESPNOW_MAX_PAYLOAD];
static char          s_jrn_json[256];

static void handle_journal_request(const EspNowReq &req) {
    int want = req.arg < ESPNOW_JRN_MAX_EVENTS ? req.arg : ESPNOW_JRN_MAX_EVENTS;
    EspNowJrnEventHdr hdr = {ESPNOW_MAGIC_JRN_EVENT, ESPNOW_PROTO_VERSION, req.req_id, 0};
    const size_t max_json = sizeof(s_jrn_pkt) - sizeof(hdr);
    uint8_t sent = 0;
    uint32_t oldest = fram_log_oldest_seq();
    uint32_t seq = fram_log_newest_seq();
    while (seq != 0 && seq >= oldest && sent < want) {
        int n = fram_log_read_batch(seq, FLOG_READ_BATCH_MAX, s_jrn_batch, s_jrn_ok);
        if (n == 0) break;
        for (int i = 0; i < n && sent < want; i++) {
            if (!s_jrn_ok[i]) continue;
            int jl = fram_log_record_json(&s_jrn_batch[i], s_jrn_json, sizeof(s_jrn_json));
            if (jl <= 0 || (size_t)jl > max_json) continue;
            hdr.idx = sent;
            memcpy(s_jrn_pkt, &hdr, sizeof(hdr));
            memcpy(s_jrn_pkt + sizeof(hdr), s_jrn_json, jl);
            if (send_paced(req.mac, s_jrn_pkt, sizeof(hdr) + jl) == ESP_OK) sent++;
        }
        seq = (seq > (uint32_t)n) ? seq - n : 0;
        esp_task_wdt_reset();
    }

    FramMetrics m = fram_metrics_get();
    EspNowJrnEndPkt end = {};
    end.magic          = ESPNOW_MAGIC_JRN_END;
    end.proto_version  = ESPNOW_PROTO_VERSION;
    end.req_id         = req.req_id;
    end.count          = sent;
    end.ts_now         = (uint32_t)time(NULL);
    end.run_seconds    = m.total_run_seconds;
    end.charge_seconds = m.total_charge_seconds;
    end.starts         = m.start_count;
    end.start_fails    = m.start_fail_count;
    end.emergencies    = m.emergency_count;
    end.charge_cycles  = m.charge_cycles;
    send_paced(req.mac, (const uint8_t *)&end, sizeof(end));
}

static void handle_request(const EspNowReq &req) {
    if (req.kind == REQ_JOURNAL) {
        handle_journal_request(req);
    } else if (req.kind == REQ_MAINT) {
        // Same note the panel's HTTP path sends: recorded from its touchscreen.
        if (!fram_log_maintenance(req.arg, "touch"))
            SLOG_PRINTF("ESP-NOW: maintenance subtype %u rejected\n", req.arg);
    }
}

void EspNowTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);

    if (!g_espnow_config.enabled) {
        SLOG_PRINTLN("EspNowTask: disabled");
        idle_forever();
    }

    // Rides on the WiFi driver already started for the STA connection; all
    // peers use channel 0 = whatever the interface is on (see
    // espnow_wifi_supervise() for keeping that channel stable).
    esp_wifi_get_mac(WIFI_IF_STA, s_own_mac);
    char mac_str[18];
    wifi_net_sta_mac_str(mac_str, sizeof(mac_str));
    SLOG_PRINTF("ESP-NOW: own MAC=%s, name=\"%s\"\n", mac_str, wifiHostname);

    s_hello_q = xQueueCreate(8, 6);
    s_req_q = xQueueCreate(4, sizeof(EspNowReq));
    esp_err_t err = esp_now_init();
    if (s_hello_q == NULL || s_req_q == NULL || err != ESP_OK) {
        SLOG_PRINTF("EspNowTask: init failed: %s\n", esp_err_to_name(err));
        idle_forever();
    }
    esp_now_set_pmk(g_espnow_config.key);

    esp_now_peer_info_t bcast = {};
    memcpy(bcast.peer_addr, BROADCAST_MAC, 6);
    bcast.channel = 0;
    bcast.ifidx   = WIFI_IF_STA;
    bcast.encrypt = false;
    err = esp_now_add_peer(&bcast);
    if (err != ESP_OK) SLOG_PRINTF("EspNowTask: broadcast peer failed: %s\n", esp_err_to_name(err));

    esp_now_register_recv_cb(espnow_on_recv);

    EspNowBeaconPkt beacon = {};
    beacon.magic         = ESPNOW_MAGIC_BEACON;
    beacon.proto_version = ESPNOW_PROTO_VERSION;
    strncpy(beacon.name, wifiHostname, sizeof(beacon.name));

    uint32_t last_beacon_ms = 0, last_state_ms = 0;

    for (;;) {
        uint8_t mac[6];
        while (xQueueReceive(s_hello_q, mac, 0) == pdTRUE) peer_touch(mac);
        EspNowReq req;
        if (xQueueReceive(s_req_q, &req, 0) == pdTRUE) handle_request(req);

        uint32_t now = uptime_ms();
        if (now - last_beacon_ms >= ESPNOW_BEACON_MS) {
            last_beacon_ms = now;
            esp_now_send(BROADCAST_MAC, (const uint8_t *)&beacon, sizeof(beacon));
        }
        if (now - last_state_ms >= ESPNOW_STATE_MS) {
            last_state_ms = now;
            peers_expire();
            EspNowStatePkt pkt;
            fill_state(pkt);
            for (int i = 0; i < ESPNOW_MAX_DISPLAYS; i++) {
                if (s_peers[i].used)
                    esp_now_send(s_peers[i].mac, (const uint8_t *)&pkt, sizeof(pkt));
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
