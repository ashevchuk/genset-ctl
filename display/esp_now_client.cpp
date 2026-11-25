#include "esp_now_client.h"
#include "config.h"
#include "ws_client.h"
#include <WiFi.h>
#include <Preferences.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---- wire format: must match ../esp_now_link.cpp and ../ESPNOW.md -------

#define ESPNOW_PROTO_VERSION   3
#define ESPNOW_MAGIC_STATE     0xE5
#define ESPNOW_MAGIC_CMD       0xEC
#define ESPNOW_MAGIC_BEACON    0xEB
#define ESPNOW_MAGIC_HELLO     0xE1
#define ESPNOW_MAGIC_JRN_REQ   0xE7
#define ESPNOW_MAGIC_JRN_EVENT 0xE8
#define ESPNOW_MAGIC_JRN_END   0xE9
#define ESPNOW_FLAG_WORKING    0x01
#define ESPNOW_KEY_LEN         16
#define ESPNOW_MAX_PAYLOAD     250

static const uint8_t BROADCAST_MAC[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

struct __attribute__((packed)) EspNowBeaconPkt {
    uint8_t magic;
    uint8_t proto_version;
    char    name[ESPNOW_NAME_LEN];
};

struct __attribute__((packed)) EspNowHelloPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t target[6];
};

struct __attribute__((packed)) EspNowStatePkt {
    uint8_t  magic;
    uint8_t  proto_version;
    uint8_t  state_id;
    uint8_t  flags;
    uint8_t  reg_inputs;
    uint16_t reg_outputs;
    uint32_t rpm;
    float    freq_hz;
    float    bat_v;
    float    fuel_l;
    float    core_v;
    float    head_temp_c;
    float    oil_temp_c;
    float    ambient_temp_c;
    float    inverter_temp_c;
    float    alternator_temp_c;
    float    vrms;
    float    irms;
    float    active_power_w;
    uint32_t sta_ip;
    uint8_t  maint_due_mask;
};

enum : uint8_t {
    ESPNOW_CMD_START = 0,
    ESPNOW_CMD_START_RETRY,
    ESPNOW_CMD_STOP,
    ESPNOW_CMD_REG_SET,
    ESPNOW_CMD_REG_TOGGLE,
    ESPNOW_CMD_MAINT,
};

struct __attribute__((packed)) EspNowCmdPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t cmd;
    uint8_t reg;
    uint8_t state;
};

struct __attribute__((packed)) EspNowJrnReqPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t req_id;
    uint8_t count;
};

struct __attribute__((packed)) EspNowJrnEventHdr {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t req_id;
    uint8_t idx;
};

struct __attribute__((packed)) EspNowJrnEndPkt {
    uint8_t  magic;
    uint8_t  proto_version;
    uint8_t  req_id;
    uint8_t  count;
    uint32_t ts_now;
    uint32_t run_seconds;
    uint32_t charge_seconds;
    uint32_t starts;
    uint32_t start_fails;
    uint32_t emergencies;
    uint32_t charge_cycles;
};

static_assert(sizeof(EspNowBeaconPkt) == 26, "wire format");
static_assert(sizeof(EspNowHelloPkt) == 8, "wire format");
static_assert(sizeof(EspNowStatePkt) == 64, "wire format");
static_assert(sizeof(EspNowCmdPkt) == 5, "wire format");
static_assert(sizeof(EspNowJrnReqPkt) == 4, "wire format");
static_assert(sizeof(EspNowJrnEventHdr) == 4, "wire format");
static_assert(sizeof(EspNowJrnEndPkt) == 32, "wire format");
static_assert(ESPNOW_JRN_MAX_EVENTS <= 32, "s_jrn_have is a 32-bit mask");

// Controller's state_name() strings indexed by its `state` enum value
// (rtos_config.h) — ui.cpp's gen_state_label() matches on these strings.
static const char *const STATE_NAMES[] = {
    "init", "idle", "stopping", "stopping", "stopping_retry", "stopped",
    "stopped_retry", "starting", "starting", "starting_retry",
    "starting_retry", "started", "running", "error",
};

// Controller shift-register bits not already named in ws_client.h.
#define REG_IGNITION_BIT    10
#define REG_IN_OVERLOAD_BIT 0
#define REG_IN_OIL_BIT      1
#define REG_IN_OUTPUT_BIT   2
#define REG_IN_OVERHEAT_BIT 3
#define REG_IN_PSU_BIT      4
#define REG_IN_GEN_BIT      5
#define REG_IN_BAT_BIT      6

// Same window as display.ino's GEN_DATA_STALE_MS: the controller pushes at 1 Hz.
#define ESPNOW_ACTIVE_WINDOW_MS 6000
#define ESPNOW_SEEN_MAX         8
#define ESPNOW_SEEN_TIMEOUT_MS  10000   // beacons arrive every 2 s
#define ESPNOW_HELLO_JOIN_MS    2000    // while not receiving state
#define ESPNOW_HELLO_KEEP_MS    30000   // keepalive; controller expires peers after 120 s

static bool     s_enabled = false;
static bool     s_gen_selected = false;
static uint8_t  s_gen_mac[6];
static uint8_t  s_key[ESPNOW_KEY_LEN];
static volatile uint32_t s_last_rx_ms = 0;
static volatile uint32_t s_rx_count   = 0;
static volatile uint32_t s_rx_applied = 0;
static volatile uint32_t s_tx_ok      = 0;
static volatile uint32_t s_tx_fail    = 0;
static volatile int      s_last_rssi  = 0;
static volatile uint16_t s_last_outputs = 0;  // raw controller output mask, diagnostics only
static volatile uint32_t s_gen_ip = 0;
static volatile bool     s_gen_ip_changed = false;
static uint32_t          s_last_hello_ms = 0;

// Join-pin state (../ESPNOW.md "Channel handling"): right after a fresh
// selection, hold the radio on the channel its beacon was last heard on
// instead of letting espnow_wifi_supervise()'s own home-AP channel park
// drag it away — that logic exists to get BACK to this panel's usual AP,
// which is the wrong channel entirely when selecting a different
// generator on a site with no AP at all. Bounded by a timeout so a
// generator that never answers can't block normal WiFi reconnection
// forever.
#define ESPNOW_PIN_TIMEOUT_MS 20000
static uint8_t  s_pin_channel = 0;  // 0 = not pinning
static uint32_t s_pin_since   = 0;

struct SeenGen {
    bool     used;
    uint8_t  mac[6];
    char     name[ESPNOW_NAME_LEN + 1];
    int      rssi;
    uint32_t last_ms;
    uint8_t  channel;  // radio channel this beacon was actually heard on
};
static SeenGen     s_seen[ESPNOW_SEEN_MAX];
static portMUX_TYPE s_seen_mux = portMUX_INITIALIZER_UNLOCKED;

// Selection requested from the LVGL task, applied in espnow_client_poll().
static portMUX_TYPE s_sel_mux = portMUX_INITIALIZER_UNLOCKED;
static bool    s_sel_pending = false;
static bool    s_sel_none = false;
static uint8_t s_sel_mac[6];
static char    s_sel_name[ESPNOW_NAME_LEN + 1];

static bool parse_mac(const char *s, uint8_t mac[6]) {
    unsigned b[6];
    if (sscanf(s, "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) != 6)
        return false;
    for (int i = 0; i < 6; i++) mac[i] = (uint8_t)b[i];
    return true;
}

static bool parse_key_hex(const char *s, uint8_t key[ESPNOW_KEY_LEN]) {
    if (strlen(s) != ESPNOW_KEY_LEN * 2) return false;
    for (int i = 0; i < ESPNOW_KEY_LEN; i++) {
        unsigned b;
        if (sscanf(s + i * 2, "%2x", &b) != 1) return false;
        key[i] = (uint8_t)b;
    }
    return true;
}

static void format_mac(const uint8_t mac[6], char *out, size_t len) {
    snprintf(out, len, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static inline bool reg_bit(uint32_t v, int n) { return (v >> n) & 1; }

static void on_beacon(const uint8_t *src, const EspNowBeaconPkt &b, int rssi) {
    portENTER_CRITICAL(&s_seen_mux);
    int slot = -1, oldest = 0;
    for (int i = 0; i < ESPNOW_SEEN_MAX; i++) {
        if (s_seen[i].used && memcmp(s_seen[i].mac, src, 6) == 0) { slot = i; break; }
        if (!s_seen[i].used && slot < 0) slot = i;
        if (s_seen[i].last_ms < s_seen[oldest].last_ms) oldest = i;
    }
    if (slot < 0) slot = oldest;
    SeenGen &g = s_seen[slot];
    g.used = true;
    memcpy(g.mac, src, 6);
    memcpy(g.name, b.name, ESPNOW_NAME_LEN);
    g.name[ESPNOW_NAME_LEN] = '\0';
    // Unauthenticated over-the-air text: keep it printable and free of the
    // characters that would break the status JSON or LVGL's '\n'-separated
    // dropdown options.
    for (char *c = g.name; *c; c++)
        if (*c < 0x20 || *c > 0x7e || *c == '"' || *c == '\\') *c = '?';
    g.rssi = rssi;
    g.last_ms = millis();
    uint8_t primary = 0;
    wifi_second_chan_t second;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK) g.channel = primary;
    portEXIT_CRITICAL(&s_seen_mux);
}

// Channel this generator's beacon was last heard on, or 0 if never heard.
static uint8_t seen_channel_for(const uint8_t mac[6]) {
    uint8_t ch = 0;
    portENTER_CRITICAL(&s_seen_mux);
    for (int i = 0; i < ESPNOW_SEEN_MAX; i++)
        if (s_seen[i].used && memcmp(s_seen[i].mac, mac, 6) == 0) { ch = s_seen[i].channel; break; }
    portEXIT_CRITICAL(&s_seen_mux);
    return ch;
}

static void on_state(const EspNowStatePkt &p, int rssi) {
    s_rx_count++;
    s_last_rx_ms = millis();
    s_last_rssi  = rssi;
    s_last_outputs = p.reg_outputs;
    if (p.sta_ip != 0 && p.sta_ip != s_gen_ip) {
        s_gen_ip = p.sta_ip;
        s_gen_ip_changed = true;
    }

    if (ws_client_fresh()) return;  // WS is primary while it is delivering
    s_rx_applied++;

    const char *st = p.state_id < sizeof(STATE_NAMES) / sizeof(STATE_NAMES[0])
                   ? STATE_NAMES[p.state_id] : "";
    bool working = p.flags & ESPNOW_FLAG_WORKING;

    // Fields ESP-NOW does not carry (power_apparent/reactive/factor, uptime)
    // keep whatever value they last had.
    portENTER_CRITICAL(&ws_gen_mux);
    g_gen.running         = working;
    g_gen.stopped         = !working;
    strlcpy(g_gen.state, st, sizeof(g_gen.state));
    g_gen.choke           = reg_bit(p.reg_outputs, REG_CHOKE);
    g_gen.fuel_pump       = reg_bit(p.reg_outputs, REG_FUEL_PUMP);
    g_gen.eco_mode        = reg_bit(p.reg_outputs, REG_ECO_MODE);
    g_gen.load_relay      = reg_bit(p.reg_outputs, REG_LOAD_RELAY);
    g_gen.charger         = reg_bit(p.reg_outputs, REG_CHARGER);
    g_gen.ignition        = reg_bit(p.reg_outputs, REG_IGNITION_BIT);
    g_gen.overload        = reg_bit(p.reg_inputs, REG_IN_OVERLOAD_BIT);
    g_gen.oil             = reg_bit(p.reg_inputs, REG_IN_OIL_BIT);
    g_gen.output_ok       = reg_bit(p.reg_inputs, REG_IN_OUTPUT_BIT);
    g_gen.overheat        = reg_bit(p.reg_inputs, REG_IN_OVERHEAT_BIT);
    g_gen.psu_sense       = reg_bit(p.reg_inputs, REG_IN_PSU_BIT);
    g_gen.gen_sense       = reg_bit(p.reg_inputs, REG_IN_GEN_BIT);
    g_gen.bat_sense       = reg_bit(p.reg_inputs, REG_IN_BAT_BIT);
    g_gen.rpm             = (float)p.rpm;
    g_gen.freq            = p.freq_hz;
    g_gen.vrms            = p.vrms;
    g_gen.irms            = p.irms;
    g_gen.power_active    = p.active_power_w;
    g_gen.head_temp       = p.head_temp_c;
    g_gen.fuel_level      = p.fuel_l;
    g_gen.oil_temp        = p.oil_temp_c;
    g_gen.ambient_temp    = p.ambient_temp_c;
    g_gen.inverter_temp   = p.inverter_temp_c;
    g_gen.alternator_temp = p.alternator_temp_c;
    g_gen.bat_voltage     = p.bat_v;
    g_gen.core_voltage    = p.core_v;
    g_gen.maint_due_mask  = p.maint_due_mask;
    g_gen.rssi            = 0;   // WS RSSI = controller's own WiFi; it has none to report here
    g_gen.valid           = true;
    g_gen.last_update     = s_last_rx_ms;
    portEXIT_CRITICAL(&ws_gen_mux);
}

// Journal answer (JrnEvent packets, then one JrnEnd) for the request in
// flight. Written by the WiFi driver task, read by loop() only once
// s_jrn_end_seen is set, after which the callback no longer writes it.
static portMUX_TYPE s_jrn_mux = portMUX_INITIALIZER_UNLOCKED;
static uint8_t  s_jrn_req_id = 0;
static bool     s_jrn_waiting = false;
static bool     s_jrn_end_seen = false;
static uint32_t s_jrn_have = 0;  // bit idx = event idx arrived
static char     s_jrn_json[ESPNOW_JRN_MAX_EVENTS][ESPNOW_MAX_PAYLOAD - sizeof(EspNowJrnEventHdr) + 1];
static EspNowJrnEndPkt s_jrn_end;

static void on_jrn_event(const uint8_t *data, int len) {
    EspNowJrnEventHdr h;
    memcpy(&h, data, sizeof(h));
    int jl = len - (int)sizeof(h);
    if (h.idx >= ESPNOW_JRN_MAX_EVENTS || jl <= 0 || jl >= (int)sizeof(s_jrn_json[0])) return;
    portENTER_CRITICAL(&s_jrn_mux);
    if (s_jrn_waiting && !s_jrn_end_seen && h.req_id == s_jrn_req_id) {
        memcpy(s_jrn_json[h.idx], data + sizeof(h), jl);
        s_jrn_json[h.idx][jl] = '\0';
        s_jrn_have |= 1u << h.idx;
    }
    portEXIT_CRITICAL(&s_jrn_mux);
}

static void on_jrn_end(const uint8_t *data) {
    EspNowJrnEndPkt e;
    memcpy(&e, data, sizeof(e));
    portENTER_CRITICAL(&s_jrn_mux);
    if (s_jrn_waiting && !s_jrn_end_seen && e.req_id == s_jrn_req_id) {
        s_jrn_end = e;
        s_jrn_end_seen = true;
    }
    portEXIT_CRITICAL(&s_jrn_mux);
}

// Runs in the WiFi driver task (not an ISR); keep it short. Packets are told
// apart by magic first: journal events vary in length and could otherwise
// match a fixed-size packet's length.
static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (len < 2 || data[1] != ESPNOW_PROTO_VERSION) return;
    int rssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
    const uint8_t magic = data[0];
    if (magic == ESPNOW_MAGIC_BEACON && len == sizeof(EspNowBeaconPkt)) {
        EspNowBeaconPkt b;
        memcpy(&b, data, sizeof(b));
        on_beacon(info->src_addr, b, rssi);
        return;
    }
    if (!s_gen_selected || memcmp(info->src_addr, s_gen_mac, 6) != 0) return;
    if (magic == ESPNOW_MAGIC_STATE && len == sizeof(EspNowStatePkt)) {
        EspNowStatePkt p;
        memcpy(&p, data, sizeof(p));
        on_state(p, rssi);
    } else if (magic == ESPNOW_MAGIC_JRN_EVENT && len > (int)sizeof(EspNowJrnEventHdr)) {
        on_jrn_event(data, len);
    } else if (magic == ESPNOW_MAGIC_JRN_END && len == sizeof(EspNowJrnEndPkt)) {
        on_jrn_end(data);
    }
}

// Resend only start/start_retry/stop: the controller ignores a repeated
// start while already working and a repeated stop just re-enters the stop
// sequence. A toggle is never resent — if the first one did arrive and only
// its ACK was lost, a resend would flip the register back.
#define ESPNOW_CMD_RETRIES      3
#define ESPNOW_CMD_RETRY_GAP_MS 300

// Written by the LVGL task and loop() (espnow_send_text: touch buttons,
// remote_io), the WiFi driver task (on_sent) and loop() (poll) — all under
// s_retry_mux, so a retry can never send a half-updated or newer packet
// (a stray resend of a toggle would flip the register back).
static portMUX_TYPE      s_retry_mux  = portMUX_INITIALIZER_UNLOCKED;
static EspNowCmdPkt      s_retry_pkt;
static uint8_t           s_retry_left = 0;
static bool              s_retry_due  = false;
static uint32_t          s_retry_at   = 0;

static void on_sent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
    // Hellos go to the broadcast address and are never ACKed.
    if (info && memcmp(info->des_addr, BROADCAST_MAC, 6) == 0) return;
    if (status == ESP_NOW_SEND_SUCCESS) s_tx_ok++;
    else                                s_tx_fail++;
    portENTER_CRITICAL(&s_retry_mux);
    if (status == ESP_NOW_SEND_SUCCESS) {
        s_retry_left = 0;
    } else if (s_retry_left > 0) {
        s_retry_at  = millis() + ESPNOW_CMD_RETRY_GAP_MS;
        s_retry_due = true;
    }
    portEXIT_CRITICAL(&s_retry_mux);
}

static bool add_peer(const uint8_t mac[6], bool encrypt) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, mac, 6);
    if (encrypt) memcpy(peer.lmk, s_key, ESPNOW_KEY_LEN);
    peer.channel = 0;
    peer.ifidx   = WIFI_IF_STA;
    peer.encrypt = encrypt;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        Serial.printf("[ESPNOW] add_peer failed: %s\n", esp_err_to_name(err));
        return false;
    }
    return true;
}

void espnow_client_init(void) {
    if (s_enabled) {
        esp_now_deinit();
        s_enabled = false;
    }
    s_gen_selected = false;
    // A fresh key/selection must prove itself: "active" may not ride on
    // packets received under the previous configuration.
    s_rx_count = 0;
    s_gen_ip = 0;

    if (!parse_key_hex(g_config.espnow_key, s_key)) {
        Serial.printf("[ESPNOW] No key — disabled (own MAC %s)\n", WiFi.macAddress().c_str());
        return;
    }
    esp_err_t err = esp_now_init();
    if (err != ESP_OK) {
        Serial.printf("[ESPNOW] esp_now_init failed: %s\n", esp_err_to_name(err));
        return;
    }
    esp_now_set_pmk(s_key);
    esp_now_register_recv_cb(on_recv);
    esp_now_register_send_cb(on_sent);
    if (!add_peer(BROADCAST_MAC, false)) {
        esp_now_deinit();
        return;
    }
    s_enabled = true;

    if (parse_mac(g_config.espnow_gen_mac, s_gen_mac) && add_peer(s_gen_mac, true))
        s_gen_selected = true;
    s_last_hello_ms = 0;
    Serial.printf("[ESPNOW] Enabled, own MAC %s, generator \"%s\" %s\n",
                  WiFi.macAddress().c_str(), g_config.espnow_gen_name,
                  s_gen_selected ? g_config.espnow_gen_mac : "(none)");
}

int espnow_client_rssi(void) {
    return s_last_rssi;
}

bool espnow_client_enabled(void) {
    return s_enabled;
}

bool espnow_client_active(void) {
    return s_enabled && s_gen_selected && s_rx_count > 0 &&
           millis() - s_last_rx_ms < ESPNOW_ACTIVE_WINDOW_MS;
}

bool espnow_send_text(const char *msg) {
    if (!s_enabled || !s_gen_selected) return false;
    EspNowCmdPkt p = {};
    p.magic         = ESPNOW_MAGIC_CMD;
    p.proto_version = ESPNOW_PROTO_VERSION;
    if      (!strcmp(msg, "start"))       p.cmd = ESPNOW_CMD_START;
    else if (!strcmp(msg, "start_retry")) p.cmd = ESPNOW_CMD_START_RETRY;
    else if (!strcmp(msg, "stop"))        p.cmd = ESPNOW_CMD_STOP;
    else {
        char *end;
        long reg = strtol(msg, &end, 10);
        if (end == msg || *end != '\0' || reg < 0 || reg > 15) return false;
        p.cmd = ESPNOW_CMD_REG_TOGGLE;
        p.reg = (uint8_t)reg;
    }
    portENTER_CRITICAL(&s_retry_mux);
    s_retry_due  = false;
    s_retry_pkt  = p;
    s_retry_left = (p.cmd == ESPNOW_CMD_REG_TOGGLE) ? 0 : ESPNOW_CMD_RETRIES;
    portEXIT_CRITICAL(&s_retry_mux);
    esp_err_t err = esp_now_send(s_gen_mac, (const uint8_t *)&p, sizeof(p));
    Serial.printf("[ESPNOW] send \"%s\": %s\n", msg, esp_err_to_name(err));
    return err == ESP_OK;
}

bool espnow_send_maint(uint8_t subtype) {
    if (!s_enabled || !s_gen_selected) return false;
    EspNowCmdPkt p = {};
    p.magic         = ESPNOW_MAGIC_CMD;
    p.proto_version = ESPNOW_PROTO_VERSION;
    p.cmd           = ESPNOW_CMD_MAINT;
    p.reg           = subtype;
    portENTER_CRITICAL(&s_retry_mux);
    s_retry_due  = false;
    s_retry_pkt  = p;
    s_retry_left = 0;  // never resent: a lost ACK would record it twice
    portEXIT_CRITICAL(&s_retry_mux);
    esp_err_t err = esp_now_send(s_gen_mac, (const uint8_t *)&p, sizeof(p));
    Serial.printf("[ESPNOW] send maintenance %u: %s\n", subtype, esp_err_to_name(err));
    return err == ESP_OK;
}

bool espnow_journal_request(uint8_t count) {
    if (!s_enabled || !s_gen_selected) return false;
    // on_sent() can't tell packets apart; a journal request's ACK must not
    // cancel a start/stop resend that is still pending.
    portENTER_CRITICAL(&s_retry_mux);
    bool cmd_pending = s_retry_left > 0;
    portEXIT_CRITICAL(&s_retry_mux);
    if (cmd_pending) return false;

    if (count > ESPNOW_JRN_MAX_EVENTS) count = ESPNOW_JRN_MAX_EVENTS;
    EspNowJrnReqPkt r = {ESPNOW_MAGIC_JRN_REQ, ESPNOW_PROTO_VERSION, 0, count};
    portENTER_CRITICAL(&s_jrn_mux);
    r.req_id = ++s_jrn_req_id;
    s_jrn_have = 0;
    s_jrn_end_seen = false;
    s_jrn_waiting = true;
    portEXIT_CRITICAL(&s_jrn_mux);
    esp_err_t err = esp_now_send(s_gen_mac, (const uint8_t *)&r, sizeof(r));
    Serial.printf("[ESPNOW] journal request %u: %s\n", r.req_id, esp_err_to_name(err));
    if (err != ESP_OK) {
        espnow_journal_cancel();
        return false;
    }
    return true;
}

void espnow_journal_cancel(void) {
    portENTER_CRITICAL(&s_jrn_mux);
    s_jrn_waiting = false;
    portEXIT_CRITICAL(&s_jrn_mux);
}

bool espnow_journal_take(EspNowJournalAnswer *out) {
    portENTER_CRITICAL(&s_jrn_mux);
    bool done = s_jrn_waiting && s_jrn_end_seen;
    if (done) {
        s_jrn_waiting = false;
        out->count          = s_jrn_end.count < ESPNOW_JRN_MAX_EVENTS ? s_jrn_end.count : ESPNOW_JRN_MAX_EVENTS;
        out->have           = s_jrn_have;
        out->ts_now         = s_jrn_end.ts_now;
        out->run_seconds    = s_jrn_end.run_seconds;
        out->charge_seconds = s_jrn_end.charge_seconds;
        out->starts         = s_jrn_end.starts;
        out->start_fails    = s_jrn_end.start_fails;
        out->emergencies    = s_jrn_end.emergencies;
        out->charge_cycles  = s_jrn_end.charge_cycles;
    }
    portEXIT_CRITICAL(&s_jrn_mux);
    return done;
}

const char *espnow_journal_event(int idx) {
    return (idx >= 0 && idx < ESPNOW_JRN_MAX_EVENTS) ? s_jrn_json[idx] : "";
}

int espnow_list_generators(EspNowGenInfo *out, int max) {
    int n = 0;
    bool selected_listed = false;
    uint32_t now = millis();
    portENTER_CRITICAL(&s_seen_mux);
    for (int i = 0; i < ESPNOW_SEEN_MAX && n < max; i++) {
        const SeenGen &g = s_seen[i];
        if (!g.used || now - g.last_ms > ESPNOW_SEEN_TIMEOUT_MS) continue;
        EspNowGenInfo &o = out[n++];
        memcpy(o.mac, g.mac, 6);
        strlcpy(o.name, g.name, sizeof(o.name));
        o.rssi = g.rssi;
        o.seen = true;
        o.selected = s_gen_selected && memcmp(g.mac, s_gen_mac, 6) == 0;
        if (o.selected) selected_listed = true;
    }
    portEXIT_CRITICAL(&s_seen_mux);
    if (s_gen_selected && !selected_listed && n < max) {
        EspNowGenInfo &o = out[n++];
        memcpy(o.mac, s_gen_mac, 6);
        strlcpy(o.name, g_config.espnow_gen_name, sizeof(o.name));
        o.rssi = 0;
        o.seen = false;
        o.selected = true;
    }
    return n;
}

void espnow_select_generator(const uint8_t *mac, const char *name) {
    portENTER_CRITICAL(&s_sel_mux);
    s_sel_none = (mac == nullptr);
    if (mac) memcpy(s_sel_mac, mac, 6);
    strlcpy(s_sel_name, name ? name : "", sizeof(s_sel_name));
    s_sel_pending = true;
    portEXIT_CRITICAL(&s_sel_mux);
}

bool espnow_take_gen_ip(char *buf, size_t len) {
    if (!s_gen_ip_changed) return false;
    s_gen_ip_changed = false;
    strlcpy(buf, IPAddress(s_gen_ip).toString().c_str(), len);
    return true;
}

static void apply_selection(void) {
    bool none;
    uint8_t mac[6];
    char name[ESPNOW_NAME_LEN + 1];
    portENTER_CRITICAL(&s_sel_mux);
    none = s_sel_none;
    memcpy(mac, s_sel_mac, 6);
    strlcpy(name, s_sel_name, sizeof(name));
    s_sel_pending = false;
    portEXIT_CRITICAL(&s_sel_mux);

    if (s_gen_selected) esp_now_del_peer(s_gen_mac);
    s_gen_selected = false;
    s_rx_count = 0;
    s_gen_ip = 0;
    if (none) {
        g_config.espnow_gen_mac[0] = '\0';
        g_config.espnow_gen_name[0] = '\0';
    } else {
        format_mac(mac, g_config.espnow_gen_mac, sizeof(g_config.espnow_gen_mac));
        strlcpy(g_config.espnow_gen_name, name, sizeof(g_config.espnow_gen_name));
        memcpy(s_gen_mac, mac, 6);
        s_gen_selected = add_peer(s_gen_mac, true);
    }

    // esp_wifi_set_channel() while the STA is actively associated tears the
    // connection down (seen live as a WiFi disconnect, reason 8) — the pin
    // only makes sense with no AP to begin with, which is exactly the case
    // where WiFi.status() isn't WL_CONNECTED. With an AP present, ESP-NOW
    // already rides whatever channel the STA is associated on — nothing to
    // pin. Found live 2026-10-17.
    s_pin_channel = 0;
    if (s_gen_selected && WiFi.status() != WL_CONNECTED) {
        uint8_t ch = seen_channel_for(s_gen_mac);
        if (ch != 0) {
            esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
            s_pin_channel = ch;
            s_pin_since   = millis();
            Serial.printf("[ESPNOW] pinning radio to channel %u while joining\n", ch);
        }
    }

    config_save();
    s_last_hello_ms = 0;
    Serial.printf("[ESPNOW] Selected generator \"%s\" %s\n", g_config.espnow_gen_name,
                  s_gen_selected ? g_config.espnow_gen_mac : "(none)");
}

void espnow_client_poll(void) {
    if (!s_enabled) return;
    if (s_sel_pending) apply_selection();

    if (s_gen_selected) {
        uint32_t every = espnow_client_active() ? ESPNOW_HELLO_KEEP_MS : ESPNOW_HELLO_JOIN_MS;
        if (s_last_hello_ms == 0 || millis() - s_last_hello_ms >= every) {
            s_last_hello_ms = millis();
            EspNowHelloPkt h = {ESPNOW_MAGIC_HELLO, ESPNOW_PROTO_VERSION, {0}};
            memcpy(h.target, s_gen_mac, 6);
            esp_now_send(BROADCAST_MAC, (const uint8_t *)&h, sizeof(h));
        }
    }

    EspNowCmdPkt pkt;
    uint8_t left;
    portENTER_CRITICAL(&s_retry_mux);
    bool go = s_retry_due && (int32_t)(millis() - s_retry_at) >= 0 && s_retry_left > 0;
    if (go) {
        s_retry_due = false;
        left = --s_retry_left;
        pkt = s_retry_pkt;
    }
    portEXIT_CRITICAL(&s_retry_mux);
    if (!go || !s_gen_selected) return;
    esp_err_t err = esp_now_send(s_gen_mac, (const uint8_t *)&pkt, sizeof(pkt));
    Serial.printf("[ESPNOW] resend cmd %u (%u left): %s\n", pkt.cmd, left, esp_err_to_name(err));
}

// ---- channel handling while the STA has no AP (../ESPNOW.md) -----------
// Mirror of the controller's espnow_wifi_supervise(): a connect attempt
// with the AP absent sweeps every channel, so instead park the radio on the
// last AP channel and probe only that channel for the SSID.

#define ESPNOW_PROBE_INTERVAL_MS  5000
#define ESPNOW_PROBE_MS_PER_CHAN  120

static uint8_t  s_home_channel = 0;
static char     s_home_ssid[33] = "";
static bool     s_home_loaded = false;
static bool     s_probe_running = false;
static uint32_t s_last_probe_ms = 0;

static void home_load(void) {
    Preferences prefs;
    if (prefs.begin("espnow", true)) {
        s_home_channel = prefs.getUChar("ch", 0);
        prefs.getString("ssid", s_home_ssid, sizeof(s_home_ssid));
        prefs.end();
    }
    s_home_loaded = true;
    Serial.printf("[ESPNOW] home channel %u, ssid \"%s\"\n", s_home_channel, s_home_ssid);
}

static void home_save(uint8_t ch, const char *ssid) {
    s_home_channel = ch;
    strlcpy(s_home_ssid, ssid, sizeof(s_home_ssid));
    Preferences prefs;
    if (prefs.begin("espnow", false)) {
        prefs.putUChar("ch", ch);
        prefs.putString("ssid", s_home_ssid);
        prefs.end();
    }
    Serial.printf("[ESPNOW] home channel saved: %u, ssid \"%s\"\n", ch, s_home_ssid);
}

bool espnow_wifi_join_pinned(void) {
    if (s_pin_channel == 0 || !s_gen_selected) return false;
    if (espnow_client_active()) return false;
    if (millis() - s_pin_since >= ESPNOW_PIN_TIMEOUT_MS) return false;
    return true;
}

bool espnow_wifi_supervise(bool connected, bool may_probe) {
    if (!s_enabled) return false;
    if (espnow_wifi_join_pinned()) return false;
    if (!s_home_loaded) home_load();

    if (connected) {
        // WiFi.channel() reports whatever channel an active scan (the
        // Config tab's "Scan" button) is currently probing, not the real
        // associated channel — reading it mid-scan corrupts the saved home
        // channel with the scan's own sweep. Found live 2026-10-17: pressing
        // Scan while connected wrote home channel 1,2,3...14 in sequence,
        // and the next reconnect used one of those bogus values as its
        // channel hint, which cascaded into "associated but no IP" radio
        // restarts and ESP-NOW deinit races.
        if (WiFi.scanComplete() == WIFI_SCAN_RUNNING) return false;
        uint8_t ch = (uint8_t)WiFi.channel();
        String ssid = WiFi.SSID();
        // WiFi.SSID() is already empty in the loop pass that sees the drop.
        if (ch != 0 && ssid.length() > 0 &&
            (ch != s_home_channel || strcmp(ssid.c_str(), s_home_ssid) != 0))
            home_save(ch, ssid.c_str());
        return false;
    }
    if (s_home_channel == 0 || s_home_ssid[0] == '\0') return false;

    if (s_probe_running) {
        int16_t n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) return false;
        s_probe_running = false;
        bool seen = false;
        for (int16_t i = 0; i < n; i++) {
            if (WiFi.SSID(i) == s_home_ssid) seen = true;
        }
        WiFi.scanDelete();
        if (seen) {
            Serial.printf("[ESPNOW] \"%s\" back on channel %u — reconnecting\n", s_home_ssid, s_home_channel);
            return true;
        }
    }

    if (!may_probe || WiFi.scanComplete() == WIFI_SCAN_RUNNING) return false;

    uint8_t primary = 0;
    wifi_second_chan_t second;
    if (esp_wifi_get_channel(&primary, &second) == ESP_OK && primary != s_home_channel)
        esp_wifi_set_channel(s_home_channel, WIFI_SECOND_CHAN_NONE);

    if (millis() - s_last_probe_ms >= ESPNOW_PROBE_INTERVAL_MS) {
        s_last_probe_ms = millis();
        if (WiFi.scanNetworks(true, false, false, ESPNOW_PROBE_MS_PER_CHAN, s_home_channel) == WIFI_SCAN_RUNNING)
            s_probe_running = true;
    }
    return false;
}

bool espnow_wifi_probe_running(void) {
    return s_probe_running;
}

uint8_t espnow_home_channel(void) {
    return (s_enabled && s_home_ssid[0] != '\0') ? s_home_channel : 0;
}

void espnow_client_status_json(char *buf, size_t len) {
    char ip[16] = "";
    if (s_gen_ip) strlcpy(ip, IPAddress(s_gen_ip).toString().c_str(), sizeof(ip));
    int n = snprintf(buf, len,
             "{\"enabled\":%s,\"own_mac\":\"%s\",\"gen_name\":\"%s\",\"gen_mac\":\"%s\","
             "\"gen_ip\":\"%s\",\"active\":%s,\"rx\":%lu,\"rx_applied\":%lu,"
             "\"last_rx_age_ms\":%ld,\"rssi\":%d,\"tx_ok\":%lu,\"tx_fail\":%lu,"
             "\"wifi_channel\":%d,\"outputs\":%u,\"home_channel\":%u,\"generators\":[",
             s_enabled ? "true" : "false", WiFi.macAddress().c_str(),
             g_config.espnow_gen_name, g_config.espnow_gen_mac, ip,
             espnow_client_active() ? "true" : "false",
             (unsigned long)s_rx_count, (unsigned long)s_rx_applied,
             s_rx_count ? (long)(millis() - s_last_rx_ms) : -1L,
             s_last_rssi, (unsigned long)s_tx_ok, (unsigned long)s_tx_fail,
             (int)WiFi.channel(), (unsigned)s_last_outputs, (unsigned)s_home_channel);
    EspNowGenInfo gens[ESPNOW_SEEN_MAX];
    int count = espnow_list_generators(gens, ESPNOW_SEEN_MAX);
    for (int i = 0; i < count && n > 0 && (size_t)n < len; i++) {
        char mac[18];
        format_mac(gens[i].mac, mac, sizeof(mac));
        n += snprintf(buf + n, len - n, "%s{\"name\":\"%s\",\"mac\":\"%s\",\"rssi\":%d,\"seen\":%s}",
                      i ? "," : "", gens[i].name, mac, gens[i].rssi, gens[i].seen ? "true" : "false");
    }
    if (n > 0 && (size_t)n < len) snprintf(buf + n, len - n, "]}");
}
