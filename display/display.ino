/*
 * Generator Control Panel
 * ESP32-S3 + JC3248W535EN (320x480 QSPI, AXS15231B touch)
 * LVGL v8 — landscape 480x320 (90° rotation)
 *
 * Libraries required:
 *   - lvgl >= 8.3.9, < 9
 *   - WebSockets (Markus Sattler / arduinoWebSockets)
 *   - ArduinoJson >= 6
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <lvgl.h>
#include "driver/i2s.h"
#include <math.h>

#include "display.h"
#include "esp_bsp.h"
#include "lv_port.h"
#include "config.h"
#include "ws_client.h"
#include "ui.h"
#include "journal.h"
#include "remote_io.h"
#include "esp_now_client.h"
#include "udp_discovery.h"

// ---------------------------------------------------------------------------
// Audio beep (I2S) — pines from DEMO_MP3/pincfg.h
// ---------------------------------------------------------------------------
#define AUDIO_I2S_PORT   I2S_NUM_0
#define AUDIO_I2S_BCK    42
#define AUDIO_I2S_LRCK   2
#define AUDIO_I2S_DO     41
#define AUDIO_SAMPLE_RATE 22050

static volatile bool s_beep_busy = false;

static void beep_play(uint16_t freq_hz, uint16_t duration_ms, uint16_t amplitude = 7000) {
    const int sr    = AUDIO_SAMPLE_RATE;
    const int total = sr * duration_ms / 1000;
    const int chunk = 256;
    int16_t   buf[chunk * 2];
    int       done  = 0;
    while (done < total) {
        int n = (total - done < chunk) ? (total - done) : chunk;
        for (int i = 0; i < n; i++) {
            int16_t s = (int16_t)(amplitude * sinf(2.0f * M_PI * freq_hz * (done + i) / sr));
            buf[i*2]   = s;
            buf[i*2+1] = s;
        }
        size_t written;
        i2s_write(AUDIO_I2S_PORT, buf, n * 4, &written, pdMS_TO_TICKS(200));
        done += n;
    }
}

static void beep_task(void *arg) {
    int type = (int)(intptr_t)arg;
    if (type == 1) {                    // generator started: two ascending beeps
        beep_play(880,  120);
        vTaskDelay(pdMS_TO_TICKS(60));
        beep_play(1100, 150);
    } else if (type == 2) {             // alarm: three rapid high-pitched beeps
        for (int i = 0; i < 3; i++) {
            beep_play(1400, 80, 9000);
            if (i < 2) vTaskDelay(pdMS_TO_TICKS(70));
        }
    } else {                            // generator stopped: single descending beep
        beep_play(660, 200);
    }
    i2s_zero_dma_buffer(AUDIO_I2S_PORT);
    s_beep_busy = false;
    vTaskDelete(NULL);
}

static void beep_trigger(int type) {
    if (s_beep_busy) return;            // ignore if still playing
    s_beep_busy = true;
    xTaskCreate(beep_task, "beep", 4096, (void *)(intptr_t)type, 2, NULL);
}

static void beep_init() {
    i2s_config_t cfg = {};
    cfg.mode               = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate        = AUDIO_SAMPLE_RATE;
    cfg.bits_per_sample    = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format     = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.intr_alloc_flags   = 0;
    cfg.dma_buf_count      = 4;
    cfg.dma_buf_len        = 256;
    cfg.use_apll           = false;
    i2s_driver_install(AUDIO_I2S_PORT, &cfg, 0, NULL);

    i2s_pin_config_t pins = {};
    pins.bck_io_num   = AUDIO_I2S_BCK;
    pins.ws_io_num    = AUDIO_I2S_LRCK;
    pins.data_out_num = AUDIO_I2S_DO;
    pins.data_in_num  = I2S_PIN_NO_CHANGE;
    i2s_set_pin(AUDIO_I2S_PORT, &pins);
    i2s_zero_dma_buffer(AUDIO_I2S_PORT);
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool wifi_was_connected  = false;
static bool wifi_connecting     = false;   // between begin() and WL_CONNECTED
static bool ws_initialized      = false;
static bool ota_started         = false;   // setup_ota() must run only once:
                                           // each ota_server.on() APPENDS a new
                                           // handler — re-running it on every
                                           // WiFi reconnect leaks heap
static bool scan_pending        = false;
static bool ota_active          = false;
static bool gen_was_running     = false;
static bool gen_state_known     = false;   // true after first valid update
static bool gen_host_ui_dirty   = false;   // gen_host changed outside the Config tab

static uint32_t last_ws_ok_ms   = 0;       // last time WS was connected
static uint32_t last_alarm_ms   = 0;       // last alarm beep timestamp

#define WS_RECONNECT_TIMEOUT_MS  60000     // hard reconnect after 60 s — don't interfere with library's 3 s retry
#define WS_SILENT_RECONNECT_MS   10000     // connected but no report this long -> reconnect
#define GEN_DATA_STALE_MS        6000      // invalidate g_gen if no update for 6 s (WS report arrives at ~1 Hz from ReportTask)
#define ALARM_INTERVAL_MS        5000      // repeat alarm beep every 5 s

static uint32_t last_ui_ms        = 0;
static uint32_t last_scan_ms      = 0;
static uint32_t last_reconnect_ms = 0;

#define WIFI_RECONNECT_INTERVAL_MS 15000   // retry interval when disconnected
#define WIFI_FULL_RESCAN_INTERVAL_MS 300000 // same, while ESP-NOW parks on the home channel
#define WIFI_CONNECT_ATTEMPT_MS    10000   // radio left alone this long after WiFi.begin()

// Grace period after 802.11 association while DHCP is still running.
// Killing the connection with disconnect()+begin() at the plain 15 s mark
// aborts our own nearly-established connection (reason=8 ASSOC_LEAVE in the
// log). Association state is tracked via WiFi events.
#define WIFI_DHCP_GRACE_MS         45000

static volatile bool wifi_associated = false;  // set by WiFi event task

// ESP32-S3 boards with a display (incl. JC3248W535) often fail to associate
// at the default 19.5 dBm: the 3.3V rail sags during TX bursts. But 8.5 dBm
// proved too weak for the DHCP exchange (association OK, no IP) — 15 dBm is
// the middle ground: enough uplink budget, still below the brownout peak.
// NOTE: resets to default whenever WiFi restarts — must be re-applied after
// every WiFi.begin().
#define WIFI_TX_POWER_LEVEL WIFI_POWER_15dBm

// This panel's own 1S Li-ion, not the generator battery reported over WS.
// Schematic: BAT — R26 33k — GPIO5 — R27 100k — GND. GPIO5 is ADC1 CH4.
// The ideal ratio is 133/100, but at 11 dB the ADC input (~88k) sits in
// parallel with the 100k and pulls the pin down. Bench check on this
// board: 4.148 V on the cell read as 2430 mV, so the scale is 4.148/2.430.
#define PANEL_BAT_ADC_PIN 5
static constexpr float PANEL_BAT_SCALE = 4.148f / 2.430f;

static void panel_bat_init() {
    analogReadResolution(12);
    analogSetPinAttenuation(PANEL_BAT_ADC_PIN, ADC_11db);
    (void)analogReadMilliVolts(PANEL_BAT_ADC_PIN);
}

// One raw conversion per call. Show the icon only after five samples
// (~1.5 s at the 300 ms UI tick) sit inside 2.80–4.35 V with < 80 mV of
// spread — an open BAT pin on USB power jitters or sits near 0 V.
// Hide only after five samples are all below 2.70 V or all above 4.40 V,
// so backlight sag does not drop a real cell off the bar.
static void panel_bat_sample(bool *present_out, float *volts_out) {
    static bool  seeded  = false;
    static bool  present = false;
    static float filt    = 0;
    static float win[5];
    static int   n = 0;
    static int   i = 0;

    uint32_t mv = analogReadMilliVolts(PANEL_BAT_ADC_PIN);
    int      raw = analogRead(PANEL_BAT_ADC_PIN);
    float v = (mv / 1000.0f) * PANEL_BAT_SCALE;
    if (!seeded) {
        filt = v;
        seeded = true;
    } else {
        filt = filt * (7.0f / 8.0f) + v * (1.0f / 8.0f);
    }

    win[i] = v;
    i = (i + 1) % 5;
    if (n < 5) n++;

    if (n == 5) {
        float mn = win[0], mx = win[0];
        bool all_low = true, all_high = true;
        for (int k = 0; k < 5; k++) {
            if (win[k] < mn) mn = win[k];
            if (win[k] > mx) mx = win[k];
            if (win[k] >= 2.70f) all_low = false;
            if (win[k] <= 4.40f) all_high = false;
        }
        bool was = present;
        if (!present) {
            if (mn >= 2.80f && mx <= 4.35f && (mx - mn) < 0.080f) present = true;
        } else if (all_low || all_high) {
            present = false;
        }
        if (present != was) {
            Serial.printf("[BAT] panel battery %s, %.2f V\n",
                          present ? "present" : "absent", filt);
        }
    }

    static uint8_t dbg = 0;
    if (++dbg >= 7) {  // ~2 s at the 300 ms UI tick
        dbg = 0;
        float mn = 0, mx = 0;
        if (n > 0) {
            mn = mx = win[0];
            for (int k = 1; k < n; k++) {
                if (win[k] < mn) mn = win[k];
                if (win[k] > mx) mx = win[k];
            }
        }
        Serial.printf("[BAT] raw=%d mv=%lu scaled=%.2f filt=%.2f "
                      "present=%d n=%d min=%.2f max=%.2f\n",
                      raw, (unsigned long)mv, v, filt,
                      present ? 1 : 0, n, mn, mx);
    }

    *present_out = present;
    *volts_out   = filt;
}

// ---------------------------------------------------------------------------
// OTA HTTP server (port 8080)
// ---------------------------------------------------------------------------
static WebServer ota_server(8080);

static const char OTA_INDEX_HTML[] PROGMEM = R"(
<!DOCTYPE html><html><head><meta charset="utf-8">
<title>Generator Panel — OTA Update</title>
<style>body{font-family:sans-serif;background:#0d1117;color:#e6edf3;
  display:flex;flex-direction:column;align-items:center;padding:40px}
h2{color:#388bfd}
form{display:flex;flex-direction:column;gap:12px;width:320px}
input[type=file]{padding:8px;background:#161b22;border:1px solid #30363d;color:#e6edf3}
button{padding:12px;background:#1f6feb;color:#fff;border:none;
  border-radius:6px;cursor:pointer;font-size:16px}
button:hover{background:#388bfd}
#status{margin-top:16px;color:#3fb950}
</style></head><body>
<h2>Firmware Update</h2>
<form method="POST" action="/update" enctype="multipart/form-data">
  <input type="file" name="image" accept=".bin" required>
  <button type="submit">Upload &amp; Flash</button>
</form>
<div id="status"></div>
</body></html>
)";

static void ota_progress_cb(size_t current, size_t final_size) {
    int pct = (final_size > 0) ? (int)(100ULL * current / final_size) : 0;
    Serial.printf("[OTA] %d%%\n", pct);
    if (bsp_display_lock(20)) {
        ui_ota_set_progress(pct);
        bsp_display_unlock();
    }
}

static void setup_ota() {
    // Status page
    ota_server.on("/", HTTP_GET, []() {
        ota_server.sendHeader("Content-Encoding", "identity");
        ota_server.send_P(200, "text/html", OTA_INDEX_HTML);
    });

    // Upload handler
    ota_server.on("/update", HTTP_POST,
        // After upload completes
        []() {
            bool ok = !Update.hasError();
            ota_server.sendHeader("Connection", "close");
            ota_server.send(200, "text/plain", ok ? "OK" : "FAIL");
            Serial.printf("[OTA] Done, success=%d\n", ok);
            delay(500);
            if (bsp_display_lock(100)) {
                if (ok) ui_ota_set_progress(100);
                else    ui_ota_hide();
                bsp_display_unlock();
            }
            delay(1500);
            ESP.restart();
        },
        // While receiving data
        []() {
            HTTPUpload &upload = ota_server.upload();

            if (upload.status == UPLOAD_FILE_START) {
                Serial.printf("[OTA] Start: %s\n", upload.filename.c_str());
                ota_active = true;
                if (bsp_display_lock(100)) {
                    ui_ota_show();
                    bsp_display_unlock();
                }
                Update.onProgress(ota_progress_cb);
                if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                    Update.printError(Serial);
                }

            } else if (upload.status == UPLOAD_FILE_WRITE) {
                if (Update.write(upload.buf, upload.currentSize)
                        != upload.currentSize) {
                    Update.printError(Serial);
                }

            } else if (upload.status == UPLOAD_FILE_END) {
                if (!Update.end(true)) {
                    Update.printError(Serial);
                }
                ota_active = false;
            }
        }
    );

    // ESP-NOW provisioning (../ESPNOW.md). Unauthenticated, like /update
    // above — LAN-only, same trust level as reflashing the panel outright.
    // POST fields, each optional: `key` (fleet key), `gen_mac` + `gen_name`
    // (select a generator without the touchscreen; empty gen_mac = none).
    static char espnow_json[1024];
    ota_server.on("/espnow", HTTP_GET, []() {
        espnow_client_status_json(espnow_json, sizeof(espnow_json));
        ota_server.send(200, "application/json", espnow_json);
    });
    ota_server.on("/espnow", HTTP_POST, []() {
        if (ota_server.hasArg("key"))
            strlcpy(g_config.espnow_key, ota_server.arg("key").c_str(), sizeof(g_config.espnow_key));
        if (ota_server.hasArg("gen_mac")) {
            strlcpy(g_config.espnow_gen_mac,  ota_server.arg("gen_mac").c_str(),  sizeof(g_config.espnow_gen_mac));
            strlcpy(g_config.espnow_gen_name, ota_server.arg("gen_name").c_str(), sizeof(g_config.espnow_gen_name));
        }
        config_save();
        espnow_client_init();
        espnow_client_status_json(espnow_json, sizeof(espnow_json));
        ota_server.send(espnow_client_enabled() ? 200 : 400, "application/json", espnow_json);
    });

    ota_server.begin();
    Serial.println("[OTA] Server started on port 8080");
}

// ---------------------------------------------------------------------------
// Connect callback from Config tab
// ---------------------------------------------------------------------------
static void on_connect_request(const char *ssid, const char *pass,
                                const char *host) {
    Serial.printf("[APP] Connect request: ssid=%s host=%s\n", ssid, host);

    strlcpy(g_config.wifi_ssid, ssid, sizeof(g_config.wifi_ssid));
    strlcpy(g_config.wifi_pass, pass, sizeof(g_config.wifi_pass));
    strlcpy(g_config.gen_host,  host, sizeof(g_config.gen_host));
    config_save();

    WiFi.disconnect(true);
    delay(200);
    WiFi.begin(ssid, pass);
    WiFi.setTxPower(WIFI_TX_POWER_LEVEL);  // reset by disconnect(true) — reapply
    espnow_client_init();  // disconnect(true) calls STA.end(), tearing ESP-NOW down with it — same as the WIFI_OFF radio-restart path below
    wifi_connecting   = true;
    last_reconnect_ms = millis();
    ws_initialized    = false;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    Serial.println("[APP] Generator Panel starting");
    beep_init();
    panel_bat_init();
    remote_io_init();

    // Init display + LVGL
    // Default stack (4096) is too small for 9 charts — increase to 12 KB
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = {
            .task_priority    = 4,
            .task_stack       = 12288,
            .task_affinity    = -1,
            .task_max_sleep_ms = 500,
            .timer_period_ms  = 5,
        },
        .buffer_size   = EXAMPLE_LCD_QSPI_H_RES * EXAMPLE_LCD_QSPI_V_RES,
        .rotate        = LV_DISP_ROT_90,
    };
    bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();

    // Config (SPIFFS)
    config_init();

    // Build UI
    bsp_display_lock(0);
    ui_init();
    ui_on_connect_request(on_connect_request);
    bsp_display_unlock();

    // Log WiFi events with disconnect reason codes — essential for diagnosing
    // association problems (2=AUTH_EXPIRE, 15=4WAY_HANDSHAKE_TIMEOUT=wrong
    // password, 200=BEACON_TIMEOUT=weak signal, 201=NO_AP_FOUND, 202=AUTH_FAIL,
    // 203=ASSOC_FAIL). Runs in the WiFi event task — Serial only, no LVGL.
    WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info) {
        switch (event) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            wifi_associated = true;
            Serial.printf("[%lu] [WiFi] Associated with AP, RSSI: %d dBm\n",
                          millis(), WiFi.RSSI());
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            Serial.printf("[%lu] [WiFi] Got IP: %s RSSI: %d dBm\n",
                          millis(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
            break;
        case ARDUINO_EVENT_WIFI_STA_LOST_IP:
            Serial.printf("[%lu] [WiFi] Lost IP\n", millis());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            wifi_associated = false;
            Serial.printf("[%lu] [WiFi] Disconnected, reason=%d\n",
                          millis(), info.wifi_sta_disconnected.reason);
            break;
        default:
            break;
        }
    });

    // Start WiFi
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // disable modem sleep — reduces WS latency/drops
    WiFi.setAutoReconnect(false);  // we handle reconnect manually
    espnow_client_init();
    udp_discovery_init();
    if (strlen(g_config.wifi_ssid) > 0) {
        Serial.printf("[APP] Connecting to WiFi: %s\n", g_config.wifi_ssid);
        WiFi.begin(g_config.wifi_ssid, g_config.wifi_pass);
        WiFi.setTxPower(WIFI_TX_POWER_LEVEL);
        wifi_connecting     = true;
        last_reconnect_ms   = millis();
    } else {
        Serial.println("[APP] No WiFi configured — open Config tab");
    }

    Serial.println("[APP] Setup done");
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
// Heap monitoring: restart if free heap drops below this threshold.
#define HEAP_MIN_FREE_BYTES  40000
#define HEAP_LOG_INTERVAL_MS 60000

// LVGL watchdog: if bsp_display_lock() hasn't succeeded for this long,
// the LVGL task is stuck inside draw_bitmap() on a frozen SPI DMA transfer.
#define LVGL_WATCHDOG_MS 10000

void loop() {
    bool wifi_ok = (WiFi.status() == WL_CONNECTED);

    // WiFi state change
    if (wifi_ok && !wifi_was_connected) {
        wifi_was_connected = true;
        wifi_connecting    = false;
        Serial.printf("[APP] WiFi connected, IP: %s\n",
                      WiFi.localIP().toString().c_str());
        // The controller's WS endpoint now shares GEN_HTTP_PORT with its HTTP
        // server, at /ws (see CLAUDE.md's Web/MQTT interfaces section,
        // 2026-09-25 — WebSockets4WebServer, previously a separate +1 port).
        // Port is fixed (GEN_HTTP_PORT, config.h) — not user-configurable,
        // since it never changes in practice.
        ws_client_init(g_config.gen_host, GEN_HTTP_PORT);
        ws_initialized = true;
        if (!ota_started) {
            setup_ota();
            ota_started = true;
        } else {
            // Handlers are already registered — just re-arm the listening
            // socket in case it died with the previous netif.
            ota_server.begin();
        }
    } else if (!wifi_ok && wifi_was_connected) {
        wifi_was_connected = false;
        ws_initialized     = false;
        ws_client_disconnect();
        portENTER_CRITICAL(&ws_gen_mux);
        memset(&g_gen, 0, sizeof(g_gen));
        portEXIT_CRITICAL(&ws_gen_mux);
        gen_state_known    = false;
        last_ws_ok_ms      = millis();
        Serial.println("[APP] WiFi disconnected");
        last_reconnect_ms = millis() - WIFI_RECONNECT_INTERVAL_MS;
    }

    // Auto-reconnect when disconnected and SSID is configured.
    // Skip while an async scan is running: WiFi.begin() aborts the scan and
    // the scan in turn can wedge the association attempt.
    // Once associated (DHCP in progress) use the longer grace period —
    // disconnect()+begin() here would tear down our own connection.
    //
    // With ESP-NOW enabled and a known home channel, a plain retry every
    // 15 s would sweep all channels each time and keep ESP-NOW deaf for the
    // whole AP outage. espnow_wifi_supervise() instead parks the radio on
    // the home channel, probes only that channel, and reports when the SSID
    // is back — then connect immediately with that channel as the hint.
    // Full-sweep retries drop to WIFI_FULL_RESCAN_INTERVAL_MS (AP moved).
    bool may_probe = !wifi_ok && !scan_pending && !wifi_associated &&
                     millis() - last_reconnect_ms >= WIFI_CONNECT_ATTEMPT_MS;
    bool ap_back   = espnow_wifi_supervise(wifi_ok, may_probe);
    uint8_t home_ch = espnow_home_channel();
    // A just-selected generator's own channel wins outright while joining —
    // see esp_now_client.h's espnow_wifi_join_pinned() for why a plain AP
    // full-sweep here would drag the radio right back off it.
    bool join_pinned = espnow_wifi_join_pinned();
    espnow_client_poll();
    udp_discovery_poll();

    // The selected generator's IP arrives inside its encrypted ESP-NOW state,
    // so the WS target follows the generator picked on the Config tab.
    char learned_ip[16];
    if (espnow_take_gen_ip(learned_ip, sizeof(learned_ip)) &&
        strcmp(learned_ip, g_config.gen_host) != 0) {
        Serial.printf("[APP] Generator IP from ESP-NOW: %s (was %s)\n",
                      learned_ip, g_config.gen_host);
        strlcpy(g_config.gen_host, learned_ip, sizeof(g_config.gen_host));
        config_save();
        gen_host_ui_dirty = true;
        if (ws_initialized) ws_client_init(g_config.gen_host, GEN_HTTP_PORT);
    }

    // LAN UDP discovery (udp_discovery.h) hands over a ready-made IP
    // directly — no radio pairing involved, so just repoint WS at it.
    char lan_ip[16];
    if (ui_take_lan_pick(lan_ip, sizeof(lan_ip)) &&
        strcmp(lan_ip, g_config.gen_host) != 0) {
        Serial.printf("[APP] Generator IP from LAN discovery: %s (was %s)\n",
                      lan_ip, g_config.gen_host);
        strlcpy(g_config.gen_host, lan_ip, sizeof(g_config.gen_host));
        config_save();
        gen_host_ui_dirty = true;
        if (ws_initialized) ws_client_init(g_config.gen_host, GEN_HTTP_PORT);
    }

    if (!wifi_ok && !scan_pending && !espnow_wifi_probe_running() && !join_pinned &&
        strlen(g_config.wifi_ssid) > 0) {
        uint32_t retry_after = wifi_associated ? WIFI_DHCP_GRACE_MS
                             : home_ch         ? WIFI_FULL_RESCAN_INTERVAL_MS
                                               : WIFI_RECONNECT_INTERVAL_MS;
        if (ap_back || millis() - last_reconnect_ms >= retry_after) {
            last_reconnect_ms = millis();
            wifi_connecting   = true;
            if (wifi_associated) {
                // Associated but DHCP never completed. A plain
                // disconnect()+begin() does not reset the lwIP DHCP client —
                // a full radio cycle does.
                Serial.printf("[%lu] [APP] Associated but no IP for %ds — radio restart (%s)\n",
                              millis(), (int)(WIFI_DHCP_GRACE_MS / 1000),
                              g_config.wifi_ssid);
                WiFi.mode(WIFI_OFF);
                delay(100);
                WiFi.mode(WIFI_STA);
                WiFi.setSleep(false);
                espnow_client_init();  // WIFI_OFF tears ESP-NOW down with the driver
            } else {
                Serial.printf("[%lu] [APP] WiFi reconnect attempt (%s)...\n",
                              millis(), g_config.wifi_ssid);
                WiFi.disconnect(false);
            }
            WiFi.begin(g_config.wifi_ssid, g_config.wifi_pass, ap_back ? home_ch : 0);
            WiFi.setTxPower(WIFI_TX_POWER_LEVEL);
        }
    }

    // Take a consistent snapshot of g_gen for this loop iteration.
    // The WS task (core 0) writes g_gen; we read it here (core 1).
    GenData gen;
    portENTER_CRITICAL(&ws_gen_mux);
    gen = g_gen;
    portEXIT_CRITICAL(&ws_gen_mux);

    // Hardware remote start/stop buttons + autostart trigger + status
    // outputs. Polled unconditionally every loop() iteration (not gated
    // behind the UI's own 300ms tick below) so button debounce timing and
    // latency don't depend on LVGL's refresh cadence or OTA state.
    link_ui_state_t link = !gen.valid             ? LINK_UI_NONE   :
                           ws_client_fresh()      ? LINK_UI_WS     :
                           espnow_client_active() ? LINK_UI_ESPNOW : LINK_UI_NONE;
    bool link_ok = link != LINK_UI_NONE;
    remote_io_poll(gen, link_ok);

    // WebSocket watchdog (library reconnect runs in ws_task on core 0)
    // Judged on WS's own reports, not gen.valid: ESP-NOW keeps gen fresh, so
    // a gen-based check would never notice a dead WS while ESP-NOW is up.
    if (ws_initialized) {
        if (ws_client_fresh()) {
            last_ws_ok_ms = millis();
        }
        // Half-open socket: "connected", but nothing arrives (seen after a
        // controller reboot). The heartbeat catches it too; this is the
        // backstop and covers a server that answers pings but sends nothing.
        static uint32_t last_silent_reconnect_ms = 0;
        if (ws_client_silent_ms() >= WS_SILENT_RECONNECT_MS &&
            millis() - last_silent_reconnect_ms >= WS_SILENT_RECONNECT_MS) {
            last_silent_reconnect_ms = millis();
            Serial.println("[APP] WS connected but silent — forcing reconnect");
            ws_client_reconnect();
        }
        // Hard reconnect if not connected for too long
        if (!ws_client_connected() &&
            millis() - last_ws_ok_ms >= WS_RECONNECT_TIMEOUT_MS) {
            last_ws_ok_ms = millis();
            Serial.println("[APP] WS reconnect timeout — forcing reconnect");
            ws_client_reconnect();
        }
    }

    // Stale data: no update for too long → clear and reconnect
    if (gen.valid && millis() - gen.last_update >= GEN_DATA_STALE_MS) {
        portENTER_CRITICAL(&ws_gen_mux);
        g_gen.valid = false;
        portEXIT_CRITICAL(&ws_gen_mux);
        gen.valid       = false;
        gen_state_known = false;
        Serial.println("[APP] Gen data stale — clearing display, reconnecting WS");
        ws_client_reconnect();
        last_ws_ok_ms = millis();
    }

    // Alarm beep: OVLD / OIL / OVHEAT — repeat every ALARM_INTERVAL_MS
    if (gen.valid && (gen.overload || gen.oil || gen.overheat)) {
        if (millis() - last_alarm_ms >= ALARM_INTERVAL_MS) {
            last_alarm_ms = millis();
            Serial.printf("[APP] Alarm: OVLD=%d OIL=%d OVHEAT=%d\n",
                          gen.overload, gen.oil, gen.overheat);
            beep_trigger(2);
        }
    } else {
        last_alarm_ms = millis();
    }

    // OTA server
    if (wifi_ok) {
        ota_server.handleClient();
    }

    // Journal fetch (blocks this loop iteration for the HTTP round-trips;
    // LVGL keeps running in its own task)
    journal_poll(wifi_ok && !ota_active);

    // Check pending WiFi scan
    if (scan_pending) {
        int n = WiFi.scanComplete();
        if (n >= 0) {
            // Only touch LVGL under the lock; on timeout retry next loop —
            // calling UI code unlocked corrupts LVGL structures.
            if (bsp_display_lock(10)) {
                scan_pending = false;
                ui_wifi_scan_results(n);
                bsp_display_unlock();
            }
        }
    }

    // espnow_wifi_supervise()'s own probe scan is not a UI scan.
    if (!scan_pending && !espnow_wifi_probe_running() && (millis() - last_scan_ms > 200)) {
        last_scan_ms = millis();
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) scan_pending = true;
    }

    // UI refresh every 300 ms + LVGL watchdog
    //
    // Watchdog design note: this must trigger only on *repeated, consecutive*
    // lock failures, not on wall-clock time since the last success. A single
    // failed 50 ms lock attempt combined with a stale last-success timestamp
    // used to be enough to restart the board — but that timestamp can go
    // stale for reasons that have nothing to do with LVGL: journal_poll()
    // and the WiFi reconnect branches above can each block this very loop()
    // for several seconds, during which bsp_display_lock() is never even
    // attempted. When the loop finally reaches this code again, one bad-luck
    // 50 ms timeout (LVGL merely mid-frame) was enough to read as "LVGL has
    // been stuck the whole time" and reboot — even though the LVGL task was
    // never actually wedged. Requiring several consecutive failures, each
    // sampled ~300 ms apart, means the LVGL task must genuinely fail to
    // yield the mutex across the whole window, not just once after an
    // unrelated stall elsewhere in loop().
    static uint32_t last_lvgl_lock_ok_ms   = 0;
    static bool     lvgl_watchdog_armed    = false;
    static uint16_t lvgl_lock_fail_streak  = 0;

    if (!ota_active && millis() - last_ui_ms >= 300) {
        last_ui_ms = millis();
        // Sample outside the LVGL lock. Still runs when the lock fails, so
        // the five-sample presence window does not stall on a mid-frame draw.
        bool  panel_bat_present = false;
        float panel_bat_volts   = 0;
        panel_bat_sample(&panel_bat_present, &panel_bat_volts);
        if (bsp_display_lock(50)) {
            last_lvgl_lock_ok_ms  = millis();
            lvgl_watchdog_armed   = true;
            lvgl_lock_fail_streak = 0;
            wifi_ui_state_t wst = wifi_ok        ? WIFI_ST_CONNECTED  :
                                  wifi_connecting ? WIFI_ST_CONNECTING :
                                                    WIFI_ST_OFF;
            ui_set_wifi_status(wst, wifi_ok ? WiFi.RSSI() : 0);
            ui_set_link_status(link, espnow_client_rssi());
            if (gen_host_ui_dirty) {
                ui_set_gen_host(g_config.gen_host);
                gen_host_ui_dirty = false;
            }
            ui_gen_list_refresh();
            if (gen.valid) {
                ui_update(gen);
            }
            ui_set_panel_battery(panel_bat_present, panel_bat_volts);
            // Detect generator start/stop → beep
            if (gen.valid) {
                if (!gen_state_known) {
                    gen_was_running = gen.running;
                    gen_state_known = true;
                } else if (gen.running != gen_was_running) {
                    beep_trigger(gen.running ? 1 : 0);
                    gen_was_running = gen.running;
                }
            }
            bsp_display_unlock();
        } else if (lvgl_watchdog_armed) {
            lvgl_lock_fail_streak++;
            uint32_t stuck_ms = (uint32_t)lvgl_lock_fail_streak * 300;
            if (stuck_ms >= LVGL_WATCHDOG_MS) {
                // LVGL task stuck in draw_bitmap() on frozen SPI DMA — restart.
                Serial.printf("[APP] LVGL task stuck for >%ums (%u consecutive "
                              "lock failures) — restarting\n",
                              stuck_ms, lvgl_lock_fail_streak);
                delay(200);
                ESP.restart();
            }
        }
    }

    // Heap monitor: log every minute, restart if critically low
    static uint32_t last_heap_ms = 0;
    if (millis() - last_heap_ms >= HEAP_LOG_INTERVAL_MS) {
        last_heap_ms = millis();
        uint32_t free_heap = esp_get_free_heap_size();
        Serial.printf("[APP] Free heap: %u bytes\n", free_heap);
        if (free_heap < HEAP_MIN_FREE_BYTES) {
            Serial.printf("[APP] Heap critically low (%u < %u) — restarting\n",
                          free_heap, HEAP_MIN_FREE_BYTES);
            delay(200);
            ESP.restart();
        }
    }

    delay(10);
}
