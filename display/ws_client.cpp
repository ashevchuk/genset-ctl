#include "ws_client.h"
#include "esp_now_client.h"
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

GenData g_gen = {};

// Protects g_gen reads/writes between the WS task (core 0) and loop() (core 1).
portMUX_TYPE ws_gen_mux = portMUX_INITIALIZER_UNLOCKED;

static WebSocketsClient wsClient;
static volatile bool     ws_connected       = false;
static char              s_host[64]         = {};
static uint16_t          s_port             = 0;
static TaskHandle_t      s_ws_task          = NULL;
static volatile bool     s_do_disconnect    = false;
static volatile bool     s_do_reconnect     = false;
static volatile uint32_t s_last_ws_rx_ms    = 0;
static volatile uint32_t s_ws_connected_ms  = 0;

// The controller reports at 1 Hz. A dead AP leaves the socket "connected"
// until the beacon/TCP timeouts fire several seconds later; this window
// is what lets ESP-NOW take over before that.
#define WS_FRESH_MS 2500

// WebSocketsClient is NOT thread-safe. wsClient.loop() runs in ws_task
// (core 0) while ws_send() is called from the LVGL task (UI button events)
// and begin()/disconnect() from loop() (core 1). All library calls must be
// serialized through this mutex. Created in ws_client_init() before the
// task starts.
static SemaphoreHandle_t ws_api_mutex = NULL;

static void onWSEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
    case WStype_CONNECTED:
        s_ws_connected_ms = millis();
        ws_connected = true;
        Serial.println("[WS] Connected");
        break;

    case WStype_DISCONNECTED:
        ws_connected = false;
        Serial.println("[WS] Disconnected");
        break;

    case WStype_TEXT: {
        StaticJsonDocument<2048> doc;
        if (deserializeJson(doc, payload, length) != DeserializationError::Ok) break;
        s_last_ws_rx_ms = millis();

        JsonObject sw  = doc["switch"];
        JsonObject val = doc["value"];
        JsonObject dt  = doc["date"];

        // Write g_gen under spinlock so loop() always sees a consistent snapshot.
        portENTER_CRITICAL(&ws_gen_mux);

        if (!sw.isNull()) {
            // The controller sends 1/0 (not true/false) in the "switch"
            // section to shave a few bytes per field. ArduinoJson's
            // operator|(bool) only returns the real value when the variant's
            // stored type is actually boolean (is<bool>()) - for a JSON
            // number it silently falls through to the default every time.
            // Read as int and coerce instead.
            g_gen.running        = (sw["run"]              | 0) != 0;
            g_gen.stopped        = (sw["stop"]             | 0) != 0;
            g_gen.choke          = (sw["Choke"]            | 0) != 0;
            g_gen.fuel_pump      = (sw["Fuel_pump"]        | 0) != 0;
            g_gen.eco_mode       = (sw["ECO"]              | 0) != 0;
            g_gen.load_relay     = (sw["Load_switch"]      | 0) != 0;
            g_gen.charger        = (sw["Charger"]          | 0) != 0;
            g_gen.ignition       = (sw["Ignition"]         | 0) != 0;
            g_gen.overload       = (sw["Overload"]         | 0) != 0;
            g_gen.oil            = (sw["Oil"]              | 0) != 0;
            g_gen.output_ok      = (sw["Output"]           | 0) != 0;
            g_gen.overheat       = (sw["Overheat"]         | 0) != 0;
            g_gen.psu_sense      = (sw["PSU_sense"]        | 0) != 0;
            g_gen.gen_sense      = (sw["Generator_sense"]  | 0) != 0;
            g_gen.bat_sense      = (sw["Battery_sense"]    | 0) != 0;
        }
        if (!val.isNull()) {
            g_gen.rpm            = val["RPM"]                      | 0.0f;
            g_gen.freq           = val["FREQUENCY"]                | 0.0f;
            g_gen.vrms           = val["VRMS"]                     | 0.0f;
            g_gen.irms           = val["IRMS"]                     | 0.0f;
            g_gen.power_active   = val["POWER_ACTIVE"]             | 0.0f;
            g_gen.power_apparent = val["POWER_APPARENT"]           | 0.0f;
            g_gen.power_reactive = val["POWER_REACTIVE"]           | 0.0f;
            g_gen.power_factor   = val["POWER_FACTOR"]             | 0.0f;
            g_gen.head_temp      = val["HEAD_TEMP"]                | 0.0f;
            g_gen.fuel_level     = val["ADC1_0"]                   | 0.0f;
            g_gen.oil_temp       = val["ADC0_0"]                   | 0.0f;
            g_gen.ambient_temp   = val["ADC0_1"]                   | 0.0f;
            g_gen.inverter_temp  = val["ADC0_2"]                   | 0.0f;
            g_gen.alternator_temp= val["ADC0_3"]                   | 0.0f;
            g_gen.bat_voltage    = val["ADC1_1"]                   | 0.0f;
            g_gen.core_voltage   = val["ADC1_2"]                   | 0.0f;
            g_gen.rssi           = val["RSSI"]                     | 0;
            g_gen.maint_due_mask = (uint8_t)(val["MAINT_DUE_MASK"] | 0);
            const char *st = val["STATE"] | "";
            strlcpy(g_gen.state, st, sizeof(g_gen.state));
        }
        if (!dt.isNull()) {
            g_gen.uptime = dt["UPTIME"] | (uint32_t)0;
        }
        g_gen.valid       = true;
        g_gen.last_update = millis();

        portEXIT_CRITICAL(&ws_gen_mux);
        break;
    }
    default:
        break;
    }
}

// WebSocket task — runs on core 0 so it never blocks the Arduino loop() on core 1.
// WiFiClient::connect() can block up to ~3 s per TCP attempt; isolating it here
// keeps the display, OTA, and Serial output fully responsive during reconnect.
static void ws_task_fn(void *arg) {
    for (;;) {
        if (xSemaphoreTake(ws_api_mutex, portMAX_DELAY) == pdTRUE) {
            if (s_do_disconnect) {
                s_do_disconnect = false;
                wsClient.disconnect();
            }
            if (s_do_reconnect) {
                s_do_reconnect = false;
                wsClient.disconnect();
                wsClient.setReconnectInterval(5000);
            }
            wsClient.loop();
            xSemaphoreGive(ws_api_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void ws_client_init(const char *host, uint16_t port) {
    strlcpy(s_host, host, sizeof(s_host));
    s_port = port;

    portENTER_CRITICAL(&ws_gen_mux);
    memset(&g_gen, 0, sizeof(g_gen));
    portEXIT_CRITICAL(&ws_gen_mux);

    // First call: create the mutex before the task exists — no race.
    if (ws_api_mutex == NULL) {
        ws_api_mutex = xSemaphoreCreateMutex();
    }

    // Re-init on WiFi reconnect: ws_task may be inside wsClient.loop().
    if (xSemaphoreTake(ws_api_mutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        wsClient.begin(host, port, "/ws");
        wsClient.onEvent(onWSEvent);
        wsClient.setReconnectInterval(5000);
        // Ping every 15 s; 2 missed pongs (3 s each) = dead link. Catches a
        // half-open socket (e.g. the controller rebooted) that otherwise
        // stays "connected" forever, since the panel rarely sends anything.
        wsClient.enableHeartbeat(15000, 3000, 2);
        xSemaphoreGive(ws_api_mutex);
        Serial.printf("[WS] Init: ws://%s:%d/ws\n", host, port);
    } else {
        Serial.println("[WS] Init: mutex timeout — skipped");
    }

    if (!s_ws_task) {
        xTaskCreatePinnedToCore(ws_task_fn, "ws_loop", 8192, NULL, 3, &s_ws_task, 0);
    }
}

void ws_client_disconnect(void) {
    s_do_disconnect = true;
    ws_connected = false;
}

void ws_client_reconnect(void) {
    Serial.printf("[WS] Reconnect requested: ws://%s:%d/ws\n", s_host, s_port);
    s_do_reconnect = true;
}

bool ws_client_connected(void) {
    return ws_connected;
}

uint32_t ws_client_silent_ms(void) {
    if (!ws_connected) return 0;
    uint32_t since = s_last_ws_rx_ms > s_ws_connected_ms ? s_last_ws_rx_ms : s_ws_connected_ms;
    return millis() - since;
}

bool ws_client_fresh(void) {
    return ws_connected && s_last_ws_rx_ms != 0 && millis() - s_last_ws_rx_ms < WS_FRESH_MS;
}

void ws_send(const char *msg) {
    // A connected-but-silent socket is usually a dead AP not yet detected —
    // writing into it loses the command, so prefer ESP-NOW then.
    if (espnow_client_enabled() && !ws_client_fresh()) {
        espnow_send_text(msg);
        return;
    }
    // Called from the LVGL task (UI buttons); serialize with ws_task's loop().
    if (!ws_connected || ws_api_mutex == NULL) return;
    if (xSemaphoreTake(ws_api_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (ws_connected) wsClient.sendTXT(msg);
        xSemaphoreGive(ws_api_mutex);
    } else {
        Serial.println("[WS] send skipped — mutex timeout");
    }
}

void ws_send_toggle(uint8_t reg) {
    char buf[4];
    snprintf(buf, sizeof(buf), "%d", reg);
    ws_send(buf);
}
