/*
 * mqtt.cpp — MQTT publish/subscribe.
 *
 * Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code
 * organization" section and [[main-cpp-modularization]] project memory for
 * the pattern this follows). Moved verbatim.
 */

#include "mqtt.h"
#include "rtos_config.h"
#include "fram_log.h"
#include "shiftreg.h"
#include "smoothed_safe.h"
#include "debug.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>

// ---- externs into esp32-gen-ctl-ai-rtos.cpp's own globals ------------------
extern SmoothedSafe<double> adc_2_0;
extern SmoothedSafe<double> adc_2_1;
double fuel_level_from_adc(double adc_value);

MqttConfig_t g_mqtt_config = {
    .enabled = false,
    .host = "",
    .port = 1883,
    .user = "",
    .password = "",
    .client_id = "gen-ctl",
    .topic_prefix = "gen-ctl"
};

static WiFiClient g_mqtt_wifi_client;
static PubSubClient g_mqtt_client(g_mqtt_wifi_client);

// Load MQTT config from wifi.json document
void mqtt_config_load(const JsonDocument &doc) {
    JsonObjectConst mqtt = doc["mqtt"];
    if (mqtt.isNull()) return;
    g_mqtt_config.enabled = mqtt["enabled"] | false;
    strncpy(g_mqtt_config.host,         mqtt["host"]         | "",        sizeof(g_mqtt_config.host)         - 1);
    g_mqtt_config.port =                mqtt["port"]         | 1883;
    strncpy(g_mqtt_config.user,         mqtt["user"]         | "",        sizeof(g_mqtt_config.user)         - 1);
    strncpy(g_mqtt_config.password,     mqtt["password"]     | "",        sizeof(g_mqtt_config.password)     - 1);
    strncpy(g_mqtt_config.client_id,    mqtt["client_id"]    | "gen-ctl", sizeof(g_mqtt_config.client_id)    - 1);
    strncpy(g_mqtt_config.topic_prefix, mqtt["topic_prefix"] | "gen-ctl", sizeof(g_mqtt_config.topic_prefix) - 1);
}

// MQTT message callback — called from g_mqtt_client.loop() inside MqttTask
static void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    char msg[64] = {0};
    if (length >= sizeof(msg)) length = sizeof(msg) - 1;
    memcpy(msg, payload, length);

    // Build expected prefix: "{topic_prefix}/cmd/"
    char prefix_cmd[100];
    snprintf(prefix_cmd, sizeof(prefix_cmd), "%s/cmd/", g_mqtt_config.topic_prefix);
    int prefix_cmd_len = strlen(prefix_cmd);

    if (strncmp(topic, prefix_cmd, prefix_cmd_len) != 0) return;
    const char *action = topic + prefix_cmd_len;

    if (xWebCommandQueue == NULL) return;
    WebCommand_t web_cmd = {};
    web_cmd.source = FLOG_SRC_MQTT;

    if (strcmp(action, "maintenance") == 0) {
        // Payload: subtype name (oil, spark_plug, ...) or index 0-5
        int sub = fram_log_maint_parse(msg);
        if (sub >= 0) fram_log_maintenance((uint8_t)sub, "mqtt");
        return;
    }

    if (strcmp(action, "start") == 0) {
        web_cmd.type = WEB_CMD_START;
        xQueueSend(xWebCommandQueue, &web_cmd, 0);
    } else if (strcmp(action, "stop") == 0) {
        web_cmd.type = WEB_CMD_STOP;
        xQueueSend(xWebCommandQueue, &web_cmd, 0);
    } else if (strcmp(action, "start_retry") == 0) {
        web_cmd.type = WEB_CMD_START_RETRY;
        xQueueSend(xWebCommandQueue, &web_cmd, 0);
    } else {
        // Register commands: choke, fuel_pump, eco_mode, load_relay, charger
        static const struct { const char *name; uint8_t reg; } reg_map[] = {
            {"choke",      REG_CHOKE},
            {"fuel_pump",  REG_FUEL_PUMP},
            {"eco_mode",   REG_ECO_MODE},
            {"load_relay", REG_LOAD_RELAY},
            {"charger",    REG_CHARGER},
        };
        for (int i = 0; i < 5; i++) {
            if (strcmp(action, reg_map[i].name) == 0) {
                // Strict match against the documented protocol (MQTT.md: payload is
                // 0/1/toggle) instead of the old "first char != '0'" heuristic, which
                // read any unrecognized payload — "off", "false", "OFF", garbage — as
                // ON. That's backwards for a relay command: an unrecognized payload
                // must be a no-op, never a guessed action, especially for
                // load_relay/eco_mode/charger. Found live 2026-09-18.
                if (strcmp(msg, "toggle") == 0) {
                    web_cmd.type  = WEB_CMD_REG_TOGGLE;
                    web_cmd.reg   = reg_map[i].reg;
                    web_cmd.state = false;
                } else if (strcmp(msg, "1") == 0) {
                    web_cmd.type  = WEB_CMD_REG_SET;
                    web_cmd.reg   = reg_map[i].reg;
                    web_cmd.state = true;
                } else if (strcmp(msg, "0") == 0) {
                    web_cmd.type  = WEB_CMD_REG_SET;
                    web_cmd.reg   = reg_map[i].reg;
                    web_cmd.state = false;
                } else {
                    SDEBUG("[MQTT] cmd/", action, ": unrecognized payload \"", msg, "\" (want 0/1/toggle) — ignored");
                    break;
                }
                xQueueSend(xWebCommandQueue, &web_cmd, 0);
                break;
            }
        }
    }
}

// ----------------------------------------------------------------------------
// MqttTask: MQTT connection, subscribe to commands, publish state
// ----------------------------------------------------------------------------
void MqttTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset

    if (!g_mqtt_config.enabled || strlen(g_mqtt_config.host) == 0) {
        // MQTT disabled — idle task
        SLOG_PRINTLN("MqttTask: disabled");
        for (;;) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    SLOG_PRINTF("MqttTask: broker=%s:%u prefix=%s\n",
                  g_mqtt_config.host, g_mqtt_config.port,
                  g_mqtt_config.topic_prefix);

    g_mqtt_client.setServer(g_mqtt_config.host, g_mqtt_config.port);
    g_mqtt_client.setCallback(mqtt_callback);
    g_mqtt_client.setBufferSize(1024);  // /metrics JSON is ~650 bytes
    g_mqtt_client.setKeepAlive(15);
    // PubSubClient's connect()/loop() busy-wait on the socket for up to this
    // many seconds with no esp_task_wdt_reset() in between (default 15s —
    // exactly the TWDT timeout, a zero-margin race that trips the task
    // watchdog if the broker is ever a hair slow to answer). Keep well under
    // TASK_WDT_TIMEOUT_MS/1000 so a stuck socket always resolves first.
    g_mqtt_client.setSocketTimeout(5);

    // Per-chip unique client id: two controllers sharing one id knock each
    // other off the broker in an endless session-takeover loop (reconnect
    // storm → TCP socket churn → heap pressure).
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "%s-%06X", g_mqtt_config.client_id,
             (unsigned)((ESP.getEfuseMac() >> 24) & 0xFFFFFF));
    SLOG_PRINTF("MqttTask: client_id=%s\n", client_id);

    char sub_topic[80];
    snprintf(sub_topic, sizeof(sub_topic), "%s/cmd/#", g_mqtt_config.topic_prefix);

    char state_topic[80];
    snprintf(state_topic, sizeof(state_topic), "%s/state", g_mqtt_config.topic_prefix);

    char event_topic[80];
    snprintf(event_topic, sizeof(event_topic), "%s/event", g_mqtt_config.topic_prefix);

    char metrics_topic[80];
    snprintf(metrics_topic, sizeof(metrics_topic), "%s/metrics", g_mqtt_config.topic_prefix);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_publish_ms = 0;
    uint32_t last_metrics_ms = 0;
    uint32_t last_connect_ms = 0;
    state last_published_state = STATE_INIT;

    for (;;) {
        if (WiFi.status() == WL_CONNECTED) {
            if (!g_mqtt_client.connected()) {
                // Rate-limit reconnects. A broker that accepts us and then
                // drops the session (duplicate client id, ACL, restart) would
                // otherwise trigger a connect storm at loop rate — TCP socket
                // churn that fragments the heap.
                uint32_t now_conn = millis();
                if (last_connect_ms != 0 && now_conn - last_connect_ms < 3000) {
                    esp_task_wdt_reset();
                    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
                    continue;
                }
                last_connect_ms = now_conn;
                bool ok;
                if (strlen(g_mqtt_config.user) > 0) {
                    ok = g_mqtt_client.connect(client_id,
                                               g_mqtt_config.user,
                                               g_mqtt_config.password);
                } else {
                    ok = g_mqtt_client.connect(client_id);
                }
                if (ok) {
                    g_mqtt_client.subscribe(sub_topic);
                    SLOG_PRINTF("MQTT connected, subscribed to %s\n", sub_topic);
                    last_publish_ms = 0; // force immediate publish after reconnect
                } else {
                    SLOG_PRINTF("MQTT connect failed rc=%d, retry in 5s\n",
                                  g_mqtt_client.state());
                    esp_task_wdt_reset();
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    continue;
                }
            }

            // Process incoming messages
            g_mqtt_client.loop();

            // Publish state every 5 s or on state change
            uint32_t now_ms = millis();
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

            bool state_changed = (cur_state != last_published_state);
            if (state_changed || (now_ms - last_publish_ms >= 5000)) {
                last_published_state = cur_state;
                last_publish_ms = now_ms;

                char buf[512];
                int n = snprintf(buf, sizeof(buf),
                    "{"
                    "\"state\":\"%s\","
                    "\"state_id\":%d,"
                    "\"is_running\":%d,"
                    "\"rpm\":%lu,"
                    "\"freq\":%.2f,"
                    "\"bat_v\":%.2f,"
                    "\"fuel\":%.1f,"
                    "\"head_temp\":%.1f,"
                    "\"reg_choke\":%d,"
                    "\"reg_fuel_pump\":%d,"
                    "\"reg_eco_mode\":%d,"
                    "\"reg_load_relay\":%d,"
                    "\"reg_charger\":%d,"
                    "\"in_output\":%d,"
                    "\"in_gen_sense\":%d,"
                    "\"in_oil\":%d,"
                    "\"in_overload\":%d,"
                    "\"vrms\":%.1f,"
                    "\"power\":%.0f,"
                    "\"rssi\":%d,"
                    "\"run_hours\":%.2f,"
                    "\"starts\":%lu,"
                    "\"heap\":%u,"
                    "\"min_heap\":%u"
                    "}",
                    state_name(cur_state),
                    (int)cur_state,
                    (cur_state == STATE_RUNNING) ? 1 : 0,
                    (unsigned long)sensors.rpm,
                    power.frequency,
                    (double)adc_2_1.get(),
                    (double)fuel_level_from_adc(adc_2_0.get()),
                    sensors.head_temp,
                    (outputs & (1 << REG_CHOKE))      ? 1 : 0,
                    (outputs & (1 << REG_FUEL_PUMP))   ? 1 : 0,
                    (outputs & (1 << REG_ECO_MODE))    ? 1 : 0,
                    (outputs & (1 << REG_LOAD_RELAY))  ? 1 : 0,
                    (outputs & (1 << REG_CHARGER))     ? 1 : 0,
                    (inputs  & (1 << REG_IN_OUTPUT))   ? 1 : 0,
                    (inputs  & (1 << REG_IN_GEN_SENSE))? 1 : 0,
                    (inputs  & (1 << REG_IN_OIL))      ? 1 : 0,
                    (inputs  & (1 << REG_IN_OVERLOAD)) ? 1 : 0,
                    power.rms_voltage,
                    power.active_power,
                    WiFi.RSSI(),
                    (double)fram_metrics_get().total_run_seconds / 3600.0,
                    (unsigned long)fram_metrics_get().start_count,
                    (unsigned)ESP.getFreeHeap(),
                    (unsigned)ESP.getMinFreeHeap()
                );
                if (n > 0 && n < (int)sizeof(buf)) {
                    g_mqtt_client.publish(state_topic, buf, false);
                }
            }

            // Mirror journal events to {prefix}/event
            FramLogRecord ev;
            while (fram_log_pop_for_mqtt(&ev)) {
                char jbuf[256];
                if (fram_log_record_json(&ev, jbuf, sizeof(jbuf)) > 0) {
                    g_mqtt_client.publish(event_topic, jbuf, false);
                }
            }

            // Lifetime metrics every 60 s (retained: survives client restarts)
            if (now_ms - last_metrics_ms >= 60000) {
                last_metrics_ms = now_ms;
                char mbuf[768];
                if (fram_metrics_json(mbuf, sizeof(mbuf), g_limits_config.maint_interval_hours) > 0) {
                    g_mqtt_client.publish(metrics_topic, mbuf, true);
                }
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
