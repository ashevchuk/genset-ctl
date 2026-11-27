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
#include "wifi_net.h"
#include <mqtt_client.h>
#include <esp_mac.h>
#include <esp_heap_caps.h>
#include <esp_task_wdt.h>
#include "uptime.h"

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

static esp_mqtt_client_handle_t s_client = NULL;
static volatile bool s_connected = false;
static volatile bool s_publish_now = false;  // set on (re)connect: publish state right away
static char s_sub_topic[80];

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

// MQTT message callback — called from the esp-mqtt client task
static void mqtt_callback(const char *topic, const uint8_t *payload, unsigned int length) {
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

static void mqtt_event_handler(void *arg, esp_event_base_t base, int32_t event_id, void *event_data) {
    (void)arg;
    (void)base;
    esp_mqtt_event_handle_t ev = (esp_mqtt_event_handle_t)event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        esp_mqtt_client_subscribe(ev->client, s_sub_topic, 0);
        SLOG_PRINTF("MQTT connected, subscribed to %s\n", s_sub_topic);
        s_connected = true;
        s_publish_now = true;
        break;
    case MQTT_EVENT_DISCONNECTED:
        if (s_connected) SLOG_PRINTLN("MQTT disconnected, retry in 5s");
        s_connected = false;
        break;
    case MQTT_EVENT_ERROR:
        if (ev->error_handle != NULL && ev->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            SLOG_PRINTF("MQTT connect refused rc=%d, retry in 5s\n",
                        (int)ev->error_handle->connect_return_code);
        }
        break;
    case MQTT_EVENT_DATA: {
        // Commands are a few bytes; a message split across several events
        // would only be a malformed one, so only whole messages are handled.
        if (ev->current_data_offset != 0 || ev->data_len != ev->total_data_len) break;
        char topic[128];
        if (ev->topic_len <= 0 || ev->topic_len >= (int)sizeof(topic)) break;
        memcpy(topic, ev->topic, ev->topic_len);
        topic[ev->topic_len] = '\0';
        mqtt_callback(topic, (const uint8_t *)ev->data, (unsigned int)ev->data_len);
        break;
    }
    default:
        break;
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

    // Per-chip unique client id: two controllers sharing one id knock each
    // other off the broker in an endless session-takeover loop (reconnect
    // storm → TCP socket churn → heap pressure). Suffix is the last three MAC
    // bytes, low byte first, as earlier firmware formatted it.
    static char client_id[64];
    uint8_t mac[6] = {};
    esp_efuse_mac_get_default(mac);
    snprintf(client_id, sizeof(client_id), "%s-%06X", g_mqtt_config.client_id,
             (unsigned)(mac[3] | (mac[4] << 8) | (mac[5] << 16)));
    SLOG_PRINTF("MqttTask: client_id=%s\n", client_id);

    snprintf(s_sub_topic, sizeof(s_sub_topic), "%s/cmd/#", g_mqtt_config.topic_prefix);

    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.broker.address.hostname = g_mqtt_config.host;
    mqtt_cfg.broker.address.port = g_mqtt_config.port;
    mqtt_cfg.broker.address.transport = MQTT_TRANSPORT_OVER_TCP;
    mqtt_cfg.credentials.client_id = client_id;
    if (strlen(g_mqtt_config.user) > 0) {
        mqtt_cfg.credentials.username = g_mqtt_config.user;
        mqtt_cfg.credentials.authentication.password = g_mqtt_config.password;
    }
    mqtt_cfg.session.keepalive = 15;
    mqtt_cfg.buffer.size = 1024;  // /metrics JSON is ~650 bytes
    // Bounds each blocking publish from this task, well under the 15s TWDT
    // window, so a stalled broker socket always resolves first.
    mqtt_cfg.network.timeout_ms = 5000;
    mqtt_cfg.network.reconnect_timeout_ms = 5000;
    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_client == NULL) {
        SLOG_PRINTLN("MqttTask: client init failed");
        for (;;) {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    esp_mqtt_client_register_event(s_client, MQTT_EVENT_ANY, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);

    char state_topic[80];
    snprintf(state_topic, sizeof(state_topic), "%s/state", g_mqtt_config.topic_prefix);

    char event_topic[80];
    snprintf(event_topic, sizeof(event_topic), "%s/event", g_mqtt_config.topic_prefix);

    char metrics_topic[80];
    snprintf(metrics_topic, sizeof(metrics_topic), "%s/metrics", g_mqtt_config.topic_prefix);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t last_publish_ms = 0;
    uint32_t last_metrics_ms = 0;
    state last_published_state = STATE_INIT;

    for (;;) {
        // Reconnects (5s apart) and incoming commands are handled by the
        // esp-mqtt client task; this loop only publishes while connected.
        if (s_connected) {
            if (s_publish_now) {
                s_publish_now = false;
                last_publish_ms = 0;
            }

            // Publish state every 5 s or on state change
            uint32_t now_ms = uptime_ms();
            state cur_state;
            SensorData_t sensors;
            PowerData_t power;
            uint8_t inputs;
            uint16_t outputs;
            uint16_t cooldown_left;
            portENTER_CRITICAL(&g_state_spinlock);
            cur_state = g_state.current_state;
            cooldown_left = g_state.cooldown_left_s;
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
                    "\"cooldown_left\":%u,"
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
                    (unsigned)cooldown_left,
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
                    (int)wifi_net_rssi(),
                    (double)fram_metrics_get().total_run_seconds / 3600.0,
                    (unsigned long)fram_metrics_get().start_count,
                    (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                    (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)
                );
                if (n > 0 && n < (int)sizeof(buf)) {
                    esp_mqtt_client_publish(s_client, state_topic, buf, 0, 0, 0);
                }
            }

            // Mirror journal events to {prefix}/event
            FramLogRecord ev;
            while (fram_log_pop_for_mqtt(&ev)) {
                char jbuf[256];
                if (fram_log_record_json(&ev, jbuf, sizeof(jbuf)) > 0) {
                    esp_mqtt_client_publish(s_client, event_topic, jbuf, 0, 0, 0);
                }
            }

            // Lifetime metrics every 60 s (retained: survives client restarts)
            if (now_ms - last_metrics_ms >= 60000) {
                last_metrics_ms = now_ms;
                char mbuf[768];
                if (fram_metrics_json(mbuf, sizeof(mbuf), g_limits_config.maint_interval_hours) > 0) {
                    esp_mqtt_client_publish(s_client, metrics_topic, mbuf, 0, 0, 1);
                }
            }
        }

        esp_task_wdt_reset();
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}
