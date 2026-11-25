#pragma once

#include <stdint.h>
#include <stddef.h>
#include <ArduinoJson.h>

// MQTT publish/subscribe. Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp
// (see CLAUDE.md's "Code organization" and [[main-cpp-modularization]]
// project memory for the pattern this follows).

struct MqttConfig_t {
    bool enabled;
    char host[64];
    uint16_t port;
    char user[33];
    char password[65];
    char client_id[33];
    char topic_prefix[33];
};

// Defined (non-static) in mqtt.cpp — exposed so the main .cpp's config-apply
// path (wifi_config_apply()) can print it right after loading, same pattern
// as g_limits_config.
extern MqttConfig_t g_mqtt_config;

// Load MQTT config from a parsed wifi.json document. Called from the main
// .cpp's config-apply path at boot/config-save.
void mqtt_config_load(const JsonDocument &doc);

// MqttTask: connects, subscribes to {prefix}/cmd/#, publishes
// {prefix}/state (every 5s or on state change), {prefix}/event (journal
// mirror), {prefix}/metrics (every 60s). See MQTT.md for the wire format.
void MqttTask(void *pvParameters);
