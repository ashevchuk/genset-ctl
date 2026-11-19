#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <WebSockets4WebServer.h>
#include <ArduinoJson.h>

// WebTask: HTTP (WebServer) + WebSocket (WebSockets4WebServer, at /ws) —
// both on port 80, see CLAUDE.md's Web/MQTT interfaces section — the entire
// always-on HTTP/WS API surface (page serving, start/stop/reg commands,
// FRAM log/metrics/maintenance/coredump, wifi-config API, file manager API,
// OTA upload, firmware-partition management) plus the WS text-command
// dispatch (onWsServerEvent). Extracted 2026-09-24 from
// esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's "Code organization" and
// [[main-cpp-modularization]] project memory for the pattern this follows).
//
// NOT included here: update_mode() (a separate, pre-RTOS-boot OTA-only web
// server with its own minimal route set — its own module, update_mode.cpp/
// .h, since 2026-09-24) and the esp_app_desc_t firmware-image-descriptor
// override (a compile-time image property, unrelated to request serving,
// still in the main .cpp).
void WebTask(void *pvParameters);

// Registers every normal-operation HTTP route on the global `webServer`.
// Called once from the main .cpp's setup().
void setup_server_routes_ws(void);

// WebSockets4WebServer's onEvent callback — dispatches WS text commands
// ("start"/"stop"/a register number) the same way the HTTP /start, /stop,
// /reg routes do. Registered via setup_server_routes_ws()'s own
// webServer.addHook() around hookForWebserver(), so accept takes xWsMutex.
void onWsServerEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);

// Chunked static-file server (FFat, gzip-aware) used by every page route
// above and by update_mode.cpp's own reduced route set.
void serve_static_ws(WebServer &srv, const char *filePath, const char *contentTypeOverride = nullptr, bool cacheable = false);

// Parses the WiFi mode/AP/STA-networks fields of the current HTTP request
// (webServer.arg()) into a wifi.json document — shared by the normal
// /api/wifi/save route here and update_mode.cpp's own copy of that route.
void parse_wifi_core_fields(JsonDocument &doc);
