/*
 * http_server.h — HTTP + WebSocket server on ESP-IDF's esp_http_server.
 *
 * Serves the dashboard pages and static files from FFat, the JSON/command
 * API, uploads and OTA, and the /ws WebSocket. Handlers run in the httpd
 * task (core 0, WebTask's priority), one request at a time.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <esp_http_server.h>
#include "multipart.h"

#define HTTP_SERVER_PORT 80

// Normal operation: every route plus /ws. Needs the RTOS objects created in
// setup_rtos(), so it is started from WebTask.
bool http_server_start(void);

// Reduced route set for update_mode() (JP1 jumper): /reset, /wifi-config,
// /api/wifi/save (WiFi fields only) and /update.
bool http_server_start_update_mode(void);

// Multipart upload sink with an abort notification for a part that never
// reached its closing boundary.
class UploadHandler : public MultipartHandler {
public:
    virtual void on_file_end_aborted() = 0;
};

// Streams the request body through a multipart parser. Returns false if the
// connection failed before the whole body arrived; `complete` tells whether
// the closing boundary was seen.
bool receive_multipart(httpd_req_t *req, UploadHandler &handler, bool &complete);

// Number of connected WebSocket clients (read from any task).
int http_ws_client_count(void);

// Queues `data` as a text frame to every WebSocket client; the send happens
// in the httpd task, so the caller never blocks on a slow client. Returns
// false if the previous broadcast is still going out or `len` is too large.
bool http_ws_broadcast(const char *data, size_t len);

// Drives the WebSocket ping/pong heartbeat; call once per second.
void http_ws_tick(void);
