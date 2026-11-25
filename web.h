#pragma once

// WebTask: starts the HTTP + WebSocket server (http_server.h, served from
// the httpd task) once the RTOS objects its handlers need exist, then polls
// LAN UDP discovery (gen_discovery.h).
void WebTask(void *pvParameters);
