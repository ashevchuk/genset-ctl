#include "web.h"

#include <cstdlib>
#include <cstring>

#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_task_wdt.h>
#include <esp_heap_caps.h>
#include <Update.h>
#include <FFat.h>
#include <WiFi.h>

#include "rtos_config.h"
#include "fram_log.h"
#include "config.h"
#include "esp_now_link.h"  // ESPNOW_KEY_LEN
#include "gen_discovery.h"
#include "shiftreg.h"
#include "mcp39f511a_dev.h"
#include "powermeter.h"
#include "debug.h"
#include "bootloader_update.h"

extern WebServer webServer;
extern WebSockets4WebServer wsServer;
extern FS *filesystem;
extern File uploadFileWs;
extern SemaphoreHandle_t xWsMutex;

// Set true once setup_rtos() has created the web command queue — read here
// (not written) so the HTTP/WS handlers below enqueue onto xWebCommandQueue
// instead of the pre-RTOS fallback. Flipped before WebTask is created
// (see setup_rtos()); the fallback still refuses REG_START.
extern volatile bool g_rtos_running;

// State-machine entry points (still in the main .cpp) — the pre-RTOS
// fallback path in onWsServerEvent()/the HTTP routes below.
void start_working();
void start_retry_working();
void stop_working();

// wifi.json save (still in the main .cpp).
bool wifi_config_save(const JsonDocument &doc);

// Rate guard for the static-file handlers below. Kept from the Async* era
// as cheap defense-in-depth: WebServer::handleClient() naturally serializes
// requests one at a time via the TCP backlog rather than livelocking the
// way AsyncTCP's single _async_service_task did under a burst of parallel
// requests, but rejecting anything beyond a small rate per short window
// still costs nothing (no mutex wait, no file I/O) and keeps WebTask
// responsive to the rest of its own loop (wsServer.loop(), watchdog reset).
// 8, not 4: a single normal page load fetches ~6 assets (html+css+2-3 js+
// json) that a browser can fire close together.
#define STATIC_REQUEST_RATE_LIMIT 8
#define STATIC_REQUEST_RATE_WINDOW_MS 1000

// Whole frame must be a decimal register index. String::toInt() returns 0
// for any non-numeric text, and 0 is the choke register — an unrecognized
// frame used to toggle it.
static bool ws_parse_reg(const char *s, uint8_t *reg_out) {
  if (!s || s[0] < '0' || s[0] > '9') return false;
  char *end = nullptr;
  long v = strtol(s, &end, 10);
  if (end == s || *end != '\0') return false;
  if (v < 0 || v > 15) return false;
  *reg_out = (uint8_t)v;
  return true;
}

// ----------------------------------------------------------------------------
// onWsServerEvent: WebSocketsServer's onEvent callback (registered on
// `wsServer` by the main .cpp's setup()). Same command set as the HTTP
// /start, /stop, /reg routes below, just over a raw text frame instead of a
// query string.
// ----------------------------------------------------------------------------
void onWsServerEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    SLOG_PRINTF("WebSocket client #%u connected from %s - free=%u largest8bit=%u\n",
                  num, wsServer.remoteIP(num).toString().c_str(),
                  ESP.getFreeHeap(), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  } else if (type == WStype_DISCONNECTED) {
    SLOG_PRINTF("WebSocket client #%u disconnected - free=%u largest8bit=%u\n",
                  num, ESP.getFreeHeap(),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  } else if (type == WStype_TEXT) {
    // Copy into our own small stack buffer rather than mutating the
    // library's payload buffer in place (unlike AsyncWebSocket, which
    // guaranteed room for the old code's data[len]=0 trick).
    char buf[32];
    size_t n = (length < sizeof(buf) - 1) ? length : sizeof(buf) - 1;
    memcpy(buf, payload, n);
    buf[n] = 0;

    // Use RTOS queues when RTOS is running
    if (g_rtos_running && xWebCommandQueue != NULL) {
      WebCommand_t cmd = {};
      cmd.source = FLOG_SRC_WEBSOCKET;
      uint8_t reg = 0;

      if (strcmp(buf, "start") == 0) {
        cmd.type = WEB_CMD_START;
        cmd.reg = 0;
        xQueueSend(xWebCommandQueue, &cmd, 0);
      } else if (strcmp(buf, "start_retry") == 0) {
        cmd.type = WEB_CMD_START_RETRY;
        cmd.reg = 0;
        xQueueSend(xWebCommandQueue, &cmd, 0);
      } else if (strcmp(buf, "stop") == 0) {
        cmd.type = WEB_CMD_STOP;
        cmd.reg = 0;
        xQueueSend(xWebCommandQueue, &cmd, 0);
      } else if (ws_parse_reg(buf, &reg)) {
        cmd.type = WEB_CMD_REG_TOGGLE;
        cmd.reg = reg;
        xQueueSend(xWebCommandQueue, &cmd, 0);
      }
    } else {
      // Fallback to direct calls when RTOS not running. In practice this
      // path is unreachable for the new stack, since nothing calls
      // wsServer.loop() until WebTask exists, and WebTask is only created
      // after g_rtos_running is set — kept anyway for symmetry with the
      // old handler and in case that assumption ever changes.
      uint8_t reg = 0;
      if (strcmp(buf, "start") == 0) {
        start_working();
      } else if (strcmp(buf, "start_retry") == 0) {
        start_retry_working();
      } else if (strcmp(buf, "stop") == 0) {
        stop_working();
      } else if (ws_parse_reg(buf, &reg) && reg != REG_START) {
        bool reg_state = IS_REG_ON(reg);
        if (reg_state) {
          reg_set(reg, false);
        } else {
          reg_set(reg, true);
        }

        String message = "{\"switch\":{";

        for (int i = 0; i < ShiftRegListLength; i++) {
          if (reg == ShiftRegs[i].reg) {
            bool reg_state2 = IS_REG_ON(ShiftRegs[i].reg);

            message += "\"";
            message += ShiftRegs[i].name;
            message += "\":";
            message += reg_state2 ? "1" : "0";
            break;
          }
        }

        message += "}}";

        wsServer.broadcastTXT(message);
      }
    }
  }
}

static bool static_request_rate_exceeded(void) {
  static uint32_t window_start = 0;
  static uint8_t window_count = 0;
  uint32_t now = millis();
  if (now - window_start >= STATIC_REQUEST_RATE_WINDOW_MS) {
    window_start = now;
    window_count = 0;
  }
  window_count++;
  return window_count > STATIC_REQUEST_RATE_LIMIT;
}

// Same MIME-sniffing logic as the old onNotFound's /static/* handler,
// extracted once since the new static route (below) and its onNotFound
// both need it.
static String mime_type_for_path(const String &path) {
  String lowerPath = path;
  lowerPath.toLowerCase();
  if (lowerPath.endsWith(".gz")) {
    lowerPath = lowerPath.substring(0, lowerPath.length() - 3);
  }
  if (lowerPath.endsWith(".html") || lowerPath.endsWith(".htm")) return "text/html";
  if (lowerPath.endsWith(".css")) return "text/css";
  if (lowerPath.endsWith(".js")) return "application/javascript";
  if (lowerPath.endsWith(".json")) return "application/json";
  if (lowerPath.endsWith(".png")) return "image/png";
  if (lowerPath.endsWith(".jpg") || lowerPath.endsWith(".jpeg")) return "image/jpeg";
  if (lowerPath.endsWith(".gif")) return "image/gif";
  if (lowerPath.endsWith(".svg")) return "image/svg+xml";
  if (lowerPath.endsWith(".ico")) return "image/x-icon";
  if (lowerPath.endsWith(".txt")) return "text/plain";
  if (lowerPath.endsWith(".xml")) return "application/xml";
  if (lowerPath.endsWith(".pdf")) return "application/pdf";
  if (lowerPath.endsWith(".zip")) return "application/zip";
  return "application/octet-stream";
}

// Hand-rolled instead of WebServer::streamFile(): streamFile() reads the
// whole file in one internal call with no per-chunk hook, which would mean
// wrapping the entire transfer in xFlashI2CGuard (the whole-file-duration I2C
// starvation this session's Async* rewrite specifically fixed). Same
// per-chunk-release pattern as serve_static_html() above, just against
// WebServer's send()/sendContent() primitives instead of a response object.
void serve_static_ws(WebServer &srv, const char *filePath, const char *contentTypeOverride, bool cacheable) {
  if (static_request_rate_exceeded()) {
    srv.send(503, "text/plain", "Server busy, try again shortly");
    return;
  }

  String gzFilePath = String(filePath) + ".gz";
  bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
  File file = filesystem->open(gzFilePath, "r");
  bool useGzip = file && !file.isDirectory();
  if (!useGzip) {
    if (file) file.close();
    file = filesystem->open(filePath, "r");
  }
  if (!file || file.isDirectory()) {
    if (file) file.close();
    if (held) xSemaphoreGive(xFlashI2CGuard);
    srv.send(404, "text/plain", String("File not found: ") + filePath);
    return;
  }
  if (held) xSemaphoreGive(xFlashI2CGuard);

  size_t fileSize = file.size();
  String contentType = contentTypeOverride ? String(contentTypeOverride) : mime_type_for_path(filePath);
  srv.setContentLength(fileSize);
  if (useGzip) srv.sendHeader("Content-Encoding", "gzip");
  if (cacheable) srv.sendHeader("Cache-Control", "public, max-age=3600");
  srv.send(200, contentType.c_str(), "");

  uint8_t buf[512];
  size_t sent = 0;
  unsigned chunk = 0;
  while (sent < fileSize) {
    bool chunkHeld = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
    size_t n = file.read(buf, sizeof(buf));
    if (chunkHeld) xSemaphoreGive(xFlashI2CGuard);
    if (n == 0) break;
    srv.sendContent((const char *)buf, n);
    sent += n;
    // Large static assets (chart.js.gz etc.) + a slow TCP client can keep
    // handleClient() here for seconds — same WebTask TWDT hazard as /update.
    if ((++chunk & 7) == 0) esp_task_wdt_reset();
  }
  file.close();
}

// Equivalent of the old onNotFound's /static/* branch, registered directly
// as webServer's onNotFound (WebServer has no wildcard route matching, so
// this is the only way to catch arbitrary /static/<path> requests).
static void webServerNotFound(void) {
  String uri = webServer.uri();

  if (uri.startsWith("/static/")) {
    String path = uri.substring(8);
    if (path.indexOf("..") >= 0 || path.length() == 0) {
      webServer.send(403, "text/plain", "Forbidden");
      return;
    }
    String filePath = "/" + path;
    serve_static_ws(webServer, filePath.c_str(), mime_type_for_path(path).c_str(), true);
    return;
  }

  webServer.send(404, "text/plain", "Not found");
}

static const char *fw_ota_state_name(esp_ota_img_states_t state) {
  switch (state) {
    case ESP_OTA_IMG_NEW: return "new";
    case ESP_OTA_IMG_PENDING_VERIFY: return "pending_verify";
    case ESP_OTA_IMG_VALID: return "valid";
    case ESP_OTA_IMG_INVALID: return "invalid";
    case ESP_OTA_IMG_ABORTED: return "aborted";
    default: return "undefined";
  }
}

// Reads the esp_app_desc_t embedded in a partition's image directly from
// flash (no boot required) so the "other" slot's build date/time can be
// shown before the user commits to switching to it.
static void fw_append_partition_json(char *buf, size_t bufSize, size_t &off,
                                      const esp_partition_t *part, bool running) {
  esp_app_desc_t desc = {};
  bool haveDesc = (part != nullptr) && (esp_ota_get_partition_description(part, &desc) == ESP_OK);
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  if (part != nullptr) esp_ota_get_state_partition(part, &state);

  off += snprintf(buf + off, bufSize - off,
                   "{\"label\":\"%s\",\"running\":%s,\"state\":\"%s\","
                   "\"has_image\":%s,\"version\":\"%s\",\"date\":\"%s\",\"time\":\"%s\"}",
                   part ? part->label : "", running ? "true" : "false",
                   fw_ota_state_name(state), haveDesc ? "true" : "false",
                   haveDesc ? desc.version : "", haveDesc ? desc.date : "",
                   haveDesc ? desc.time : "");
}

static void fw_build_partitions_json(char *buf, size_t bufSize) {
  const esp_partition_t *running = esp_ota_get_running_partition();
  const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);

  size_t off = 0;
  off += snprintf(buf + off, bufSize - off, "{\"running\":");
  fw_append_partition_json(buf, bufSize, off, running, true);
  off += snprintf(buf + off, bufSize - off, ",\"other\":");
  fw_append_partition_json(buf, bufSize, off, other, false);
  snprintf(buf + off, bufSize - off, "}");
}

// Parses the WiFi mode/AP/STA-networks fields of the current HTTP request
// into a wifi.json document — shared by the normal /api/wifi/save route
// below and update_mode()'s own copy of that route in the main .cpp.
void parse_wifi_core_fields(JsonDocument &doc) {
  auto p = [&](const char *k) -> String {
    return webServer.hasArg(k) ? webServer.arg(k) : "";
  };
  doc["mode"] = p("mode");
  doc["hostname"] = p("hostname");
  JsonObject ap = doc["ap"].to<JsonObject>();
  ap["ssid"] = p("ap_ssid");
  ap["password"] = p("ap_pass");
  int ch = atoi(p("ap_ch").c_str());
  ap["channel"] = (ch >= 1 && ch <= 13) ? ch : 1;
  ap["hidden"] = (atoi(p("ap_hidden").c_str()) != 0);
  ap["max_conn"] = 4;
  ap["ip"] = "192.168.4.1";
  ap["gw"] = "192.168.4.1";
  ap["sn"] = "255.255.255.0";
  JsonObject sta = doc["sta"].to<JsonObject>();
  sta["dhcp"] = (atoi(p("sta_dhcp").c_str()) != 0);
  sta["ip"] = p("sta_ip");
  sta["gw"] = p("sta_gw");
  sta["sn"] = p("sta_sn");
  sta["dns1"] = p("sta_dns1");
  sta["dns2"] = p("sta_dns2");
  JsonArray nets = sta["networks"].to<JsonArray>();
  for (int i = 1; i <= WIFI_NETWORKS_MAX; i++) {
    String sk = "sta_ssid_" + String(i);
    String pk = "sta_pass_" + String(i);
    String s = p(sk.c_str());
    String pw = p(pk.c_str());
    // pw.length() >= 8 matches WPA2's own minimum — the recovery-mode copy
    // of this handler used to skip this check (only required s.length() >
    // 0), a real inconsistency found during the legacy-code audit; the
    // shared helper now applies the stricter check everywhere.
    if (s.length() > 0 && pw.length() >= 8) {
      JsonObject n = nets.add<JsonObject>();
      n["ssid"] = s;
      n["password"] = pw;
    }
  }
}

void setup_server_routes_ws(void) {
  // Not a webServer.on() route — a hook (WEBSERVER_HAS_HOOK, see
  // libraries/WebServer/'s own WebServer.h comments) that intercepts a
  // "GET /ws..." request before any header is even read off the socket,
  // hands the raw client to wsServer, and tells webServer to forget about
  // it. Registered first, deliberately: WebServer::_parseRequest() runs
  // every hook (in registration order) right after the request line is
  // parsed, before matching against any of the on() routes below.
  // The library hook accepts the socket (newClient + the GET line) without
  // any lock. Wrap it so that path takes xWsMutex, same as loop() and
  // broadcastTXT(). Non-/ws requests never touch the client list, so they
  // don't wait on the lock. If the lock times out, leave the upgrade to
  // the normal HTTP path (it 404s and the client retries) instead of
  // mutating the slot list unlocked.
  WebServer::HookFunction ws_hook = wsServer.hookForWebserver("/ws", onWsServerEvent);
  webServer.addHook([ws_hook](const String &method, const String &url, NetworkClient *tcpClient,
                              WebServer::ContentTypeFunction contentType) {
    if (!(method == "GET" && url.indexOf("/ws") == 0)) {
      return WebServer::CLIENT_REQUEST_CAN_CONTINUE;
    }
    struct Lock {
      bool held;
      Lock() : held(xWsMutex && TAKE_MUTEX(xWsMutex)) {}
      ~Lock() { if (held) GIVE_MUTEX(xWsMutex); }
    } lock;
    if (!lock.held) {
      SLOG_PRINTLN("[WS] xWsMutex timeout accepting client");
      return WebServer::CLIENT_REQUEST_CAN_CONTINUE;
    }
    return ws_hook(method, url, tcpClient, contentType);
  });

  // Ping/pong heartbeat — found live 2026-09-25 that WEBSOCKETS_SERVER_CLIENT_MAX
  // (16, see libraries/WebSockets/CMakeLists.txt — raised from the library's
  // own default of 5 the same day, see that file's own comment) can genuinely
  // run out: a single browser tab, over one
  // long enough session with a few real network hiccups, ended up holding 4
  // simultaneous established TCP connections to port 80 at once (confirmed via
  // `ss -tnp` on the client machine — not a theory) despite the page itself
  // only ever intending one. Without this, the server has zero proactive way to
  // notice a client that's still `tcp->connected()`-true but has gone silent —
  // it only reclaims a slot on the next write failure or once the OS-level TCP
  // stack itself eventually notices the peer is gone, which can take a long
  // time with no active traffic to trigger detection. This bounds that.
  // Verified live 2026-09-25 against a raw socket that completed the WS
  // handshake and then deliberately never sent a single pong: first ping
  // fires immediately (a fresh client's lastPing starts effectively at
  // "never"), and WebSockets.cpp's own handleHBTimeout() forces an immediate
  // retry ping right after each missed pong rather than waiting the full
  // pingInterval again — so a genuinely silent client gets disconnected
  // after pongTimeout*disconnectTimeoutCount = 5000*2 = 10s, not
  // pingInterval-scale. A client that DOES respond normally is pinged every
  // 15s as configured and never comes close to this path. A slot killed
  // this way used to keep pongTimeoutCount >= disconnectTimeoutCount, and
  // the next client on that slot was dropped at the first 5s check — before
  // any ping — which is how one tab piled up connections. newClient() and
  // clientDisconnect() now zero that counter. Comfortably clears
  // this project's own documented network jitter (post-reboot reachability
  // delay alone is 19-30s) while still reclaiming a truly stuck slot fast.
  wsServer.enableHeartbeat(15000, 5000, 2);

  webServer.on("/fw", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/fw.html");
  });
  webServer.on("/journal", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/journal.html");
  });
  webServer.on("/", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/index.html");
  });
  webServer.on("/wifi-config", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/wifi-config.html");
  });
  webServer.on("/files", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/fm.html");
  });
  // Browsers auto-request this on every page load regardless of any <link
  // rel="icon">; without an explicit route it falls through to
  // webServerNotFound() (which only catches /static/*) and 404s silently.
  // cacheable=true since it never changes at runtime.
  webServer.on("/favicon.ico", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/favicon.ico", "image/x-icon", true);
  });

  // Phase 3b: everything else, same xWebCommandQueue enqueue logic as the
  // Async* routes above, just against WebServer's request-object-free API
  // (state lives in webServer itself, not a per-request pointer).
  webServer.on("/start", http_method::HTTP_GET, []() {
    webServer.send(200, "text/plain", "OK");
    if (g_rtos_running && xWebCommandQueue != NULL) {
      WebCommand_t cmd = {};
      cmd.type = WEB_CMD_START;
      cmd.source = FLOG_SRC_HTTP;
      xQueueSend(xWebCommandQueue, &cmd, 0);
    } else {
      start_working();
    }
  });

  webServer.on("/start_retry", http_method::HTTP_GET, []() {
    webServer.send(200, "text/plain", "OK");
    if (g_rtos_running && xWebCommandQueue != NULL) {
      WebCommand_t cmd = {};
      cmd.type = WEB_CMD_START_RETRY;
      cmd.source = FLOG_SRC_HTTP;
      xQueueSend(xWebCommandQueue, &cmd, 0);
    } else {
      start_retry_working();
    }
  });

  webServer.on("/stop", http_method::HTTP_GET, []() {
    webServer.send(200, "text/plain", "OK");
    if (g_rtos_running && xWebCommandQueue != NULL) {
      WebCommand_t cmd = {};
      cmd.type = WEB_CMD_STOP;
      cmd.source = FLOG_SRC_HTTP;
      xQueueSend(xWebCommandQueue, &cmd, 0);
    } else {
      stop_working();
    }
  });

  webServer.on("/reg", http_method::HTTP_GET, []() {
    bool _set = true;
    if (webServer.hasArg("set")) {
      _set = (bool)webServer.arg("set").toInt();
    }
    if (webServer.hasArg("reg")) {
      int8_t reg = webServer.arg("reg").toInt();
      if (reg > -1 && reg < 16) {
        if (g_rtos_running && xWebCommandQueue != NULL) {
          WebCommand_t cmd = {};
          cmd.type = WEB_CMD_REG_SET;
          cmd.reg = (uint8_t)reg;
          cmd.state = _set;
          cmd.source = FLOG_SRC_HTTP;
          xQueueSend(xWebCommandQueue, &cmd, 0);
        } else if (reg == REG_START) {
          // Pre-RTOS /reg used to crank here with no state check. Queue
          // path is now live before WebTask starts; this is the backstop
          // if that flag/queue race ever returns. Found 2026-09-28.
        } else {
          bool reg_state = IS_REG_ON((uint8_t)reg);
          if (_set && !reg_state) {
            reg_set((uint8_t)reg, true);
          } else if (!_set && reg_state) {
            reg_set((uint8_t)reg, false);
          }
        }
      }
    }
    webServer.send(200, "text/plain", "OK");
  });

  webServer.on("/log/wipe", http_method::HTTP_GET, []() {
    if (!webServer.hasArg("confirm") || webServer.arg("confirm") != "1") {
      webServer.send(400, "application/json", "{\"error\":\"add confirm=1\"}");
      return;
    }
    bool metrics_too = webServer.hasArg("metrics") && webServer.arg("metrics") == "1";
    fram_log_wipe(metrics_too);
    webServer.send(200, "application/json", "{\"ok\":1}");
  });

  webServer.on("/energy/reset", http_method::HTTP_GET, []() {
    if (!webServer.hasArg("confirm") || webServer.arg("confirm") != "1") {
      webServer.send(400, "application/json", "{\"error\":\"add confirm=1\"}");
      return;
    }
    // PowerMeterTask holds xSerial2Mutex across a full measurement burst
    // (many 9600-baud register R/Ws) — routinely > MUTEX_TIMEOUT_MS (100).
    // TAKE_MUTEX therefore often fails here without ever talking to the chip;
    // found live 2026-09-29: /energy/reset → 500 in ~126 ms, counters unchanged.
    // Wait long enough to outlast one PowerMeter tick instead.
    bool got_lock = false;
    bool ok = false;
    if (xSerial2Mutex != NULL &&
        xSemaphoreTake(xSerial2Mutex, pdMS_TO_TICKS(3000)) == pdTRUE) {
      got_lock = true;
      ok = mcp39f511a.resetEnergyCounters();
      GIVE_MUTEX(xSerial2Mutex);
    }
    if (!got_lock) {
      webServer.send(503, "application/json", "{\"error\":\"uart busy\"}");
    } else if (ok) {
      // Drop the 30s PowerMeterTask energy cache / g_state so the next WS
      // broadcast shows zeros without waiting for the slow energy period.
      powermeter_note_energy_reset();
      webServer.send(200, "application/json", "{\"ok\":1}");
    } else {
      webServer.send(500, "application/json", "{\"error\":\"reset failed\"}");
    }
  });

  webServer.on("/log", http_method::HTTP_GET, []() {
    uint32_t newest = fram_log_newest_seq();
    uint32_t oldest = fram_log_oldest_seq();

    int count = 20;
    if (webServer.hasArg("count")) {
      count = webServer.arg("count").toInt();
      if (count < 1) count = 1;
      if (count > 50) count = 50;
    }
    // Optional filter group — same names as journal.html FILTERS / EV_META
    // ("engine"/"charge"/"alarm"/"maint"/"sys"). Empty or "all" = no filter.
    String group = webServer.hasArg("group") ? webServer.arg("group") : "";
    if (group == "all") group = "";
    bool filter = group.length() > 0;

    uint32_t from = newest;
    if (webServer.hasArg("before")) {
      uint32_t before = (uint32_t)webServer.arg("before").toInt();
      from = (before > 0) ? before - 1 : 0;
      if (from > newest) from = newest;
    }
    // Exclusive lower bound (live poll: only events newer than this seq).
    uint32_t after = 0;
    if (webServer.hasArg("after")) {
      after = (uint32_t)webServer.arg("after").toInt();
    }

    // Cap I2C reads per request so a sparse filter can't monopolize
    // WebTask/I2C for a full 4080-record walk in one HTTP handleClient().
    // Still pet the TWDT inside the loop (same pattern as /update and
    // /api/upload chunks, and fram wipe) — a generator controller must not
    // panic from an admin journal query even if a future change raises the
    // cap or a single read stalls toward the 500ms i2c_run_on_owner timeout.
    const int max_scan = filter ? 400 : count;
    int want = count;
    int scanned = 0;
    int matched = 0;
    uint32_t scanned_to = from;

    String out;
    out.reserve(count * 220 + 128);
    out += "{\"newest\":";
    out += String(newest);
    out += ",\"oldest\":";
    out += String(oldest);
    out += ",\"ts_now\":";
    out += String((uint32_t)time(NULL));
    out += ",\"group\":\"";
    out += filter ? group : "all";
    out += "\",\"events\":[";

    char jbuf[256];
    FramLogRecord rec;
    bool first = true;
    for (uint32_t seq = from; seq >= oldest && seq > after && want > 0 && scanned < max_scan; seq--) {
      scanned++;
      scanned_to = seq;
      if ((scanned & 7) == 0) esp_task_wdt_reset();
      if (!fram_log_read(seq, &rec)) continue;
      if (filter && strcmp(fram_log_group_name(rec.type), group.c_str()) != 0) continue;
      if (fram_log_record_json(&rec, jbuf, sizeof(jbuf)) <= 0) continue;
      if (!first) out += ",";
      out += jbuf;
      first = false;
      matched++;
      want--;
    }
    esp_task_wdt_reset();
    // more=true when older matching rows may still exist below scanned_to.
    bool hit_count = (matched >= count);
    bool hit_scan_cap = (scanned >= max_scan) && (scanned_to > oldest) && (scanned_to > after + 1);
    bool more = (scanned_to > oldest) && (scanned_to > after + 1) && (hit_count || hit_scan_cap);
    out += "],\"returned\":";
    out += String(matched);
    out += ",\"scanned\":";
    out += String(scanned);
    out += ",\"scanned_to\":";
    out += String(scanned > 0 ? scanned_to : 0);
    out += ",\"more\":";
    out += more ? "true" : "false";
    out += "}";
    webServer.send(200, "application/json", out);
  });

  webServer.on("/metrics/set", http_method::HTTP_GET, []() {
    if (!webServer.hasArg("confirm") || webServer.arg("confirm") != "1") {
      webServer.send(400, "application/json", "{\"error\":\"add confirm=1\"}");
      return;
    }

    FramMetricsPatch_t p = {};
    char err[80];
    auto take_f = [&](const char *name, uint32_t flag, float *dst) -> bool {
      if (!webServer.hasArg(name)) return true;
      float v = webServer.arg(name).toFloat();
      if (v < 0.0f) {
        snprintf(err, sizeof(err), "{\"error\":\"%s must be >= 0\"}", name);
        webServer.send(400, "application/json", err);
        return false;
      }
      p.fields |= flag;
      *dst = v;
      return true;
    };
    auto take_u = [&](const char *name, uint32_t flag, uint32_t *dst) -> bool {
      if (!webServer.hasArg(name)) return true;
      long v = webServer.arg(name).toInt();
      if (v < 0) {
        snprintf(err, sizeof(err), "{\"error\":\"%s must be >= 0\"}", name);
        webServer.send(400, "application/json", err);
        return false;
      }
      p.fields |= flag;
      *dst = (uint32_t)v;
      return true;
    };

    // hours is the original name; run_hours matches GET /metrics.
    if (webServer.hasArg("run_hours") || webServer.hasArg("hours")) {
      const char *name = webServer.hasArg("run_hours") ? "run_hours" : "hours";
      if (!take_f(name, FRAM_METRICS_SET_RUN_HOURS, &p.run_hours)) return;
    }
    if (!take_f("charge_hours", FRAM_METRICS_SET_CHARGE_HOURS, &p.charge_hours)) return;
    if (!take_u("starts", FRAM_METRICS_SET_STARTS, &p.starts)) return;
    if (!take_u("start_fails", FRAM_METRICS_SET_START_FAILS, &p.start_fails)) return;
    if (!take_u("emergencies", FRAM_METRICS_SET_EMERGENCIES, &p.emergencies)) return;
    if (!take_u("charge_cycles", FRAM_METRICS_SET_CHARGE_CYCLES, &p.charge_cycles)) return;
    if (!take_f("max_head_temp", FRAM_METRICS_SET_MAX_HEAD_TEMP, &p.max_head_temp)) return;
    if (!take_f("max_rpm", FRAM_METRICS_SET_MAX_RPM, &p.max_rpm)) return;
    if (!take_f("max_power", FRAM_METRICS_SET_MAX_POWER, &p.max_power)) return;

    if (p.fields == 0) {
      webServer.send(400, "application/json", "{\"error\":\"no fields\"}");
      return;
    }
    if (!fram_metrics_set(&p)) {
      webServer.send(400, "application/json", "{\"error\":\"set failed\"}");
      return;
    }
    char metrics[768];
    fram_metrics_json(metrics, sizeof(metrics), g_limits_config.maint_interval_hours);
    char buf[800];
    if (metrics[0] == '{') {
      snprintf(buf, sizeof(buf), "{\"ok\":1,%s", metrics + 1);
    } else {
      snprintf(buf, sizeof(buf), "{\"ok\":1}");
    }
    webServer.send(200, "application/json", buf);
  });

  webServer.on("/metrics", http_method::HTTP_GET, []() {
    char buf[768];
    fram_metrics_json(buf, sizeof(buf), g_limits_config.maint_interval_hours);
    webServer.send(200, "application/json", buf);
  });

  // Raw dump of the `coredump` partition (partitions.csv: 0xFF0000, 64KB).
  // ESP-IDF's coredump-to-flash is already compiled in and enabled by this
  // core/toolchain build (CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH=1, confirmed
  // in the precompiled esp32-libs sdkconfig) — it saves a FULL core dump
  // (every registered task's own backtrace + registers, not just a task
  // name) on any panic/abort, including a TWDT-triggered one, with zero
  // code of ours involved. It was just never retrievable before 2026-08-25;
  // FLOG_EV_TASK_WDT's `tasks` field (see fram_log.h) only ever captures a
  // task-name string — by design, there's no safe budget in that ISR
  // context to do real stack unwinding, so it will keep being exactly that
  // sparse no matter what. This endpoint is the actual upgrade: dumps the
  // whole partition raw (unused space reads as 0xFF/0x00 padding, which
  // every coredump reader already knows to ignore via the embedded
  // header+checksum) so it can be decoded externally:
  //   pip install esp-coredump          (one-time)
  //   curl http://<host>/coredump -o coredump.bin
  //   python -m esp_coredump info_corefile --core coredump.bin
  //       --core-format raw --prog build_archive/<matching-build>.elf
  // The .elf must be the exact one running when the crash happened — see
  // build_archive/ (compile.sh/compile_fast.sh) for why that now survives.
  webServer.on("/coredump", http_method::HTTP_GET, []() {
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, NULL);
    if (!part) {
      webServer.send(404, "text/plain", "No coredump partition in partitions.csv");
      return;
    }
    webServer.sendHeader("Content-Disposition", "attachment; filename=\"coredump.bin\"");
    webServer.setContentLength(part->size);
    webServer.send(200, "application/octet-stream", "");
    uint8_t buf[512];
    size_t sent = 0;
    unsigned chunk = 0;
    while (sent < part->size) {
      size_t n = (part->size - sent < sizeof(buf)) ? (part->size - sent) : sizeof(buf);
      bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
      esp_err_t err = esp_partition_read(part, sent, buf, n);
      if (held) xSemaphoreGive(xFlashI2CGuard);
      if (err != ESP_OK) break;
      webServer.sendContent((const char *)buf, n);
      sent += n;
      if ((++chunk & 7) == 0) esp_task_wdt_reset();
    }
  });

  webServer.on("/maintenance", http_method::HTTP_GET, []() {
    if (!webServer.hasArg("type")) {
      webServer.send(400, "application/json", "{\"error\":\"type required\"}");
      return;
    }
    int sub = fram_log_maint_parse(webServer.arg("type").c_str());
    if (sub < 0) {
      webServer.send(400, "application/json", "{\"error\":\"unknown type\"}");
      return;
    }
    String note_s;
    if (webServer.hasArg("note")) {
      note_s = webServer.arg("note");
    }
    bool ok = fram_log_maintenance((uint8_t)sub, note_s.c_str());
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"ok\":%d,\"type\":\"%s\"}",
             ok ? 1 : 0, fram_log_maint_name((uint8_t)sub));
    webServer.send(ok ? 200 : 503, "application/json", buf);
  });

  webServer.on("/reset", http_method::HTTP_GET, []() {
    webServer.send(200, "text/plain", "OK");
    yield();
    ESP.restart();
  });

  webServer.on("/api/wifi/config", http_method::HTTP_GET, []() {
    bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
    File f = filesystem->open(WIFI_JSON_PATH, "r");
    if (!f || f.isDirectory()) {
      if (f) f.close();
      if (held) xSemaphoreGive(xFlashI2CGuard);
      webServer.send(404, "application/json", "{\"error\":\"not found\"}");
      return;
    }
    size_t n = f.size();
    if (n > WIFI_JSON_SIZE) n = WIFI_JSON_SIZE;
    String buf;
    buf.reserve(n + 1);
    while (f.available() && buf.length() < n) buf += (char)f.read();
    f.close();
    if (held) xSemaphoreGive(xFlashI2CGuard);
    webServer.send(200, "application/json", buf);
  });

  webServer.on("/api/wifi/save", http_method::HTTP_POST, []() {
    JsonDocument doc;
    parse_wifi_core_fields(doc);
    auto p = [&](const char *k) -> String {
      return webServer.hasArg(k) ? webServer.arg(k) : "";
    };
    JsonObject mqtt_obj = doc["mqtt"].to<JsonObject>();
    mqtt_obj["enabled"] = (atoi(p("mqtt_enabled").c_str()) != 0);
    mqtt_obj["host"] = p("mqtt_host");
    int mqtt_port = atoi(p("mqtt_port").c_str());
    mqtt_obj["port"] = (mqtt_port > 0 && mqtt_port <= 65535) ? mqtt_port : 1883;
    mqtt_obj["user"] = p("mqtt_user");
    mqtt_obj["password"] = p("mqtt_pass");
    String mqtt_cid = p("mqtt_client_id");
    mqtt_obj["client_id"] = mqtt_cid.length() > 0 ? mqtt_cid : "gen-ctl";
    String mqtt_pfx = p("mqtt_prefix");
    mqtt_obj["topic_prefix"] = mqtt_pfx.length() > 0 ? mqtt_pfx : "gen-ctl";

    JsonObject limits_obj = doc["limits"].to<JsonObject>();
    limits_obj["temp_air_max"]      = atof(p("limit_temp_air").c_str());
    limits_obj["temp_oil_max"]      = atof(p("limit_temp_oil").c_str());
    limits_obj["temp_engine_max"]   = atof(p("limit_temp_engine").c_str());
    limits_obj["temp_inverter_max"] = atof(p("limit_temp_inverter").c_str());
    limits_obj["power_max"]         = atof(p("limit_power").c_str());
    limits_obj["auto_load_relay_on_run"] = (atoi(p("limit_auto_load_relay").c_str()) != 0);
    limits_obj["choke_cold_start_temp"]  = atof(p("limit_choke_cold_start").c_str());
    JsonObject maint_int_obj = limits_obj["maint_interval"].to<JsonObject>();
    for (uint8_t i = 0; i < FLOG_MAINT_TYPES; i++) {
        // "limit_maint_interval_" (21 chars) + "fuel_filter" (11 chars) = 32
        // chars exactly — a 32-byte buffer truncates it to 31 (drops the
        // trailing 'r'), so hasArg() never found the real POST field and
        // this type silently saved as 0 every time. Found live 2026-08-25.
        char key[48];
        snprintf(key, sizeof(key), "limit_maint_interval_%s", fram_log_maint_name(i));
        maint_int_obj[fram_log_maint_name(i)] = atof(p(key).c_str());
    }

    JsonObject timing_obj = doc["timing"].to<JsonObject>();
    int max_try = atoi(p("timing_max_start_try").c_str());
    timing_obj["max_start_try"]            = (max_try >= 1) ? max_try : 5;
    timing_obj["start_time_check_ms"]      = atol(p("timing_start_time_check").c_str());
    timing_obj["start_time_ms"]            = atol(p("timing_start_time").c_str());
    timing_obj["start_retry_time_ms"]      = atol(p("timing_start_retry_time").c_str());
    timing_obj["stopping_time_ms"]         = atol(p("timing_stopping_time").c_str());
    timing_obj["running_time_ms"]          = atol(p("timing_running_time").c_str());
    timing_obj["load_relay_stop_delay_ms"] = atol(p("timing_load_relay_stop_delay").c_str());
    timing_obj["after_run_period_ms"]      = atol(p("timing_after_run_period").c_str());

    JsonObject battery_obj = doc["battery"].to<JsonObject>();
    battery_obj["threshold_start_charge_v"] = atof(p("bat_threshold_start_charge").c_str());
    battery_obj["threshold_stop_charge_v"]  = atof(p("bat_threshold_stop_charge").c_str());
    battery_obj["threshold_starting_low_v"] = atof(p("bat_threshold_starting_low").c_str());
    battery_obj["threshold_shutdown_v"]     = atof(p("bat_threshold_shutdown").c_str());
    battery_obj["shutdown_persist_time_s"]  = atol(p("bat_shutdown_persist_time").c_str());
    battery_obj["start_charge_persist_time_s"] = atol(p("bat_start_charge_persist_time").c_str());
    battery_obj["read_period_ms"]           = atol(p("bat_read_period").c_str());
    battery_obj["read_starting_period_ms"]  = atol(p("bat_read_starting_period").c_str());
    battery_obj["read_running_period_ms"]   = atol(p("bat_read_running_period").c_str());
    battery_obj["read_after_run_period_ms"] = atol(p("bat_read_after_run_period").c_str());
    battery_obj["read_charging_period_ms"]  = atol(p("bat_read_charging_period").c_str());
    battery_obj["psu_loss_backup_enabled"]  = (atoi(p("bat_psu_loss_backup").c_str()) != 0);

    JsonObject engine_obj = doc["engine"].to<JsonObject>();
    engine_obj["rpm_max"]                = atol(p("engine_rpm_max").c_str());
    engine_obj["rpm_starting"]           = atol(p("engine_rpm_starting").c_str());
    engine_obj["rpm_running"]            = atol(p("engine_rpm_running").c_str());
    engine_obj["rpm_starting_threshold"] = atol(p("engine_rpm_starting_threshold").c_str());
    engine_obj["rpm_running_threshold"]  = atol(p("engine_rpm_running_threshold").c_str());
    engine_obj["fuel_pump_stop_level"]   = atof(p("engine_fuel_pump_stop_level").c_str());
    engine_obj["fuel_pump_no_rise_timeout_s"] = atol(p("engine_fuel_pump_no_rise_timeout").c_str());

    JsonObject time_obj = doc["time"].to<JsonObject>();
    time_obj["tz_offset_hours"]   = atoi(p("tz_offset_hours").c_str());
    time_obj["tz_offset_minutes"] = atoi(p("tz_offset_minutes").c_str());

    // A bad key would otherwise save fine and silently disable the link at
    // the next boot (espnow_config_load() fails closed) — refuse it here.
    bool espnow_enabled = atoi(p("espnow_enabled").c_str()) != 0;
    String espnow_key = p("espnow_key");
    espnow_key.trim();
    espnow_key.toLowerCase();
    bool espnow_key_ok = espnow_key.length() == ESPNOW_KEY_LEN * 2;
    for (size_t i = 0; espnow_key_ok && i < espnow_key.length(); i++)
      espnow_key_ok = isxdigit((unsigned char)espnow_key[i]);
    if (espnow_enabled && !espnow_key_ok) {
      webServer.send(400, "text/plain", "ESP-NOW key must be exactly 32 hex characters. Nothing was saved.");
      return;
    }
    JsonObject espnow_obj = doc["espnow"].to<JsonObject>();
    espnow_obj["enabled"] = espnow_enabled;
    espnow_obj["key"]     = espnow_key;

    if (!wifi_config_save(doc)) {
      webServer.send(500, "text/plain", "Save failed");
      return;
    }
    webServer.send(200, "text/plain", "OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP.restart();
  });

  webServer.on("/api/files", http_method::HTTP_GET, []() {
    String path = "/";
    if (webServer.hasArg("path")) {
      path = webServer.arg("path");
    }
    if (path.indexOf("..") >= 0) {
      webServer.send(403, "application/json", "{\"error\":\"Forbidden\"}");
      return;
    }
    if (!path.startsWith("/")) {
      path = "/" + path;
    }
    // Canonical form: no trailing slash except root. FFat/VFS is flaky about
    // "/js/" vs "/js" — forcing a slash here made directory opens fail
    // intermittently (UI then showed the new path with the previous listing).
    while (path.length() > 1 && path.endsWith("/")) {
      path.remove(path.length() - 1);
    }

    bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
    File dir = filesystem->open(path);
    if ((!dir || !dir.isDirectory()) && path != "/") {
      if (dir) dir.close();
      // Retry the other slash form — some FAT mounts accept only one.
      String alt = path + "/";
      dir = filesystem->open(alt);
    }
    if ((!dir || !dir.isDirectory()) && path != "/") {
      if (dir) dir.close();
      // Brief retry: a concurrent serve_static_ws()/upload can make a
      // momentarily-busy FFat open fail even for a real directory.
      vTaskDelay(pdMS_TO_TICKS(20));
      esp_task_wdt_reset();
      dir = filesystem->open(path);
    }
    if (!dir || !dir.isDirectory()) {
      if (dir) dir.close();
      if (held) xSemaphoreGive(xFlashI2CGuard);
      webServer.send(404, "application/json", "{\"error\":\"Directory not found\"}");
      return;
    }

    String json = "{\"path\":\"";
    json += path;
    json += "\",\"files\":[";
    bool first = true;
    unsigned n_entries = 0;
    File entry = dir.openNextFile();
    while (entry) {
      if (!first) json += ",";
      first = false;

      String entryName = entry.name();
      int lastSlash = entryName.lastIndexOf('/');
      if (lastSlash >= 0) entryName = entryName.substring(lastSlash + 1);

      String entryPath = (path == "/") ? (String("/") + entryName) : (path + "/" + entryName);

      json += "{\"name\":\"";
      json += entryName;
      json += "\",\"path\":\"";
      json += entryPath;
      json += "\",\"isDir\":";
      json += entry.isDirectory() ? "true" : "false";
      json += ",\"size\":";
      json += String(entry.size());
      json += "}";

      entry.close();
      entry = dir.openNextFile();
      if ((++n_entries & 15) == 0) esp_task_wdt_reset();
    }
    dir.close();
    if (held) xSemaphoreGive(xFlashI2CGuard);

    size_t totalBytes = FFat.totalBytes();
    size_t usedBytes = FFat.usedBytes();
    size_t freeBytes = totalBytes - usedBytes;

    json += "],\"fsinfo\":{";
    json += "\"total\":" + String(totalBytes) + ",";
    json += "\"used\":" + String(usedBytes) + ",";
    json += "\"free\":" + String(freeBytes);
    json += "}}";

    webServer.send(200, "application/json", json);
  });

  webServer.on("/api/mkdir", http_method::HTTP_POST, []() {
    String path = "/";
    if (webServer.hasArg("path")) path = webServer.arg("path");
    if (!path.endsWith("/")) path += "/";

    String dirName = "";
    if (webServer.hasArg("name")) dirName = webServer.arg("name");

    SLOG_PRINTF("Mkdir: received path='%s', name='%s'\n", path.c_str(), dirName.c_str());

    if (path.indexOf("..") >= 0 || dirName.indexOf("..") >= 0 || dirName.indexOf("/") >= 0) {
      webServer.send(403, "text/plain", "Forbidden");
      return;
    }

    String dirPath = path + dirName;
    bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
    bool ok = FFat.mkdir(dirPath);
    if (held) xSemaphoreGive(xFlashI2CGuard);

    if (ok) {
      webServer.send(200, "text/plain", "OK");
      SLOG_PRINTF("Directory created: %s\n", dirPath.c_str());
    } else {
      webServer.send(500, "text/plain", "Failed to create directory");
      SLOG_PRINTF("Failed to create directory: %s\n", dirPath.c_str());
    }
  });

  webServer.on("/api/delete", http_method::HTTP_DELETE, []() {
    String path = "";
    if (webServer.hasArg("path")) path = webServer.arg("path");

    if (path.indexOf("..") >= 0 || path.length() == 0) {
      webServer.send(403, "text/plain", "Forbidden");
      return;
    }
    if (!path.startsWith("/")) path = "/" + path;
    while (path.length() > 1 && path.endsWith("/")) {
      path.remove(path.length() - 1);
    }

    bool held0 = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);
    File file = filesystem->open(path);
    if (!file) {
      // Same /js vs /js/ quirk as /api/files.
      file = filesystem->open(path + "/");
    }
    if (!file) {
      if (held0) xSemaphoreGive(xFlashI2CGuard);
      webServer.send(404, "text/plain", "File not found");
      return;
    }
    bool isDir = file.isDirectory();
    file.close();

    bool success = isDir ? (FFat.rmdir(path) || FFat.rmdir(path + "/"))
                         : FFat.remove(path);
    if (held0) xSemaphoreGive(xFlashI2CGuard);

    if (success) {
      webServer.send(200, "text/plain", "OK");
      SLOG_PRINTF("Deleted: %s\n", path.c_str());
    } else {
      webServer.send(500, "text/plain", "Failed to delete");
      SLOG_PRINTF("Failed to delete: %s\n", path.c_str());
    }
  });

  // Phase 3c: /api/upload via HTTPUpload status machine. Path comes from the
  // query string of THIS request (not a global set by a prior request) —
  // same reasoning as the Async* version's comment: reading it fresh every
  // call avoids the file landing in the wrong directory if two uploads
  // interleave. UPLOAD_FILE_ABORTED is new here (no Async* equivalent) —
  // cleans up a partial file instead of leaving a truncated one behind.
  webServer.on(
      "/api/upload", http_method::HTTP_POST,
      []() {
        webServer.send(200, "text/plain", "OK");
      },
      []() {
        HTTPUpload &upload = webServer.upload();

        String currentPath = "/";
        if (webServer.hasArg("path")) {
          currentPath = webServer.arg("path");
        }
        if (!currentPath.endsWith("/") && currentPath != "/") {
          currentPath += "/";
        }
        if (currentPath.indexOf("..") >= 0) {
          currentPath = "/";
        }

        // Same spi_flash_guard_start()/xFlashI2CGuard collision guard as every
        // other FFat access in this file (see the comment on the Async*
        // /api/upload handler above).
        bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);

        if (upload.status == UPLOAD_FILE_START) {
          String filePath = currentPath + upload.filename;
          SLOG_PRINTF("Upload: Opening file: %s\n", filePath.c_str());
          uploadFileWs = filesystem->open(filePath, "w");
          if (!uploadFileWs) {
            SLOG_PRINTF("Upload: Failed to open file: %s\n", filePath.c_str());
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (uploadFileWs) {
            size_t written = uploadFileWs.write(upload.buf, upload.currentSize);
            if (written != upload.currentSize) {
              SLOG_PRINTF("Upload: Failed to write chunk\n");
            }
          }
          // WebServer::handleClient() blocks synchronously for the entire
          // upload, so WebTask's own esp_task_wdt_reset() (after
          // handleClient() returns) never runs until the whole file is
          // done — a large enough file starves the 15s TWDT mid-transfer
          // (confirmed live: FLOG_EV_TASK_WDT during a ~1.3MB /update test).
          // Pet it per chunk instead.
          esp_task_wdt_reset();
        } else if (upload.status == UPLOAD_FILE_END) {
          if (uploadFileWs) {
            uploadFileWs.close();
            SLOG_PRINTF("Upload: File received complete\n");
          } else {
            SLOG_PRINTF("Upload: File failed\n");
          }
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          if (uploadFileWs) {
            String filePath = currentPath + upload.filename;
            uploadFileWs.close();
            filesystem->remove(filePath);
            SLOG_PRINTF("Upload: Aborted, removed partial file: %s\n", filePath.c_str());
          }
        }

        if (held) xSemaphoreGive(xFlashI2CGuard);
        vTaskDelay(pdMS_TO_TICKS(1));
      });

  // Phase 3c: /update via HTTPUpload status machine. Per the plan's scope
  // decision #4: this closes a pre-existing gap (the Async* /update route
  // never guarded Update.begin/write/end with xFlashI2CGuard, unlike every other
  // flash-writing route in this file) since the code was being rewritten
  // anyway.
  webServer.on(
      "/update", http_method::HTTP_POST,
      []() {
        webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP.restart();
      },
      []() {
        HTTPUpload &upload = webServer.upload();
        bool held = (xSemaphoreTake(xFlashI2CGuard, pdMS_TO_TICKS(600)) == pdTRUE);

        if (upload.status == UPLOAD_FILE_START) {
          Serial.setDebugOutput(true);
          SLOG_PRINTF("Update: %s\n", upload.filename.c_str());
          if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
          }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
          if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
          }
          // See the matching comment in /api/upload above — same
          // blocking-handleClient()-starves-TWDT hazard applies here too.
          esp_task_wdt_reset();
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true)) {
            SLOG_PRINTF("Update Success: %u\nRebooting...\n", (unsigned)upload.totalSize);
            FramLogRecord r = {};
            r.type = FLOG_EV_FW_UPDATE;
            r.d.fw_update.size = upload.totalSize;
            // Update.end(true) already flipped the boot pointer to the
            // partition we just wrote, so "next update partition" (relative
            // to the still-running old one) is that just-written image —
            // read its version before we reboot into it.
            const esp_partition_t *justWritten = esp_ota_get_next_update_partition(NULL);
            esp_app_desc_t desc = {};
            if (justWritten && esp_ota_get_partition_description(justWritten, &desc) == ESP_OK) {
              strlcpy(r.d.fw_update.version, desc.version, sizeof(r.d.fw_update.version));
            }
            fram_log_event(&r);
          } else {
            Update.printError(Serial);
          }
          Serial.setDebugOutput(false);
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          Update.end(false);
          Serial.setDebugOutput(false);
        }

        if (held) xSemaphoreGive(xFlashI2CGuard);
      });

#if BOOTLOADER_UPDATE_ENDPOINT_ENABLED
  // Temporary, single-use capability — see bootloader_update.h for the full
  // rationale. Off (0) in every normal build.
  setup_bootloader_update_route();
#endif

  webServer.on("/api/fw/partitions", http_method::HTTP_GET, []() {
    char buf[512];
    fw_build_partitions_json(buf, sizeof(buf));
    webServer.send(200, "application/json", buf);
  });

  webServer.on("/api/fw/switch-slot", http_method::HTTP_GET, []() {
    if (!webServer.hasArg("confirm") || webServer.arg("confirm") != "1") {
      webServer.send(400, "application/json", "{\"error\":\"add confirm=1\"}");
      return;
    }
    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
      webServer.send(500, "application/json", "{\"error\":\"no alternate OTA partition\"}");
      return;
    }
    esp_app_desc_t desc = {};
    if (esp_ota_get_partition_description(target, &desc) != ESP_OK) {
      webServer.send(400, "application/json", "{\"error\":\"alternate slot has no valid image\"}");
      return;
    }
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    esp_ota_get_state_partition(target, &state);
    if (state == ESP_OTA_IMG_INVALID || state == ESP_OTA_IMG_ABORTED) {
      webServer.send(400, "application/json", "{\"error\":\"alternate slot marked invalid, refusing\"}");
      return;
    }
    if (esp_ota_set_boot_partition(target) != ESP_OK) {
      webServer.send(500, "application/json", "{\"error\":\"failed to set boot partition\"}");
      return;
    }
    webServer.send(200, "application/json", "{\"ok\":1,\"rebooting\":1}");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP.restart();
  });

  webServer.onNotFound(webServerNotFound);
}

void WebTask(void *pvParameters) {
    (void)pvParameters;
    esp_task_wdt_add(NULL);  // self-register before first wdt_reset

    for (;;) {
        webServer.handleClient();
        if (TAKE_MUTEX(xWsMutex)) {
            wsServer.loop();
            GIVE_MUTEX(xWsMutex);
        }
        gen_discovery_poll();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}
