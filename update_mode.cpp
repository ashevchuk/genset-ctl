#include "update_mode.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <FFat.h>
#include <Update.h>
#include <ArduinoJson.h>

#include "rtos_config.h"
#include "shiftreg.h"
#include "oled_dev.h"
#include "web.h"
#include "config.h"
#include "debug.h"

extern WebServer webServer;

// Still defined (non-static) in the main .cpp — update_mode()'s own
// reduced /api/wifi/save route reuses it rather than duplicating the save
// logic.
bool wifi_config_save(const JsonDocument &doc);

// Fallback AP credentials this recovery mode connects to — used nowhere
// else, so no reason for these to be visible outside this file.
static const char *wifi_fb_host = "gen-ctl";
static const char *wifi_fb_ssid = "xxxxxxxxxxx";
static const char *wifi_fb_password = "xxxxxxxxxxx";

void update_mode(void) {
  REG_ON(REG_LED1);
  oled.clear();

  oled.println("Update mode");

  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(wifi_fb_ssid, wifi_fb_password);

  oled.print("Connecting: ");
  oled.println(wifi_fb_ssid);

  for (uint8_t i = 0; i < 5; i++) {
    if (WiFi.waitForConnectResult(60000) == WL_CONNECTED) {
      break;
    }

    delay(100);
    SLOG_PRINT(".");
    oled.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    SLOG_PRINTLN("WiFi Failed");
    oled.clear();
    oled.println("fail");
    REG_ON(REG_BUZZER);
    delay(10);
    REG_OFF(REG_BUZZER);
    REG_OFF(REG_LED0);
    ESP.restart();
  } else {

    WiFi.setAutoReconnect(true);

    oled.println("");

    //  if (WiFi.waitForConnectResult() == WL_CONNECTED) {
    oled.println("Connected");
    delay(500);

    oled.clear();

    oled.println("Update mode...");
  }

  MDNS.begin(wifi_fb_host);

  // Mount FFat so wifi-config can read/write wifi.json
  if (!FFat.begin(true)) {
    SLOG_PRINTLN("[update_mode] FFat mount failed — wifi-config save unavailable");
  }

  webServer.on("/reset", http_method::HTTP_GET, []() {
    webServer.send(200, "text/plain", "OK");
    ESP.restart();
  });

  webServer.on("/wifi-config", http_method::HTTP_GET, []() {
    serve_static_ws(webServer, "/wifi-config.html");
  });

  webServer.on("/api/wifi/save", http_method::HTTP_POST, []() {
    JsonDocument doc;
    parse_wifi_core_fields(doc);
    if (!wifi_config_save(doc)) {
      webServer.send(500, "text/plain", "Save failed");
      return;
    }
    webServer.send(200, "text/plain", "OK. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP.restart();
  });

  webServer.on(
      "/update", http_method::HTTP_POST,
      []() {
        webServer.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
        REG_ON(REG_BUZZER);
        delay(100);
        REG_OFF(REG_BUZZER);
        delay(50);
        REG_ON(REG_BUZZER);
        delay(100);
        REG_OFF(REG_BUZZER);
        oled.println("Updated");
        ESP.restart();
      },
      []() {
        HTTPUpload &upload = webServer.upload();
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
          // No RTOS/TWDT in this code path (update_mode() runs before
          // setup_rtos()), so no watchdog-starvation risk here — unlike the
          // main-operation /update route, this loop just needs to keep
          // spinning (see the for(;;) below), which handleClient() being
          // slow doesn't threaten.
        } else if (upload.status == UPLOAD_FILE_END) {
          if (Update.end(true)) {
            SLOG_PRINTF("Update Success: %u\nRebooting...\n", (unsigned)upload.totalSize);
          } else {
            Update.printError(Serial);
          }
          Serial.setDebugOutput(false);
        } else if (upload.status == UPLOAD_FILE_ABORTED) {
          Update.end(false);
          Serial.setDebugOutput(false);
        }
      });

  webServer.begin();
  MDNS.addService("http", "tcp", 80);

  for (;;) {
    REG_ON(REG_LED1);
    webServer.handleClient();
    yield();
    REG_OFF(REG_LED1);
  }
}
