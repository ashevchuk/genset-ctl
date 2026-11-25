#include "update_mode.h"
#include <esp_system.h>

#include "wifi_net.h"
#include <mdns.h>
#include "ffat.h"

#include "rtos_config.h"
#include "shiftreg.h"
#include "oled_dev.h"
#include "http_server.h"
#include "config.h"
#include "debug.h"

// Fallback AP credentials this recovery mode connects to — used nowhere
// else, so no reason for these to be visible outside this file.
static const char *wifi_fb_host = "gen-ctl";
static const char *wifi_fb_ssid = "xxxxxxxxxxx";
static const char *wifi_fb_password = "xxxxxxxxxxx";

void update_mode(void) {
  REG_ON(REG_LED1);
  oled.clear();

  oled.println("Update mode");

  wifi_net_start_sta();
  wifi_net_connect(wifi_fb_ssid, wifi_fb_password);

  oled.print("Connecting: ");
  oled.println(wifi_fb_ssid);

  for (uint8_t i = 0; i < 5; i++) {
    if (wifi_net_wait_connected(60000)) {
      break;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    SLOG_PRINT(".");
    oled.print(".");
  }

  if (!wifi_net_connected()) {
    SLOG_PRINTLN("WiFi Failed");
    oled.clear();
    oled.println("fail");
    REG_ON(REG_BUZZER);
    vTaskDelay(pdMS_TO_TICKS(10));
    REG_OFF(REG_BUZZER);
    REG_OFF(REG_LED0);
    esp_restart();
  } else {

    wifi_net_set_auto_reconnect(true);

    oled.println("");

    oled.println("Connected");
    vTaskDelay(pdMS_TO_TICKS(500));

    oled.clear();

    oled.println("Update mode...");
  }

  // Already initialised if the got-IP hook started it; only the name matters.
  mdns_init();
  mdns_hostname_set(wifi_fb_host);

  // Mount FFat so wifi-config can read/write wifi.json
  if (!ffat_mount()) {
    SLOG_PRINTLN("[update_mode] FFat mount failed — wifi-config save unavailable");
  }

  if (!http_server_start_update_mode()) {
    SLOG_PRINTLN("[update_mode] http_server start failed");
  }
  mdns_service_add(nullptr, "_http", "_tcp", 80, nullptr, 0);

  // Requests are served by the httpd task; this task only stays alive.
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}
