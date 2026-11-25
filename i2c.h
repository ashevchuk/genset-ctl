#pragma once

#include <stdint.h>
#include <stddef.h>
#include <time.h>

// I2CTask: sole owner of the I2C bus — ADS1115×2, INA233, MCP9600, DS3231 RTC,
// SSD1306 OLED (via oled_menu.cpp's i2c_render_display()), FM24V10 FRAM
// (via fram_log.cpp's fram_log_service_tick()); tacho. Also owns the
// on-demand I2CJob_t job queue (i2c_run_on_owner(), prototype in
// rtos_config.h — called from fram_log.cpp too, so it stays declared there,
// not here).
// Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's
// "Code organization" and [[main-cpp-modularization]] project memory for
// the pattern this follows).
void I2CTask(void *pvParameters);

// Wraps time(NULL) — see i2c.cpp for why this isn't a direct RTC read.
// Called from report.cpp and the main .cpp's own loop() (NTP-sync path).
uint32_t rtc_now_unixtime(void);

// On-demand DS3231 write for the NTP-triggered adjustment in the main
// .cpp's loop() — the actual hardware write goes through I2CTask via
// i2c_run_on_owner(), same pattern as FRAM admin ops and the on-demand
// INA233 read.
bool rtc_adjust_now(time_t unix_time);

// Boot-time (pre-RTOS) NTP/RTC bring-up, joined this module 2026-09-24
// alongside the RTC-owning code above — called once each from the main
// .cpp's setup().
void setup_ntp();
void setup_rtc();
