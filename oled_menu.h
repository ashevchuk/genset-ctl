#pragma once

#include <stddef.h>  // size_t

// OLED menu system: SSD1306 status/config UI driven by the rotary encoder.
// Owns MENU_MAIN/MENU_STATUS/MENU_WIFI*/MENU_MAINT*/MENU_FUEL and the
// generic list/confirm screen engine behind them — see the module's own
// banner comment in oled_menu.cpp for the full picture. Deliberately tiny
// public surface: everything else in there is static to that file.

// Encoder polling + menu navigation task (no I2C at all — see rtos_config.h
// task-priority table). Created once from setup_rtos() in the main .cpp.
void DisplayTask(void *pvParameters);

// Pulls the latest menu snapshot off xMenuRenderQueue and draws it. Called
// once per ~200ms from I2CTask (the sole OLED/I2C writer) — never call this
// from anywhere else.
void i2c_render_display(void);

// sizeof(MenuState_t) — that type stays private to oled_menu.cpp; the only
// other file that needs it is setup_rtos() (main .cpp), to size
// xMenuRenderQueue at creation.
size_t oled_menu_state_size(void);
