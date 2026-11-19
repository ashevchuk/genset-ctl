#pragma once

// PowerMeterTask: reads the MCP39F511A power meter over ESP-IDF UART (UART2).
// Extracted 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp (see CLAUDE.md's
// "Code organization" and [[main-cpp-modularization]] project memory for
// the pattern this follows).
void PowerMeterTask(void *pvParameters);

// Boot-time config/calibration read + debug-dump helpers — all called once
// from the main .cpp's setup(), joined this module 2026-09-24 since
// PowerMeterTask already owns the device (mcp39f511a). See each function's
// own comment in powermeter.cpp for what it does.
bool mcp39f511a_load_config_from_flash();
void mcp39f511a_print_config();
void mcp39f511a_print_config_serial(void);
void mcp39f511a_print_full_config_serial(void);
void mcp39f511a_print_energy_counters_serial(void);

// After a successful mcp39f511a.resetEnergyCounters(): zero the 30s energy
// cache and g_state.power energy fields so WS/OLED show 0 immediately
// instead of waiting up to MCP39F511A_ENERGY_READ_PERIOD_MS.
void powermeter_note_energy_reset(void);
