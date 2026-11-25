#pragma once

// Pre-RTOS-boot OTA-only recovery mode, entered from the main .cpp's
// setup() when REG_IN_JP1 (a physical jumper) is off at boot — starts the
// HTTP server with a reduced route set (/reset, /wifi-config,
// /api/wifi/save, /update; see http_server_start_update_mode()) and never
// returns. Extracted
// 2026-09-24 from esp32-gen-ctl-ai-rtos.cpp — see CLAUDE.md's "Code
// organization" and [[main-cpp-modularization]] project memory.
void update_mode(void);
