#pragma once

// Compile-time gate for the /api/bootloader/update HTTP route — a
// temporary, single-use capability, never meant to ship enabled in a
// normal build.
//
// OTA never touches the bootloader partition (0x1000-0x8000, ahead of the
// partition table) on purpose — unlike app0/app1, it has no A/B redundancy,
// so a power loss or reset mid-erase/write there bricks the device
// permanently, recoverable only via USB/esptool. See bootloader_update.cpp
// and CLAUDE.md's OTA-rollback section for the full rationale.
//
// This route exists ONLY to push a new bootloader (e.g. one built with
// CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE) to a controller with no physical
// USB access, as a one-time operation: build with this =1, OTA that build
// in, hit the route once, confirm the device reboots cleanly on the new
// bootloader, then OTA back a normal build with this =0 (the default) so
// the capability doesn't linger on a running controller any longer than
// the single operation needs it.
#ifndef BOOTLOADER_UPDATE_ENDPOINT_ENABLED
#define BOOTLOADER_UPDATE_ENDPOINT_ENABLED 0
#endif

#if BOOTLOADER_UPDATE_ENDPOINT_ENABLED
// Registers POST /api/bootloader/update on webServer. Call once from
// setup_server_routes_ws() (web.cpp) — a no-op declaration when the flag
// above is 0, so a normal build carries no trace of this route at all.
void setup_bootloader_update_route(void);
#endif
