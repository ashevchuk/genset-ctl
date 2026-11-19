#!/usr/bin/env bash
set -uo pipefail
# Silent build: same firmware as ./compile.sh, but with DEBUG_SERIAL_ENABLED
# forced to 0 — every SLOG_*() Serial call and ESP-IDF/Arduino-core's own
# ESP_LOG* output (see debug.h's SLOG_SILENCE_IDF_LOGS()) compiled out/
# silenced, for a build where nothing should assume a serial monitor is
# attached. Also layers sdkconfig.nodebug.defaults on top of the normal
# sdkconfig.defaults (see that file) — SLOG_SILENCE_IDF_LOGS() is a runtime
# call from inside setup(), so it can't reach the 2nd-stage bootloader's own
# banner or the handful of ESP-IDF boot lines printed before setup() ever
# runs; CONFIG_BOOTLOADER_LOG_LEVEL_NONE/CONFIG_LOG_DEFAULT_LEVEL_NONE are
# the compile-time options that do.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT="$(basename "$PROJECT_DIR")"

# Forces the flag globally without touching debug.h: EXTRA_CPPFLAGS is
# ESP-IDF's own project.cmake mechanism (read once, applied as a
# COMPILE_OPTIONS to every component in the build) — respects debug.h's own
# #ifndef guard exactly like any other -D would, just without editing a
# tracked file and needing to remember to revert it.
export EXTRA_CPPFLAGS="-DDEBUG_SERIAL_ENABLED=0"

# ESP-IDF's own project.cmake reads this env var (targets.cmake) as a
# semicolon-separated list, applied in order on top of each other — the
# project-root-relative path matches where sdkconfig.defaults already lives.
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.nodebug.defaults"

# Own build directory, not compile.sh's build/ — EXTRA_CPPFLAGS is only
# read at cmake-configure time, and CMake has no way to notice an
# environment variable changed between runs (it's not a tracked configure
# dependency), so alternating debug/nodebug builds in the same build/ risks
# silently reusing whichever flags were cached from the last configure
# regardless of which script you just ran. A separate directory sidesteps
# that instead of relying on remembering to `idf.py fullclean` on every
# switch.
BUILD_DIR="${SCRIPT_DIR}/build_nodebug"

# idf.py's default sdkconfig path is CMAKE_SOURCE_DIR/sdkconfig — i.e. the
# SAME idf/sdkconfig file compile.sh/compile_bootloader_update.sh use,
# regardless of -B. `-B` only isolates ninja/build outputs, not this file.
# Found live 2026-09-25: without this override, the very first run of this
# script (before this fix existed) silently baked
# CONFIG_BOOTLOADER_LOG_LEVEL_NONE/CONFIG_LOG_DEFAULT_LEVEL_NONE into the
# SHARED idf/sdkconfig — meaning the next plain ./compile.sh would have
# silently inherited a silenced bootloader/boot log too, since
# sdkconfig.defaults is only ever consulted while generating a brand-new
# sdkconfig (see CLAUDE.md's own documented gotcha on that). `-D SDKCONFIG=`
# is a genuine cmake cache entry (confirmed against project.cmake — this
# variable is read from cmake cache only, never from the environment, unlike
# SDKCONFIG_DEFAULTS above), so it has to be passed on every idf.py
# invocation here, not just the first.
SDKCONFIG_ARG="SDKCONFIG=${BUILD_DIR}/sdkconfig"

source /opt/esp-idf/export.sh > /dev/null

cd "$SCRIPT_DIR"

# See compile.sh's matching comment — a fresh clone has no managed_components/
# yet for patch_arduino_esp32.sh to patch until something fetches it.
if [ ! -d "$SCRIPT_DIR/managed_components" ]; then
    echo "managed_components/ not present yet (fresh checkout) — running idf.py reconfigure to fetch dependencies..."
    idf.py -D "$SDKCONFIG_ARG" -B "$BUILD_DIR" reconfigure || exit 1
fi

# Re-applies (idempotently) a hand patch to the arduino-esp32 managed
# component that idf.py's own component manager would otherwise silently
# discard on a reconfigure/dependency refresh — see the script itself.
# Shared with compile.sh (patches managed_components/ once, not per build
# directory).
./patch_arduino_esp32.sh || exit 1

# Fresh esp_app_desc.version[] (yyyymmddhhmm) even on incremental rebuilds —
# see bump_fw_version.sh.
./bump_fw_version.sh || exit 1

idf.py -D "$SDKCONFIG_ARG" -B "$BUILD_DIR" build
BUILD_STATUS=$?

# Archive is part of a successful compile, not a best-effort extra — see
# archive_build.sh. Filename still suffixed -nodebug so it's distinguishable
# at a glance; the trailing _<yyyymmddhhmm> is esp_app_desc.version[], which
# decode_crash.sh already parses.
if [ "${BUILD_STATUS}" -ne 0 ]; then
    exit "${BUILD_STATUS}"
fi
"${SCRIPT_DIR}/archive_build.sh" "${BUILD_DIR}" nodebug || exit 1

echo ""
echo "Silent build ready: ${BUILD_DIR}/esp32-gen-ctl-ai-rtos-idf.bin"
echo "OTA: ./burn_nodebug.sh   |   USB: ./flash_nodebug.sh [port]   (both from ${SCRIPT_DIR})"
echo "(plain ./burn.sh/./flash.sh always act on build/'s own binary, not this one)"
exit 0
