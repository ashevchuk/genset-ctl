#!/usr/bin/env bash
set -uo pipefail
# Same firmware as ./compile.sh, but with BOOTLOADER_UPDATE_ENDPOINT_ENABLED
# forced to 1 — compiles in POST /api/bootloader/update (see
# bootloader_update.h/.cpp), a temporary, single-use capability for pushing
# a new bootloader.bin to a controller with no physical USB access. Off (0)
# in every other build on purpose: a standing, unauthenticated "rewrite my
# own bootloader" HTTP route is a severe, avoidable liability on a
# controller that otherwise never has this door open. The intended
# workflow: build+OTA this once, hit the route once, confirm the device
# reboots cleanly on the new bootloader, then OTA back a normal
# (flag-off) build immediately — see CLAUDE.md's OTA-rollback section.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT="$(basename "$PROJECT_DIR")"

# Same EXTRA_CPPFLAGS mechanism compile_nodebug.sh uses — see its own
# comment for why this is a global -D via ESP-IDF's project.cmake rather
# than an edit to a tracked file.
export EXTRA_CPPFLAGS="-DBOOTLOADER_UPDATE_ENDPOINT_ENABLED=1"

# Own build directory, not compile.sh's build/ — same reasoning as
# compile_nodebug.sh's BUILD_DIR: EXTRA_CPPFLAGS is only read at
# cmake-configure time, and CMake won't notice it changed between runs, so
# sharing a build dir risks silently keeping whichever flags were cached
# from whichever script last configured it — exactly the kind of mixup
# this route's own risk profile can least afford.
BUILD_DIR="${SCRIPT_DIR}/build_bootloader_update"

source /opt/esp-idf/export.sh > /dev/null

cd "$SCRIPT_DIR"

# Fresh esp_app_desc.version[] (yyyymmddhhmm) even on incremental rebuilds —
# see bump_fw_version.sh.
./bump_fw_version.sh || exit 1

idf.py -B "$BUILD_DIR" build
BUILD_STATUS=$?

if [ "${BUILD_STATUS}" -ne 0 ]; then
    exit "${BUILD_STATUS}"
fi
"${SCRIPT_DIR}/archive_build.sh" "${BUILD_DIR}" bootloader-update || exit 1

echo ""
echo "Bootloader-update build ready: ${BUILD_DIR}/esp32-gen-ctl-ai-rtos-idf.bin"
echo "OTA it in: ./burn_bootloader_update.sh"
echo ""
echo "Then, once OTA'd and running:"
echo "  curl -F \"update=@/path/to/new/bootloader.bin\" \"http://<host>/api/bootloader/update?confirm=1\""
echo ""
echo "Confirm the device reboots cleanly, THEN immediately OTA back a normal"
echo "(flag-off) build via ../burn.sh or ./burn_nodebug.sh — do not leave this"
echo "build running any longer than the single operation needs."
exit 0
