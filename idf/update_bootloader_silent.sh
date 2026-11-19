#!/bin/sh
# Same as ./update_bootloader.sh, but pushes the SILENT bootloader
# (CONFIG_BOOTLOADER_LOG_LEVEL_NONE, sdkconfig.nodebug.defaults — see
# ../CLAUDE.md's compile_nodebug.sh bullet) instead of the normal chatty
# one. Uses compile_nodebug.sh's own build_nodebug/bootloader/bootloader.bin
# as the payload — this is the ONLY thing that differs from
# update_bootloader.sh, the HTTP route/procedure is identical.
#
# REQUIRES the device to already be running a build with
# POST /api/bootloader/update compiled in (./compile_bootloader_update.sh +
# ./burn_bootloader_update.sh) — this script only ever uploads a
# bootloader.bin, it does not build or OTA the app itself.

PROJECT_DIR=$(cd "$(dirname "$0")/.." && pwd)
BOOTLOADER_BIN=$(cd "$(dirname "$0")" && pwd)/build_nodebug/bootloader/bootloader.bin

if [ -f "${PROJECT_DIR}/host" ]; then
    IP=$(cat "${PROJECT_DIR}/host")
fi

if [ ! -f "${BOOTLOADER_BIN}" ]; then
    echo "Error: bootloader not found: ${BOOTLOADER_BIN}"
    echo "Run ./compile_nodebug.sh first."
    exit 1
fi

echo "${BOOTLOADER_BIN} exists ($(wc -c < "${BOOTLOADER_BIN}") bytes)."
echo "Pushing to ${IP}/api/bootloader/update — this rewrites the bootloader region"
echo "in place, with no A/B fallback. Only proceeds if the device is currently"
echo "running a BOOTLOADER_UPDATE_ENDPOINT_ENABLED=1 build (see burn_bootloader_update.sh)."
curl -F "update=@${BOOTLOADER_BIN}" "${IP}/api/bootloader/update?confirm=1"
echo
echo "If that printed \"OK. Rebooting...\": wait for the device to come back, then"
echo "confirm it's healthy (GET /api/fw/partitions, watch ttyACM if possible), and"
echo "immediately OTA a normal (flag-off) build back in — ./burn.sh or ./burn_nodebug.sh."
echo "Do not leave the BOOTLOADER_UPDATE_ENDPOINT_ENABLED build running any longer"
echo "than this one operation needs."
