#!/bin/sh
# Pushes the NORMAL (chatty, CONFIG_BOOTLOADER_LOG_LEVEL_INFO — see
# ../CLAUDE.md's OTA-rollback/bootloader_update.h section) bootloader.bin to
# a running controller over POST /api/bootloader/update?confirm=1 —
# the HTTP route bootloader_update.h/.cpp adds when
# BOOTLOADER_UPDATE_ENDPOINT_ENABLED=1. Uses compile.sh's own
# build/bootloader/bootloader.bin as the payload (same bootloader
# ./flash.sh would put on the chip over USB) — for the silent variant use
# ./update_bootloader_silent.sh instead.
#
# REQUIRES the device to already be running a build with the route compiled
# in (./compile_bootloader_update.sh + ./burn_bootloader_update.sh) — this
# script only ever uploads a bootloader.bin, it does not build or OTA the
# app itself. See ../CLAUDE.md and BOOTLOADER-UPDATE.md (repo root) for the
# full procedure and why this is a rare, single-use operation, never a
# standing step.

PROJECT_DIR=$(cd "$(dirname "$0")/.." && pwd)
BOOTLOADER_BIN=$(cd "$(dirname "$0")" && pwd)/build/bootloader/bootloader.bin

if [ -f "${PROJECT_DIR}/host" ]; then
    IP=$(cat "${PROJECT_DIR}/host")
fi

if [ ! -f "${BOOTLOADER_BIN}" ]; then
    echo "Error: bootloader not found: ${BOOTLOADER_BIN}"
    echo "Run ./compile.sh first (any normal build — compile_bootloader_update.sh's"
    echo "own build_bootloader_update/bootloader/bootloader.bin works too, it's the"
    echo "same bootloader, just built alongside the endpoint-enabled app)."
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
