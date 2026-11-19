#!/bin/sh
# OTA update esp32-gen-ctl-ai-rtos (idf.py build) via HTTP multipart upload —
# same as ../idf/burn.sh, but points at compile_bootloader_update.sh's own
# build_bootloader_update/ output instead of compile.sh's build/. A separate
# script, not a flag on burn.sh — see burn_nodebug.sh's own comment for why
# (same trap, same fix: a script that could silently act on the wrong
# build's binary is worse here than anywhere else in this project, given
# what this specific build's own route can do).

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
PROJECT="${PROJECT_DIR##*/}"

BUILD_DIR=${SCRIPT_DIR}/build_bootloader_update
BIN=${BUILD_DIR}/${PROJECT}-idf.bin

if [ -f "${PROJECT_DIR}/host" ]; then
    IP=$(cat "${PROJECT_DIR}/host")
fi

if [ -f "${BIN}" ]; then
    "${SCRIPT_DIR}/archive_build.sh" "${BUILD_DIR}" bootloader-update || exit 1
    echo "${BIN} exists. updating ${IP}..."
    curl -F "update=@${BIN}" ${IP}/update
else
    echo "Error: firmware not found: ${BIN}"
    echo "Run ./compile_bootloader_update.sh first."
    exit 1
fi
