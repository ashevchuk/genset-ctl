#!/bin/sh
# OTA update esp32-gen-ctl-ai-rtos (idf.py build) via HTTP multipart upload.
# Mirrors ../burn.sh's flow exactly, just points at this build's own
# output binary (idf/build/esp32-gen-ctl-ai-rtos-idf.bin) instead of
# arduino-builder's /tmp output. Target host comes from ../host, same
# convention as every other script here.
#
# Always flashes build/ (compile.sh's own output) regardless of which
# compile script you last ran — if you last ran compile_nodebug.sh, this
# still re-flashes whatever's already in build/, not your fresh nodebug
# build. Use ./burn_nodebug.sh for that one instead.

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

BUILD_DIR=${SCRIPT_DIR}/build
# project() name in idf/CMakeLists.txt, not the checkout directory name.
BIN=${BUILD_DIR}/esp32-gen-ctl-ai-rtos-idf.bin

if [ -f "${PROJECT_DIR}/host" ]; then
    IP=$(cat "${PROJECT_DIR}/host")
fi

if [ -f "${BIN}" ]; then
    # Archive before OTA: burn.sh used to curl whatever sat in build/, even
    # if compile.sh's tar was skipped or interrupted (found live 2026-09-27).
    "${SCRIPT_DIR}/archive_build.sh" "${BUILD_DIR}" || exit 1
    echo "${BIN} exists. updating ${IP}..."
    curl -F "update=@${BIN}" ${IP}/update
else
    echo "Error: firmware not found: ${BIN}"
    echo "Run ./compile.sh first."
    exit 1
fi
