#!/bin/sh
# Force a fresh esp_app_desc.version[] stamp on every intentional build.
#
# version[] is baked from __DATE__/__TIME__ in
# idf/main/esp32-gen-ctl-ai-rtos.cpp. ninja skips that TU when only other
# files changed, so an incremental rebuild can keep yesterday's (or this
# morning's) yyyymmddhhmm while the ELF itself is new — archive_build.sh
# then has to disambiguate with an _<elf8> suffix, and /log/fw_update show
# a stamp that doesn't match wall-clock compile time (found live
# 2026-09-28: compile at 15:11 archived as ..._202609281440_b646f7ab).
#
# Touching this one .cpp before every idf.py build makes the stamp track
# the wall-clock of the build script, so distinct firmwares stay
# distinguishable by the 12-digit version alone. Called from every script
# that runs `idf.py build` (compile.sh / compile_nodebug.sh /
# compile_bootloader_update.sh / flash.sh / flash_nodebug.sh) — burn*.sh
# only OTA an already-built bin and must not touch.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
FW_VERSION_SRC="${SCRIPT_DIR}/main/esp32-gen-ctl-ai-rtos.cpp"

if [ ! -f "${FW_VERSION_SRC}" ]; then
    echo "ERROR: bump_fw_version.sh: version TU not found: ${FW_VERSION_SRC}" >&2
    exit 1
fi

touch "${FW_VERSION_SRC}" || exit 1
