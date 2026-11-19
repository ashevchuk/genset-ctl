#!/bin/sh
# Flash esp32-gen-ctl-ai-rtos (idf.py build) over USB.
#
# Unlike ../flash.sh (which hand-assembles an esptool command line because
# arduino-builder doesn't generate one itself), this just delegates to
# `idf.py flash` — idf.py already writes build/flash_args with the correct
# bootloader+partition-table+ota_data_initial+app offsets for this exact
# build, so duplicating that by hand here would just be another place for
# the offsets to drift out of sync with ../partitions.csv.
#
# Always builds/flashes build/ (compile.sh's own output) — for the silent
# (DEBUG_SERIAL_ENABLED=0) build use ./flash_nodebug.sh instead, which
# targets build_nodebug/ and forces the flag itself (see burn.sh's own
# matching note; same trap, same fix, on the USB side instead of OTA).
#
# `idf.py flash` rebuilds if ninja is dirty, so this script used to be able
# to put a brand-new ELF on the chip without going through compile.sh's tar
# (found live 2026-09-27). Build, archive (must succeed), then flash; archive
# again afterwards in case flash's own ninja pass rebuilt anything — the
# helper SHA-dedups, so the common compile-then-flash path is a no-op.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT=${1:-/dev/ttyACM0}
BUILD_DIR="${SCRIPT_DIR}/build"

source /opt/esp-idf/export.sh > /dev/null

cd "${SCRIPT_DIR}"

# See compile.sh's matching comment — a fresh clone has no managed_components/
# yet for patch_arduino_esp32.sh to patch until something fetches it.
if [ ! -d "${SCRIPT_DIR}/managed_components" ]; then
    echo "managed_components/ not present yet (fresh checkout) — running idf.py reconfigure to fetch dependencies..."
    idf.py reconfigure || exit 1
fi

# See compile.sh / patch_arduino_esp32.sh — reapplies a hand patch to the
# arduino-esp32 managed component that a component-manager refresh would
# otherwise silently discard.
./patch_arduino_esp32.sh || exit 1

# Fresh esp_app_desc.version[] (yyyymmddhhmm) even on incremental rebuilds —
# see bump_fw_version.sh.
./bump_fw_version.sh || exit 1

idf.py build || exit 1
./archive_build.sh "${BUILD_DIR}" || exit 1
idf.py -p "${PORT}" flash || exit 1
./archive_build.sh "${BUILD_DIR}" || exit 1
