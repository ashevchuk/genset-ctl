#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT="$(basename "$PROJECT_DIR")"

# .ino/arduino-builder retired 2026-09-23 — main/<project>.cpp is real,
# hand-maintained C++ source now (see the note at its own top), no longer
# synced from anywhere. idf.py/CMake only ever sees whatever's sitting in
# that file.
source /opt/esp-idf/export.sh > /dev/null

cd "$SCRIPT_DIR"

# Fresh esp_app_desc.version[] (yyyymmddhhmm) even on incremental rebuilds —
# see bump_fw_version.sh.
./bump_fw_version.sh || exit 1

idf.py build
BUILD_STATUS=$?

# Archive is part of a successful compile, not a best-effort extra: tar
# used to run without `set -e` / `|| exit 1`, so a failed/interrupted xz
# still printed "build ready" and left burn.sh free to OTA an unarchived
# ELF (found live 2026-09-27 — see archive_build.sh). Failure here fails
# the script even if ninja itself succeeded.
if [ "${BUILD_STATUS}" -ne 0 ]; then
    exit "${BUILD_STATUS}"
fi
"${SCRIPT_DIR}/archive_build.sh" "${SCRIPT_DIR}/build" || exit 1
exit 0
