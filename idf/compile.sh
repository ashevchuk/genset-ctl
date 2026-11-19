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

# On a fresh clone managed_components/ doesn't exist yet — the component
# manager only fetches it as part of an idf.py run (build/reconfigure), but
# patch_arduino_esp32.sh needs the fetched source to already be there to
# patch. `idf.py reconfigure` does the fetch without a full build, so a
# first-ever checkout bootstraps itself here instead of failing outright.
if [ ! -d "$SCRIPT_DIR/managed_components" ]; then
    echo "managed_components/ not present yet (fresh checkout) — running idf.py reconfigure to fetch dependencies..."
    idf.py reconfigure || exit 1
fi

# Re-applies (idempotently) a hand patch to the arduino-esp32 managed
# component that idf.py's own component manager would otherwise silently
# discard on a reconfigure/dependency refresh — see the script itself.
./patch_arduino_esp32.sh || exit 1

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
