#!/usr/bin/env bash
# patch_arduino_esp32.sh — re-applies small, targeted patches to the
# espressif/arduino-esp32 managed component after every idf.py component-
# manager refresh (idf.py reconfigure / update-dependencies / a wiped
# managed_components/ all silently re-fetch this file from the registry,
# discarding any hand-edit with zero warning — unlike this project's own
# libraries/, nothing here is meant to survive that on its own). Run from
# compile.sh before every build so no patch can ever silently go missing.
#
# Each patch is idempotent and loud about anything unexpected: does nothing
# if already patched, patches if still stock, and REFUSES (loud, nonzero
# exit — never silently skips) if the file matches neither, since that
# means arduino-esp32 was updated to a version whose source no longer
# matches what the patch assumes and it needs a human to re-check before
# blindly sed-ing it. Patches are independent of each other — one failing
# does not stop the others from being attempted, but the script's own exit
# code is nonzero if ANY of them fail.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MANAGED_DIR="${SCRIPT_DIR}/managed_components/espressif__arduino-esp32"

overall_status=0

# --- Patch 1: NetworkClient::write()'s own internal write-retry constants ---
patch_networkclient() {
    local target="${MANAGED_DIR}/libraries/Network/src/NetworkClient.cpp"

    if [ ! -f "$target" ]; then
        echo "ERROR: patch_arduino_esp32.sh: $target not found — managed_components/ not fetched yet? Run idf.py build once first." >&2
        return 1
    fi

    # Root cause (found live 2026-09-24, decode_crash.sh backtrace of a real
    # Report-task TWDT panic): NetworkClient::write()'s own internal select()
    # retry loop — WIFI_CLIENT_MAX_WRITE_RETRY attempts, each blocking up to
    # WIFI_CLIENT_SELECT_TIMEOUT_US on a not-yet-writable socket — has no
    # #ifndef guard and is entirely independent of WebSockets.cpp's own
    # WEBSOCKETS_TCP_TIMEOUT check (see CLAUDE.md's Web/MQTT interfaces
    # section): that check only runs BETWEEN calls to this write(), never
    # during one, so a single stalled client can block ReportTask for up to
    # 10 * 1000ms = ~10s regardless of WEBSOCKETS_TCP_TIMEOUT's value — enough
    # on its own, with 2+ stalled clients in one broadcastTXT() pass, to bust
    # the 15s TWDT window. Tightened here to a worst case of 3 * 100ms = 300ms
    # per call — retry resets to full on every partial-progress chunk, so a
    # genuinely slow-but-working client is unaffected; only a fully
    # unresponsive one gets cut off sooner.
    local old_retry='#define WIFI_CLIENT_MAX_WRITE_RETRY     (10)'
    local new_retry='#define WIFI_CLIENT_MAX_WRITE_RETRY     (3)   // patched — see idf/patch_arduino_esp32.sh'
    local old_timeout='#define WIFI_CLIENT_SELECT_TIMEOUT_US   (1000000)'
    local new_timeout='#define WIFI_CLIENT_SELECT_TIMEOUT_US   (100000)   // patched — see idf/patch_arduino_esp32.sh'

    local has_new=0
    grep -qF "$new_retry" "$target" && grep -qF "$new_timeout" "$target" && has_new=1
    local has_old=0
    grep -qF "$old_retry" "$target" && grep -qF "$old_timeout" "$target" && has_old=1

    if [ "$has_new" -eq 1 ]; then
        echo "patch_arduino_esp32.sh [NetworkClient.cpp]: already patched, nothing to do."
        return 0
    fi

    if [ "$has_old" -eq 1 ]; then
        # Portable in-place edit (works the same on GNU and BSD/macOS sed).
        sed -i.bak \
            -e "s|^#define WIFI_CLIENT_MAX_WRITE_RETRY.*\$|${new_retry}|" \
            -e "s|^#define WIFI_CLIENT_SELECT_TIMEOUT_US.*\$|${new_timeout}|" \
            "$target"
        rm -f "${target}.bak"
        echo "patch_arduino_esp32.sh [NetworkClient.cpp]: applied (WIFI_CLIENT_MAX_WRITE_RETRY 10->3, WIFI_CLIENT_SELECT_TIMEOUT_US 1000000->100000)."
        return 0
    fi

    echo "ERROR: patch_arduino_esp32.sh: $target matches neither the stock nor the already-patched text." >&2
    echo "arduino-esp32 was probably updated to a different version — check NetworkClient::write() by hand" >&2
    echo "(WIFI_CLIENT_MAX_WRITE_RETRY / WIFI_CLIENT_SELECT_TIMEOUT_US) and update this script's old_*/new_* strings." >&2
    return 1
}

# --- Patch 2: disable arduino-esp32's own built-in WebServer ---
patch_disable_webserver() {
    local target="${MANAGED_DIR}/CMakeLists.txt"

    if [ ! -f "$target" ]; then
        echo "ERROR: patch_arduino_esp32.sh: $target not found — managed_components/ not fetched yet? Run idf.py build once first." >&2
        return 1
    fi

    # libraries/WebServer/ now vendors a patched copy of this library (adds
    # the WEBSERVER_HAS_HOOK mechanism so WebSockets4WebServer.h can share
    # port 80 at /ws instead of WebSocketsServer owning its own port 81 —
    # see CLAUDE.md's Web/MQTT interfaces section). Unlike WebSockets/
    # ArduinoJson/etc (already separate idf components), WebServer lives
    # INSIDE the single monolithic arduino-esp32 component itself —
    # this CMakeLists.txt's own ARDUINO_ALL_LIBRARIES loop compiles it
    # straight into that component's own SRCS. Leaving it enabled there
    # while also registering our own separate libraries/WebServer component
    # would define WebServer::WebServer() (and everything else) twice and
    # fail to link. Commenting its entry out of ARDUINO_ALL_LIBRARIES is the
    # smallest fix: every other bundled library this project uses (WiFi,
    # ESPmDNS, Update, Preferences, FS, ...) stays exactly as-is, only
    # WebServer's own entry is removed.
    local old_line='  WebServer'
    local new_line='  # WebServer  # disabled — patched by idf/patch_arduino_esp32.sh, vendored separately as libraries/WebServer/ (see its own CMakeLists.txt comment)'

    local has_new=0
    grep -qF "$new_line" "$target" && has_new=1
    local has_old=0
    grep -qxF "$old_line" "$target" && has_old=1

    if [ "$has_new" -eq 1 ]; then
        echo "patch_arduino_esp32.sh [CMakeLists.txt]: already patched, nothing to do."
        return 0
    fi

    if [ "$has_old" -eq 1 ]; then
        sed -i.bak "s|^${old_line}\$|${new_line}|" "$target"
        rm -f "${target}.bak"
        echo "patch_arduino_esp32.sh [CMakeLists.txt]: applied (arduino-esp32's own built-in WebServer disabled in favor of libraries/WebServer/)."
        return 0
    fi

    echo "ERROR: patch_arduino_esp32.sh: $target's ARDUINO_ALL_LIBRARIES list matches neither the stock nor the already-patched text." >&2
    echo "arduino-esp32 was probably updated to a different version — check the WebServer entry by hand and update this script." >&2
    return 1
}

# --- Patch 3: silence a missing-field-initializers error in ESP_I2S.cpp ---
patch_i2s_missing_field_initializers() {
    local target="${MANAGED_DIR}/CMakeLists.txt"

    if [ ! -f "$target" ]; then
        echo "ERROR: patch_arduino_esp32.sh: $target not found — managed_components/ not fetched yet? Run idf.py build once first." >&2
        return 1
    fi

    # Found live 2026-11-15, rebuilding against a just-updated system
    # ESP-IDF (v6.0.2 -> v6.1): i2s_chan_config_t gained two new members
    # (tx_destination/rx_destination), and libraries/ESP_I2S/src/ESP_I2S.cpp's
    # own I2S_DEFAULT_CFG macro (unrelated to this project, part of
    # arduino-esp32 itself) doesn't initialize them — trips
    # -Werror=missing-field-initializers under this project's -Wall -Wextra
    # -Werror. This project has no I2S audio hardware and never calls into
    # this file; the existing -Wno-error=missing-field-initializers in
    # idf/main/CMakeLists.txt only covers the main component, not
    # arduino-esp32's own, so it needs its own scoped suppression here —
    # same reasoning as every other per-component -Wno-error= in this repo.
    local old_marker='    -DESP32=ESP32)'

    local has_new=0
    grep -qF "missing-field-initializers" "$target" && has_new=1

    if [ "$has_new" -eq 1 ]; then
        echo "patch_arduino_esp32.sh [CMakeLists.txt missing-field-initializers]: already patched, nothing to do."
        return 0
    fi

    local has_old=0
    grep -qxF "$old_marker" "$target" && has_old=1

    if [ "$has_old" -eq 1 ]; then
        python3 - "$target" <<'PYEOF'
import sys
path = sys.argv[1]
with open(path) as f:
    content = f.read()
old = "    -DESP32=ESP32)\n"
new = old + """
# Patched by idf/patch_arduino_esp32.sh: ESP-IDF 6.1's i2s_chan_config_t
# gained members ESP_I2S.cpp's own I2S_DEFAULT_CFG macro does not
# initialize, tripping -Werror=missing-field-initializers. Unused by this
# project (no I2S audio); suppressed rather than upstream-patched since this
# file is auto-refetched by the component manager.
target_compile_options(${COMPONENT_TARGET} PRIVATE -Wno-error=missing-field-initializers)
"""
if content.count(old) != 1:
    sys.exit(1)
content = content.replace(old, new, 1)
with open(path, "w") as f:
    f.write(content)
PYEOF
        if [ $? -ne 0 ]; then
            echo "ERROR: patch_arduino_esp32.sh: anchor line found but replace failed unexpectedly." >&2
            return 1
        fi
        echo "patch_arduino_esp32.sh [CMakeLists.txt missing-field-initializers]: applied (scoped -Wno-error=missing-field-initializers for arduino-esp32's own component)."
        return 0
    fi

    echo "ERROR: patch_arduino_esp32.sh: $target's target_compile_options block matches neither the stock nor the already-patched text." >&2
    echo "arduino-esp32 was probably updated to a different version — check the -DESP32=ESP32) block by hand and update this script." >&2
    return 1
}

patch_networkclient || overall_status=1
patch_disable_webserver || overall_status=1
patch_i2s_missing_field_initializers || overall_status=1

exit "$overall_status"
