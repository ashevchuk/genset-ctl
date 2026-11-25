#!/bin/sh
# OTA update esp32-gen-ctl-ai-rtos (idf.py build) via HTTP multipart upload —
# same as ./burn.sh, but points at compile_nodebug.sh's own build_nodebug/
# output instead of compile.sh's build/. A separate script, not a flag on
# burn.sh: burn.sh silently flashing whichever binary happens to be sitting
# in build/ regardless of which compile script you last ran is exactly the
# trap this exists to avoid (found live 2026-09-25 — compile_nodebug.sh then
# plain burn.sh re-flashed the stale debug build from build/, not the fresh
# nodebug one, and the controller kept printing debug output because it was
# never actually running the new firmware).

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PROJECT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)

BUILD_DIR=${SCRIPT_DIR}/build_nodebug
# project() name in idf/CMakeLists.txt, not the checkout directory name.
BIN=${BUILD_DIR}/esp32-gen-ctl-ai-rtos-idf.bin

if [ -f "${PROJECT_DIR}/host" ]; then
    IP=$(cat "${PROJECT_DIR}/host")
fi

if [ -f "${BIN}" ]; then
    # Archive before OTA — see burn.sh / archive_build.sh. This is the path
    # that actually put 202609251849 on the test rig with no matching ELF.
    "${SCRIPT_DIR}/archive_build.sh" "${BUILD_DIR}" nodebug || exit 1
    echo "${BIN} exists. updating ${IP}..."
    curl -F "update=@${BIN}" ${IP}/update
else
    echo "Error: firmware not found: ${BIN}"
    echo "Run ./compile_nodebug.sh first."
    exit 1
fi
