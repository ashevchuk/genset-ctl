#!/bin/sh
# Flash the silent (DEBUG_SERIAL_ENABLED=0) build over USB — same as
# ./flash.sh, but targets build_nodebug/ instead of build/ (see
# compile_nodebug.sh/burn_nodebug.sh's own comments for why this is a
# separate script rather than a flag: build/ vs build_nodebug/ has to stay a
# hard directory split, not something a forgotten flag can silently get
# wrong). Also exports the same EXTRA_CPPFLAGS compile_nodebug.sh does —
# `idf.py flash` builds first if needed, so running this directly (without
# ever having run compile_nodebug.sh) must still end up with the flag
# actually set, not a build_nodebug/ that isn't really nodebug.
#
# That standalone rebuild used to skip compile_nodebug.sh's tar entirely
# (found live 2026-09-27: soak image 202609251849). Build, archive, flash,
# archive-again — same gate as ./flash.sh, pointed at build_nodebug/.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PORT=${1:-/dev/ttyACM0}
BUILD_DIR="${SCRIPT_DIR}/build_nodebug"

export EXTRA_CPPFLAGS="-DDEBUG_SERIAL_ENABLED=0"

# See compile_nodebug.sh's matching comment — layers sdkconfig.nodebug.defaults
# (bootloader/app log levels compiled to NONE) on top of sdkconfig.defaults,
# same reasoning as EXTRA_CPPFLAGS above: must be set here too so running
# this script standalone (without compile_nodebug.sh having run first)
# still ends up with a genuinely silent build, not a half-silent one.
export SDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.nodebug.defaults"

# See compile_nodebug.sh's matching comment — idf.py's default sdkconfig
# path ignores -B entirely (it's CMAKE_SOURCE_DIR/sdkconfig, shared across
# every build variant unless overridden), so this must be passed explicitly
# on every idf.py call here too, not just in compile_nodebug.sh. Plain
# variable + unquoted expansion, not a bash array — this script is
# #!/bin/sh (POSIX dash), arrays aren't available.
SDKCONFIG_ARG="SDKCONFIG=${BUILD_DIR}/sdkconfig"

source /opt/esp-idf/export.sh > /dev/null

cd "${SCRIPT_DIR}"

# Fresh esp_app_desc.version[] (yyyymmddhhmm) even on incremental rebuilds —
# see bump_fw_version.sh.
./bump_fw_version.sh || exit 1

idf.py -D "${SDKCONFIG_ARG}" -B "${BUILD_DIR}" build || exit 1
./archive_build.sh "${BUILD_DIR}" nodebug || exit 1
idf.py -D "${SDKCONFIG_ARG}" -B "${BUILD_DIR}" -p "${PORT}" flash || exit 1
./archive_build.sh "${BUILD_DIR}" nodebug || exit 1
