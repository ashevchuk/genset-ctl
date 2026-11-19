#!/usr/bin/env bash
# Mandatory build_archive step for decode_crash.sh.
#
# Usage: ./archive_build.sh BUILD_DIR [variant]
#   variant omitted              -> esp32-gen-ctl-ai-rtos-idf_<VERSION>.tar.xz
#   variant nodebug              -> ...-idf-nodebug_<VERSION>.tar.xz
#   variant bootloader-update    -> ...-idf-bootloader-update_<VERSION>.tar.xz
#
# VERSION is esp_app_desc.version[] (yyyymmddhhmm), the same stamp /log and
# fw_update carry. compile*/flash* call bump_fw_version.sh before idf.py
# build so that stamp tracks wall-clock build time (not the last time the
# version TU happened to change). A second distinct ELF compiled in the
# same minute still gets _<first8 of elf sha256> appended so the name stays
# unique.
#
# Writes .bin+.elf with xz -6 -T0, plus a sidecar <archive>.elf.sha256 so a
# later compile/flash/burn of the same ELF is a no-op instead of a duplicate.
# Exit 0 only if this ELF is already archived or a complete new archive is
# on disk. Tar/xz failure, missing inputs, or Ctrl+C during compress all
# fail the caller — compile/flash/burn must not proceed as if it succeeded.
#
# Found live 2026-09-27: soak image 202609251849 was OTA'd via burn_nodebug.sh
# after flash_nodebug.sh / idf.py flash rebuilt it, and compile_nodebug.sh's
# own tar was never a gate on flash/OTA. decode_crash.sh then had no ELF.

set -euo pipefail

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "Usage: $0 BUILD_DIR [nodebug|bootloader-update]" >&2
    exit 1
fi

BUILD_DIR=$(cd "$1" && pwd)
VARIANT=${2:-}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
# BUILD_ARCHIVE_DIR override is for tests; production callers leave it unset.
ARCHIVE_DIR="${BUILD_ARCHIVE_DIR:-${PROJECT_DIR}/build_archive}"

BIN_NAME="esp32-gen-ctl-ai-rtos-idf.bin"
ELF_NAME="esp32-gen-ctl-ai-rtos-idf.elf"
BIN="${BUILD_DIR}/${BIN_NAME}"
ELF="${BUILD_DIR}/${ELF_NAME}"

if [ -n "${VARIANT}" ]; then
    PREFIX="esp32-gen-ctl-ai-rtos-idf-${VARIANT}"
else
    PREFIX="esp32-gen-ctl-ai-rtos-idf"
fi

if [ ! -f "${BIN}" ]; then
    echo "ERROR: archive_build.sh: firmware bin not found: ${BIN}" >&2
    exit 1
fi
if [ ! -f "${ELF}" ]; then
    echo "ERROR: archive_build.sh: firmware elf not found: ${ELF}" >&2
    echo "(decode_crash.sh matches coredumps against the ELF SHA256 — a .bin-only archive is useless.)" >&2
    exit 1
fi

command -v tar >/dev/null || { echo "ERROR: archive_build.sh: tar not found" >&2; exit 1; }
command -v xz >/dev/null || { echo "ERROR: archive_build.sh: xz not found" >&2; exit 1; }
command -v sha256sum >/dev/null || { echo "ERROR: archive_build.sh: sha256sum not found" >&2; exit 1; }
command -v python3 >/dev/null || { echo "ERROR: archive_build.sh: python3 not found (needed to read esp_app_desc.version)" >&2; exit 1; }

# Same field /log and fw_update show: version[32] in esp_app_desc, 12 digits
# yyyymmddhhmm from __DATE__/__TIME__. Magic 0xABCD5432 at the start of the
# descriptor; skip false-positive hits that aren't this project's name.
VERSION=$(python3 - "${BIN}" <<'PY'
import sys
data = open(sys.argv[1], "rb").read()
magic = b"\x32\x54\xcd\xab"
i = 0
while True:
    j = data.find(magic, i)
    if j < 0:
        sys.exit(1)
    ver = data[j + 16:j + 48].split(b"\x00", 1)[0]
    name = data[j + 48:j + 80].split(b"\x00", 1)[0]
    try:
        ver_s = ver.decode("ascii")
        name_s = name.decode("ascii")
    except UnicodeDecodeError:
        i = j + 1
        continue
    if len(ver_s) == 12 and ver_s.isdigit() and name_s == "esp32-gen-ctl-ai-rtos":
        print(ver_s)
        sys.exit(0)
    i = j + 1
PY
) || {
    echo "ERROR: archive_build.sh: could not read yyyymmddhhmm version from ${BIN}" >&2
    exit 1
}

ELF_SHA=$(sha256sum "${ELF}" | awk '{print $1}')
if [ -z "${ELF_SHA}" ] || [ "${#ELF_SHA}" -ne 64 ]; then
    echo "ERROR: archive_build.sh: failed to hash ${ELF}" >&2
    exit 1
fi

mkdir -p "${ARCHIVE_DIR}"

# Sidecar written next to each new archive. Old archives (pre-this-helper)
# have none — first flash/burn of such an image creates one extra copy
# rather than silently assuming a timestamp match is the same ELF.
if [ -d "${ARCHIVE_DIR}" ]; then
    shopt -s nullglob
    for sidecar in "${ARCHIVE_DIR}"/*.elf.sha256; do
        if grep -qx "${ELF_SHA}" "${sidecar}"; then
            existing=$(basename "${sidecar}" .elf.sha256)
            echo "Already archived (elf sha256 ${ELF_SHA}) -> ${ARCHIVE_DIR}/${existing}"
            exit 0
        fi
    done
    shopt -u nullglob
fi

ARCHIVE_NAME="${PREFIX}_${VERSION}.tar.xz"
ARCHIVE="${ARCHIVE_DIR}/${ARCHIVE_NAME}"
if [ -e "${ARCHIVE}" ] || [ -e "${ARCHIVE}.partial" ]; then
    # Same compile-minute, different ELF (SHA-dedup above already skipped
    # the identical-ELF case). Keep the version in the name; disambiguate
    # with the hash prefix rather than a tar wall-clock.
    ARCHIVE_NAME="${PREFIX}_${VERSION}_${ELF_SHA:0:8}.tar.xz"
    ARCHIVE="${ARCHIVE_DIR}/${ARCHIVE_NAME}"
    if [ -e "${ARCHIVE}" ] || [ -e "${ARCHIVE}.partial" ]; then
        echo "ERROR: archive_build.sh: ${ARCHIVE} already exists" >&2
        exit 1
    fi
    echo "NOTE: ${PREFIX}_${VERSION}.tar.xz already exists with a different ELF; using ${ARCHIVE_NAME}"
fi
TMP="${ARCHIVE}.partial"
SIDECAR="${ARCHIVE}.elf.sha256"

cleanup_partial() {
    rm -f "${TMP}"
}
trap cleanup_partial EXIT INT TERM

tar -C "${BUILD_DIR}" --use-compress-program="xz -6 -T0" \
    -cf "${TMP}" "${BIN_NAME}" "${ELF_NAME}"

if [ ! -s "${TMP}" ]; then
    echo "ERROR: archive_build.sh: tar produced an empty file" >&2
    exit 1
fi

members=$(tar -tJf "${TMP}")
echo "${members}" | grep -qx "${BIN_NAME}" || {
    echo "ERROR: archive_build.sh: archive missing ${BIN_NAME}" >&2
    exit 1
}
echo "${members}" | grep -qx "${ELF_NAME}" || {
    echo "ERROR: archive_build.sh: archive missing ${ELF_NAME}" >&2
    exit 1
}

# Rename first, then sidecar. A crash between them leaves a valid tar.xz
# (decode_crash.sh can use it) and the next run writes a duplicate instead
# of claiming a sidecar whose archive is missing. Never leave a .tar.xz
# decode_crash.sh would try to open unless both members listed above.
mv -f "${TMP}" "${ARCHIVE}"
echo "${ELF_SHA}" > "${SIDECAR}"
trap - EXIT INT TERM

echo "Archived build -> ${ARCHIVE}"
echo "  version ${VERSION}  elf sha256 ${ELF_SHA}"
