#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH="$SCRIPT_DIR"

# $ARDUINO_CLI/$LIBRARIES env vars win if set (per-machine override without
# editing this file); otherwise fall back to whatever's on PATH, then to
# this machine's Arduino IDE install layout as a last resort. No fixed
# version/username baked in, so this works on any machine.
ARDUINO_CLI="${ARDUINO_CLI:-$(command -v arduino-cli 2>/dev/null || true)}"
if [ -z "$ARDUINO_CLI" ]; then
    ARDUINO_CLI=$(ls -1 "$HOME"/devel/arduino-ide_*_Linux_64bit/resources/app/node_modules/arduino-ide-extension/build/arduino-cli 2>/dev/null | sort -V | tail -1)
fi
if [ -z "$ARDUINO_CLI" ] || [ ! -x "$ARDUINO_CLI" ]; then
    echo "ERROR: arduino-cli not found. Install it (or the Arduino IDE), or set ARDUINO_CLI=/path/to/arduino-cli." >&2
    exit 1
fi

# Board settings (from README.md)
FQBN="esp32:esp32:esp32s3:\
JTAGAdapter=default,\
PSRAM=opi,\
FlashMode=qio,\
FlashSize=16M,\
LoopCore=1,\
EventsCore=1,\
USBMode=hwcdc,\
CDCOnBoot=cdc,\
MSCOnBoot=default,\
DFUOnBoot=default,\
UploadMode=default,\
PartitionScheme=default_8MB,\
CPUFreq=240,\
UploadSpeed=921600,\
DebugLevel=warn,\
EraseFlash=none,\
ZigbeeMode=default"

LIBRARIES="${LIBRARIES:-$HOME/Arduino/libraries}"
BUILD_DIR="$SCRIPT_DIR/build/sketch"

echo "=== Compiling Generator Panel ==="
echo "Sketch : $SKETCH"
echo "FQBN   : $FQBN"
echo ""

"$ARDUINO_CLI" compile \
    --fqbn "$FQBN" \
    --libraries "$LIBRARIES" \
    --build-path "$SCRIPT_DIR/build/sketch" \
    --verbose \
    "$SKETCH"

echo ""
echo "=== Build successful ==="
echo "Binaries in: $BUILD_DIR"

# Archive a timestamped .tar.xz (.bin + .elf) so a crash's exact build
# survives the next compile — build/sketch/ itself is overwritten in place
# every run, with no version history, same problem the main project's
# compile.sh/compile_fast.sh had (see their matching comment). Bundled+xz'd
# instead of plain-copied (also 2026-08-25, same day, once plain copies made
# this directory grow ~19MB/build — .elf carries full debug symbols; xz -6
# gets that down to ~4.6MB/build). decode_crash.sh (repo root, one level up)
# knows how to pull the .elf back out of these for backtrace decoding.
ARCHIVE_DIR="$SCRIPT_DIR/build_archive"
mkdir -p "$ARCHIVE_DIR"
STAMP=$(date +%Y%m%d_%H%M%S)
tar -C "$BUILD_DIR" --use-compress-program="xz -6 -T0" \
    -cf "$ARCHIVE_DIR/display_${STAMP}.tar.xz" \
    display.ino.bin $( [ -f "$BUILD_DIR/display.ino.elf" ] && echo display.ino.elf )
echo "Archived build -> $ARCHIVE_DIR/display_${STAMP}.tar.xz"
