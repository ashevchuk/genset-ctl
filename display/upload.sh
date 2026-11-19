#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SKETCH="$SCRIPT_DIR"
PORT="${1:-/dev/ttyACM0}"

# See compile.sh's own comment — same env-var-first, PATH-then-IDE-install
# discovery, no fixed version/username.
ARDUINO_CLI="${ARDUINO_CLI:-$(command -v arduino-cli 2>/dev/null || true)}"
if [ -z "$ARDUINO_CLI" ]; then
    ARDUINO_CLI=$(ls -1 "$HOME"/devel/arduino-ide_*_Linux_64bit/resources/app/node_modules/arduino-ide-extension/build/arduino-cli 2>/dev/null | sort -V | tail -1)
fi
if [ -z "$ARDUINO_CLI" ] || [ ! -x "$ARDUINO_CLI" ]; then
    echo "ERROR: arduino-cli not found. Install it (or the Arduino IDE), or set ARDUINO_CLI=/path/to/arduino-cli." >&2
    exit 1
fi

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

# Check if pre-compiled binary exists; if not, compile first
BINARY="$BUILD_DIR/display.ino.bin"
if [ ! -f "$BINARY" ]; then
    echo "No compiled binary found — compiling first..."
    "$SCRIPT_DIR/compile.sh"
fi

echo "=== Uploading to $PORT ==="

"$ARDUINO_CLI" upload \
    --fqbn "$FQBN" \
    --port "$PORT" \
    --input-dir "$BUILD_DIR" \
    --verbose

echo ""
echo "=== Upload complete ==="
echo "Monitor: screen $PORT 115200"
