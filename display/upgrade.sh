#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BIN="$SCRIPT_DIR/build/sketch/display.ino.bin"
OTA_HOST="${1:-192.168.10.2}"
OTA_PORT="${2:-8080}"
OTA_URL="http://$OTA_HOST:$OTA_PORT/update"

# Step 1: Compile
echo "=== Compiling ==="
bash "$SCRIPT_DIR/compile.sh"

# Step 2: Upload via HTTP OTA
echo ""
echo "=== Uploading firmware via OTA to $OTA_URL ==="
if [ ! -f "$BIN" ]; then
    echo "ERROR: Binary not found: $BIN"
    exit 1
fi

curl --fail \
     --progress-bar \
     -F "image=@$BIN" \
     "$OTA_URL" \
&& echo "" \
&& echo "=== OTA upload successful. Device is rebooting. ===" \
|| { echo ""; echo "ERROR: OTA upload failed."; exit 1; }
