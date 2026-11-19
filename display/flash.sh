#!/bin/sh
# Retired 2026-09-24 — a raw esptool invocation hardcoded to a specific
# machine's ~/.arduino15 esptool/boot_app0.bin paths and a
# /tmp/arduino_build_706795 scratch dir from a single past compile session
# (long gone). upload.sh (arduino-cli based, portable, auto-compiles if
# needed) is the real flash path. Original content preserved as
# ../retired/display-flash.sh.retired-20260924.
#
# Use instead: ./upload.sh [port]
echo "flash.sh is retired — a stale one-off esptool invocation, never the real flash path for this project." >&2
echo "Use: ./upload.sh [port]" >&2
exit 1
