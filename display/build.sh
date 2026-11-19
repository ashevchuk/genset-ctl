#!/bin/sh
# Retired 2026-09-24 — this was a one-off arduino-builder invocation
# hardcoded to a specific machine's ~/.arduino15 toolchain paths and a
# /tmp/arduino_build_706795 scratch dir from a single past compile session,
# never documented as an actual entry point. compile.sh (arduino-cli based,
# portable) replaced it. Original content preserved as
# ../retired/display-build.sh.retired-20260924.
#
# Use instead: ./compile.sh
echo "build.sh is retired — a stale one-off arduino-builder invocation, never the real build for this project." >&2
echo "Use: ./compile.sh" >&2
exit 1
