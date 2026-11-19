#!/bin/sh
# Retired 2026-09-23 — arduino-builder is no longer the build for this
# project. Original content preserved as retired/burn.sh.retired-20260923.
#
# Use instead: cd idf && ./burn.sh
# host/host_test/host_gen files are unaffected by this retirement — still
# read the same way by idf/burn.sh.
echo "burn.sh is retired — arduino-builder is no longer the build for this project." >&2
echo "Use: cd idf && ./burn.sh" >&2
exit 1
