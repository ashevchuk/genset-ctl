#!/bin/sh
# Retired 2026-09-23 — arduino-builder is no longer the build for this
# project. esp32-gen-ctl-ai-rtos.ino and every retired root script now live
# in retired/ (kept, not deleted — this repo has no git, so "delete" would
# mean "gone"). This script's own original content is preserved the same
# way: retired/compile.sh.retired-20260923.
#
# Use instead: cd idf && ./compile.sh
echo "compile.sh is retired — arduino-builder is no longer the build for this project." >&2
echo "Use: cd idf && ./compile.sh" >&2
exit 1
