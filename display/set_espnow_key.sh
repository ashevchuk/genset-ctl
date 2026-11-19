#!/usr/bin/env bash
# Write the ESP-NOW fleet key to a display panel (see ../ESPNOW.md). The key
# comes from stdin when piped, otherwise from a hidden prompt — never from
# the command line, and it is never printed.
#
#   ./set_espnow_key.sh [panel-host]          # default 10.100.100.204
#   some_cmd | ./set_espnow_key.sh [panel-host]
#
# To copy the key from a generator and select it in one go, use the repo
# root's espnow_key.sh instead.
set -euo pipefail

PANEL="${1:-192.168.10.2}"
URL="http://$PANEL:8080/espnow"

die() { echo "ERROR: $*" >&2; exit 1; }

if [ -t 0 ]; then
    read -r -s -p "ESP-NOW fleet key (32 hex characters): " KEY
    echo >&2
else
    read -r KEY || true
fi
[[ "$KEY" =~ ^[0-9a-fA-F]{32}$ ]] || die "key must be exactly 32 hex characters"

resp=$(printf 'key=%s' "$KEY" | curl -sS -m 10 -X POST \
        -H 'Content-Type: application/x-www-form-urlencoded' --data-binary @- \
        -w '\n%{http_code}' "$URL") || die "cannot reach $URL"
unset KEY
code=${resp##*$'\n'}; body=${resp%$'\n'*}
[ "$code" = "200" ] || die "panel rejected the key (HTTP $code): $body"

enabled=$(printf '%s' "$body" | python3 -I -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception as e:
    sys.exit("invalid JSON from panel: %s" % e)
if "enabled" not in d:
    sys.exit("missing field: enabled")
print("true" if d["enabled"] is True else "false")
') || die "unexpected panel response"
[ "$enabled" = "true" ] || die "panel did not enable ESP-NOW: $body"
echo "Key stored on panel $PANEL, ESP-NOW enabled."
