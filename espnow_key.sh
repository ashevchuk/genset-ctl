#!/usr/bin/env bash
# Pair a display panel with a generator over ESP-NOW (see ESPNOW.md):
# copies the generator's fleet key to the panel, then selects that generator
# on the panel by its hostname. The key is read from the generator's own
# /wifi.json (set it on the generator's /wifi-config page first) and is never
# printed or put on a command line.
#
#   ./espnow_key.sh <generator-host> <panel-host> [--key-only]
#
# --key-only: copy the key but leave the panel's generator selection alone.
set -euo pipefail

usage() { echo "usage: $0 <generator-host> <panel-host> [--key-only]" >&2; exit 2; }
[ $# -ge 2 ] || usage
GEN="$1"; PANEL="$2"; KEY_ONLY=0
[ "${3:-}" = "--key-only" ] && KEY_ONLY=1
[ $# -le 3 ] || usage
PANEL_URL="http://$PANEL:8080/espnow"

die() { echo "ERROR: $*" >&2; exit 1; }

# Reads JSON on stdin, prints the requested fields one per line; exits
# nonzero (with the reason on stderr) on invalid JSON or a missing field.
json_get() {
    python3 -I -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception as e:
    sys.exit("invalid JSON: %s" % e)
for path in sys.argv[1:]:
    v = d
    for k in path.split("."):
        if not isinstance(v, dict) or k not in v:
            sys.exit("missing field: " + path)
        v = v[k]
    print(json.dumps(v) if isinstance(v, (dict, list, bool)) else v)
' "$@"
}

echo "Reading ESP-NOW config from generator $GEN ..."
cfg=$(curl -sS --fail -m 10 "http://$GEN/api/wifi/config") || die "cannot read http://$GEN/api/wifi/config"
fields=$(printf '%s' "$cfg" | json_get hostname espnow.enabled espnow.key) \
    || die "generator config has no usable espnow section (enable ESP-NOW on its /wifi-config page)"
{ read -r GEN_NAME; read -r GEN_ENABLED; read -r KEY; } <<< "$fields"
unset cfg fields
[ "$GEN_ENABLED" = "true" ] || die "ESP-NOW is disabled on $GEN (enable it on its /wifi-config page)"
[[ "$KEY" =~ ^[0-9a-fA-F]{32}$ ]] || die "generator's espnow.key is not 32 hex characters"
echo "  generator name: $GEN_NAME"

echo "Writing key to panel $PANEL ..."
printf '%s\n' "$KEY" | "$(dirname "$0")/display/set_espnow_key.sh" "$PANEL" || die "writing the key to the panel failed"
unset KEY

[ "$KEY_ONLY" = 1 ] && { echo "Done (--key-only)."; exit 0; }

# Beacons arrive every 2 s; the panel lists generators heard in the last 10 s.
echo "Waiting for the panel to hear \"$GEN_NAME\" ..."
GEN_MAC=""
for _ in $(seq 1 15); do
    status=$(curl -sS --fail -m 5 "$PANEL_URL") || die "cannot read $PANEL_URL"
    GEN_MAC=$(printf '%s' "$status" | python3 -I -c '
import json, sys
name = sys.argv[1]
try:
    d = json.load(sys.stdin)
except Exception as e:
    sys.exit("invalid JSON: %s" % e)
if "generators" not in d:
    sys.exit("missing field: generators")
macs = [g["mac"] for g in d["generators"] if g.get("name") == name and g.get("seen")]
if len(macs) > 1:
    sys.exit("more than one generator named %r is on the air: %s" % (name, macs))
print(macs[0] if macs else "")
' "$GEN_NAME") || die "unexpected panel status"
    [ -n "$GEN_MAC" ] && break
    sleep 1
done
[ -n "$GEN_MAC" ] || die "panel does not hear \"$GEN_NAME\" (out of range, different WiFi channel, or the generator has not rebooted since ESP-NOW was enabled)"
echo "  heard at $GEN_MAC"

echo "Selecting \"$GEN_NAME\" on the panel ..."
curl -sS --fail -m 10 -o /dev/null --data-urlencode "gen_mac=$GEN_MAC" \
     --data-urlencode "gen_name=$GEN_NAME" "$PANEL_URL" || die "selecting the generator failed"
for _ in $(seq 1 15); do
    status=$(curl -sS --fail -m 5 "$PANEL_URL") || die "cannot read $PANEL_URL"
    active=$(printf '%s' "$status" | json_get active) || die "unexpected panel status"
    [ "$active" = "true" ] && { echo "Done: panel $PANEL receives encrypted state from \"$GEN_NAME\"."; exit 0; }
    sleep 1
done
die "generator selected, but no encrypted state arrived within 15 s (key mismatch?)"
