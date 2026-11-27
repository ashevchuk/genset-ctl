# Home Assistant — Generator Controller (MQTT)

MQTT package for the ESP32 genset controller. Matches the wire format in repo-root [`MQTT.md`](../MQTT.md). No custom integration / HACS component — plain MQTT entities under one device.

## Prerequisites

1. Controller MQTT enabled (`/wifi-config` → MQTT): broker host/port, optional user/password, `topic_prefix` (default `gen-ctl`).
2. Home Assistant MQTT integration connected to the **same** broker.
3. The broker accepts the controller's MQTT user. With the Mosquitto add-on that means a `logins:` entry in the add-on's configuration (or a Home Assistant user) with exactly the username/password set in `/wifi-config`. If it doesn't, the add-on log shows `Client gen-ctl-XXXXXX … disconnected: not authorised.` every ~5 s (the controller's reconnect interval) and every entity stays `unknown` — nothing on the controller side reports it.
4. ACL: HA and the controller can publish/subscribe on `{prefix}/#` (controller subscribes `{prefix}/cmd/#`, publishes `state` / `event` / `metrics`).
5. Home Assistant with MQTT `default_entity_id` support. The package pins entity IDs with it; the older `object_id` key is rejected outright on current releases (verified on 2026.11.1).

## Install

### Option A — packages (recommended)

```yaml
# configuration.yaml
homeassistant:
  packages: !include_dir_named packages
```

Copy (or symlink) this folder’s package into HA:

```bash
cp ha/packages/gen_ctl.yaml /config/packages/gen_ctl.yaml
```

Reload: **Developer Tools → YAML → Reload all** (or restart HA).

### Option B — direct include

```yaml
# configuration.yaml
mqtt: !include gen_ctl_mqtt.yaml
```

…after copying `ha/packages/gen_ctl.yaml` and keeping only the contents under the top-level `mqtt:` key (or wrap with `mqtt:` yourself). Prefer Option A.

### Lovelace

Import `ha/lovelace/gen_ctl_dashboard.yaml` as a new dashboard (raw config) or merge the `views:` block into an existing dashboard.

### Example automations

Optional: merge `ha/automations/gen_ctl_examples.yaml` into your automations, or place under `packages/` if you use packages for automations too. Edit notify targets before enabling.

## Topic prefix

All entities use prefix `gen-ctl`. If you changed `topic_prefix` on the controller, replace every `gen-ctl/` in `gen_ctl.yaml` (and the dashboard entity IDs stay the same), e.g. for prefix `generator`:

```bash
sed 's#"gen-ctl/#"generator/#g' ha/packages/gen_ctl.yaml > /config/packages/gen_ctl.yaml
```

Check the controller's actual value first (`GET http://<controller>/api/wifi/config` → `mqtt.topic_prefix`) — a mismatch fails silently: entities stay `unknown` and buttons/switches publish to topics nobody subscribes to.

## Entities (summary)

| Kind | Examples |
|---|---|
| Sensors | state, cooldown left, RPM, frequency, battery, fuel, head temp, Vrms, power, RSSI, heap, lifetime hours/starts |
| Binary sensors | running, AC output, gen sense, oil fault, overload |
| Switches | choke, fuel pump, ECO, load relay, charger |
| Buttons | start, start+retry, stop; maintenance record (oil / spark / filters / valves / other) |
| Metrics | retained `…/metrics` lifetime counters + per-type hours-since / due |
| Event | last journal event type + JSON attributes |

Availability: sensors expire after 30 s if `…/state` stops (controller publishes ~every 5 s). Metrics use 180 s (60 s publish).

## Register switches and header toggles

All entities share one device (`Generator`). Home Assistant's auto-generated dashboard (the default Overview, until you "take control" of it) builds one entities card per device and gives any card with two or more toggleable entities a **header toggle** — here that single toggle flips choke, fuel pump, ECO, load relay and charger together, in one tap, bypassing the controller's start/stop sequencing. The auto-generated dashboard has no option to turn this off.

- The dashboard in `lovelace/` sets `show_header_toggle: false` on every entities card. Keep it that way if you edit it.
- To keep the register switches on the Overview as ordinary per-row switches, take control of the Overview (Edit dashboard → Take control), then set `show_header_toggle: false` on the card holding the `switch.gen_ctl_*` entities.
- Alternatively, adding `entity_category: config` to the five switches removes them from auto-generated dashboards entirely (they remain on the device page and in `lovelace/`).

## Safety notes

- **Start / Start retry** are unauthenticated MQTT buttons — treat broker ACL and network trust the same as the web UI.
- Manual register switches (`choke`, `fuel_pump`, …) bypass high-level start sequencing except `REG_START`, which MQTT cannot command (same as HTTP `/reg`).
- Prefer **Start** / **Stop** for normal operation; use register switches only when you know the state machine implications.
- With cooldown enabled on the controller, **Stop** from running may leave the engine running unloaded for a while (state `cooldown`, `binary_sensor.gen_ctl_running` off). A second **Stop** stops it at once, **Start** puts the load back on. An automation that sends `stop` twice in a row therefore skips the cooldown.

## Quick test

```bash
mosquitto_sub -h <broker> -t 'gen-ctl/#' -v
mosquitto_pub -h <broker> -t gen-ctl/cmd/stop -m 1
```
