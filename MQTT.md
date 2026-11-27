# MQTT Interface — ESP32 Generator Controller

## Overview

The generator controller exposes a full MQTT interface for remote monitoring and control. MQTT is optional and can be enabled or disabled via the WiFi configuration page (`/wifi-config`). Settings are persisted in `/wifi.json` on the FFat filesystem.

MQTT (ESP-IDF's `esp-mqtt` client, managed component `espressif/mqtt`, its own TCP connection to the broker) is independent of the controller's HTTP/WebSocket web interface — unaffected by the changes to that stack (now ESP-IDF `esp_http_server` on port 80, WebSocket at `ws://<host>/ws`). Commands and state converge on the same `xWebCommandQueue`/`StateMachineTask` path described below regardless of which interface they arrive through.

All topics share a configurable **topic prefix** (default: `gen-ctl`). Examples below use `gen-ctl` as the prefix.

---

## Configuration

Settings are stored in `wifi.json` under the `mqtt` key and are edited via the web UI at `http://<device>/wifi-config`.

| Parameter | Default | Description |
|---|---|---|
| `enabled` | `false` | Enable/disable MQTT |
| `host` | _(empty)_ | Broker IP or hostname |
| `port` | `1883` | Broker TCP port |
| `user` | _(empty)_ | Username (leave empty for anonymous) |
| `password` | _(empty)_ | Password |
| `client_id` | `gen-ctl` | MQTT client identifier |
| `topic_prefix` | `gen-ctl` | Prefix for all topics |

Example `wifi.json` fragment:
```json
"mqtt": {
  "enabled": true,
  "host": "192.168.1.100",
  "port": 1883,
  "user": "mqttuser",
  "password": "secret",
  "client_id": "gen-ctl",
  "topic_prefix": "gen-ctl"
}
```

---

## Command Topics (subscribe from broker side)

The controller subscribes to `{prefix}/cmd/#`.

### Generator start / stop

| Topic | Payload | Description |
|---|---|---|
| `gen-ctl/cmd/start` | _(any)_ | Start generator (single attempt) |
| `gen-ctl/cmd/start_retry` | _(any)_ | Start generator with auto-retry (up to 5 attempts) |
| `gen-ctl/cmd/stop` | _(any)_ | Stop generator |

### Output register control

| Topic | Payload | Register | Description |
|---|---|---|---|
| `gen-ctl/cmd/choke` | `0` / `1` / `toggle` | `REG_CHOKE` | Choke valve |
| `gen-ctl/cmd/fuel_pump` | `0` / `1` / `toggle` | `REG_FUEL_PUMP` | Fuel pump relay |
| `gen-ctl/cmd/eco_mode` | `0` / `1` / `toggle` | `REG_ECO_MODE` | ECO mode (governor) |
| `gen-ctl/cmd/load_relay` | `0` / `1` / `toggle` | `REG_LOAD_RELAY` | Load relay (AC output) |
| `gen-ctl/cmd/charger` | `0` / `1` / `toggle` | `REG_CHARGER` | Battery charger |

**Payload values:**
- `1` — turn ON
- `0` — turn OFF
- `toggle` — invert current state
- anything else (`off`, `false`, `ON`, garbage, ...) — ignored, no action taken. Strict match only; there is no fuzzy/truthy fallback, so don't send anything but the three exact values above.

Commands are internally routed through the same `xWebCommandQueue` as the WebSocket and web UI commands, and are processed by `StateMachineTask`.

### Maintenance log entry

| Topic | Payload | Description |
|---|---|---|
| `gen-ctl/cmd/maintenance` | `oil` / `spark_plug` / `air_filter` / `fuel_filter` / `valves` / `other` (or index `0`–`5`) | Record a maintenance event in the FRAM journal with current engine hours |

---

## State Topic (published by controller)

### `gen-ctl/state`

Published **every 5 seconds** or **immediately on generator state change**. Payload is a JSON object. QoS 0, retain `false`.

```json
{
  "state":        "running",
  "state_id":     10,
  "is_running":   1,
  "rpm":          3000,
  "freq":         50.0,
  "bat_v":        12.85,
  "fuel":         5.2,
  "head_temp":    118.5,
  "reg_choke":    0,
  "reg_fuel_pump":1,
  "reg_eco_mode": 1,
  "reg_load_relay":1,
  "reg_charger":  0,
  "in_output":    1,
  "in_gen_sense": 1,
  "in_oil":       0,
  "in_overload":  0,
  "vrms":         221.3,
  "power":        1450.0,
  "rssi":         -62,
  "run_hours":    152.35,
  "starts":       210,
  "heap":         148392,
  "min_heap":     121044
}
```

#### State fields

| Field | Type | Description |
|---|---|---|
| `state` | string | Human-readable generator state (see table below) |
| `state_id` | int | Numeric state ID (enum value) |
| `is_running` | 0/1 | `1` when state is `running` (`STATE_RUNNING`); `0` during `cooldown`, although the engine is still running |
| `cooldown_left` | int | Seconds until the cooldown cap (`timing.cooldown_max_s`) while state is `cooldown`, else `0`. An upper bound: cooldown ends earlier once the head temperature is at or below `timing.cooldown_temp_c` |
| `rpm` | int | Engine RPM (PCNT tachometer, smoothed) |
| `freq` | float | AC output frequency, Hz (MCP39F511A power meter) |
| `bat_v` | float | Battery voltage, V (INA233 / ADC) |
| `fuel` | float | Fuel level, litres (interpolated from ADC curve) |
| `head_temp` | float | Cylinder head temperature, °C (MCP9600 thermocouple) |
| `reg_choke` | 0/1 | Choke valve output state |
| `reg_fuel_pump` | 0/1 | Fuel pump relay output state |
| `reg_eco_mode` | 0/1 | ECO mode output state |
| `reg_load_relay` | 0/1 | Load relay output state |
| `reg_charger` | 0/1 | Battery charger output state |
| `in_output` | 0/1 | AC output sense input |
| `in_gen_sense` | 0/1 | Generator sense input |
| `in_oil` | 0/1 | Oil pressure/level fault input |
| `in_overload` | 0/1 | Overload protection input |
| `vrms` | float | AC RMS voltage, V (MCP39F511A power meter) |
| `power` | float | Active power, W (MCP39F511A power meter) |
| `rssi` | int | WiFi signal strength, dBm |
| `run_hours` | float | Lifetime engine hours (FRAM metrics) |
| `starts` | int | Lifetime successful start count (FRAM metrics) |
| `heap` | int | Free heap, bytes (`ESP.getFreeHeap()`) |
| `min_heap` | int | Minimum free heap since boot, bytes (`ESP.getMinFreeHeap()`) |

#### Generator states

| `state` | `state_id` | Description |
|---|---|---|
| `init` | 0 | Boot / initial state |
| `idle` | 1 | Stopped, ready to start |
| `stopping` | 2 / 3 | Stopping in progress (2 = brief load-relay-off phase, ~500ms, before ignition is cut) |
| `stopping_retry` | 4 | Stopping after failed start retry |
| `stopped` | 5 | Confirmed stopped, transitioning to idle |
| `stopped_retry` | 6 | Stopped between retry attempts |
| `starting` | 7 / 8 | Start sequence active (starter motor cranking) |
| `starting_retry` | 9 / 10 | Auto-retry start sequence |
| `started` | 11 | Engine running, warming up |
| `running` | 12 | Engine running at operating speed, load connected |
| `error` | 13 | Error / fault state |
| `cooldown` | 14 | Engine running unloaded after a stop request, until the head cools (see below) |

`state_id` values are fixed: Home Assistant (`ha/`) and the display panel (ESP-NOW state packet) depend on them. New states are appended at the end only.

**Cooldown** (`timing.cooldown_*` in `/wifi.json`, off by default): a stop from `running` that is not a safety stop goes `stopping` (load off, ~500 ms) → `cooldown` → `stopping`, provided the head temperature is above `cooldown_temp_c`. The engine runs with load relay and ECO off for at least `cooldown_min_s`, then until the head is at or below `cooldown_temp_c` (with no head reading, until `cooldown_max_s`), and never longer than `cooldown_max_s`. During `cooldown`: a second `stop` stops at once; `start`/`start_retry` returns to `running` (load relay back per `limits.auto_load_relay_on_run`, no new `start_ok`) and logs `cooldown_cancel`; a protection input, a temperature limit or the engine dying stops at once and is logged as `emergency_stop`.

---

## Event Topic (published by controller)

### `gen-ctl/event`

Every record written to the FRAM event journal is mirrored here (QoS 0, retain `false`). Payload matches the HTTP `/log` record format:

```json
{"seq":153,"ts":1783012345,"time_valid":1,"type":"start_ok",
 "attempts":1,"detected":0,"head_temp":21.5,"bat_v":12.61,"crank_ms":2450}
```

Event `type` values: `boot`, `time_sync`, `start_request`, `start_ok`, `start_fail`, `stop_request`, `stopped` (`cooldown_s`: length of the cooldown run before it, 0 if none), `cooldown_cancel` (`source`, `elapsed_s`, `head_temp`), `emergency_stop` (reason: `oil`/`overheat`/`overload`/`stalled`), `charge_start`, `charge_stop` (reason: `threshold`/`psu_lost`/`manual`/`engine_start`), `error`, `task_wdt`, `fw_update`, `i2c_stuck`, `sensor_fail`, `maintenance`.

`boot` is logged once at the start of every `setup()`, before anything else can fail — `reset_reason` is the raw `esp_reset_reason_t` value (`esp_reset_reason()`, cast to `uint8_t`): `0`=unknown, `1`=power-on, `2`=external pin, `3`=`esp_restart()`/clean software reboot (what a normal OTA reboot reports), `4`=panic/crash (`abort()`, an uncaught exception, an `ESP_ERROR_CHECK()` failure, ...), `5`=interrupt watchdog, `6`=task watchdog, `7`=other watchdog, `8`=deep-sleep wake, `9`=brownout — see `resetReasonName()` in the `.ino` for the same mapping to human-readable names (used in the OLED status screen, not this journal). `4` (panic) here is the fastest way to notice something aborted on the previous boot even without a `task_wdt` record (e.g. an `ESP_ERROR_CHECK()` failure, which isn't a watchdog trip and gets no `task_wdt` record of its own) — cross-check against `GET /coredump`/`decode_crash.sh` for the actual backtrace:

```json
{"seq":307,"ts":1789928287,"time_valid":1,"type":"boot","reset_reason":4}
```

`task_wdt` fires when the ESP-IDF task watchdog (15 s) trips on the previous boot — a task didn't reset it in time and the chip rebooted via panic. Payload carries `uptime_s` (seconds since boot when it happened) and `tasks` (best-effort truncated text naming the starved task, from `esp_task_wdt_print_triggered_tasks`, captured in RTC no-init memory so it survives the abort+reset and gets logged once on the next boot):

```json
{"seq":231,"ts":1784500012,"time_valid":1,"type":"task_wdt","uptime_s":184,"tasks":"Mqtt (CPU 0)"}
```

`fw_update` is logged right before the reboot that follows a successful OTA upload via `POST /update` (normal operation mode only — the reduced JP1-jumper OTA recovery mode has no FRAM journal running). Payload carries `size` (total bytes written):

```json
{"seq":245,"ts":1784511000,"time_valid":1,"type":"fw_update","size":1272245}
```

All I2C hardware (ADS1115×2, INA233, MCP9600, DS3231 RTC, SSD1306 OLED, FM24V10 FRAM) is owned exclusively by a single `I2CTask` — no other task ever calls `Wire` directly. `i2c_stuck` fires once per episode when `I2CTask`'s sensor cache (`g_state.sensors.timestamp`, refreshed every ~500ms by its own periodic read) hasn't moved for 5+ consecutive seconds, checked from `ReportTask`'s existing 1Hz tick — a symptom that `I2CTask` itself has stalled (a genuinely wedged I2C device, or a long-running on-demand job such as a FRAM `/log/wipe`). Payload carries only `stuck_ms`:

```json
{"seq":250,"ts":1784903635,"time_valid":1,"type":"i2c_stuck","stuck_ms":5000}
```

`sensor_fail` fires from `I2CTask`'s periodic sensor read the first time a sensor read fails after a previously-OK read (edge-triggered — logged once per failure episode, not on every failed 500ms poll, so a device that stays down doesn't flood the log). Currently covers `mcp9600` (thermocouple) and `ina233` (battery voltage monitor) — both read with a single I2C attempt and no retry loop, unlike an earlier firmware version that retried MCP9600 reads up to 4 times per call, which could stall the bus for over a second on a genuinely unresponsive chip. Payload carries `sensor`:

```json
{"seq":256,"ts":1785266388,"time_valid":1,"type":"sensor_fail","sensor":"mcp9600"}
```

### `gen-ctl/metrics`

Lifetime metrics, published every 60 seconds with **retain = true**:

```json
{"run_hours":152.35,"run_seconds":548460,
 "charge_hours":12.05,"charge_seconds":43380,
 "starts":210,"start_fails":4,"emergencies":1,"charge_cycles":58,
 "max_head_temp":142.5,"max_rpm":3400,"max_power":2100,
 "maintenance":{"oil":{"ts":1782000000,"at_hours":120.0,"hours_since":32.4,"due":false}, "...":{}}}
```

`GET /metrics/set?confirm=1&<key>=<value>` overwrites any of the numeric fields above (same key names; `hours` is accepted as an alias for `run_hours`). Only supplied keys are written. Hours are stored as rounded seconds. Counts and peaks must be `>= 0`.

`maintenance.<type>.due` (bool) is present only for types with a nonzero
interval configured under `limits.maint_interval` in `/wifi-config` — `true`
once `hours_since` reaches that interval. Same threshold drives the
`MAINT_DUE` dashboard banner over WebSocket; this is the MQTT mirror of it.
The WebSocket report also carries `MAINT_DUE_MASK` (uint8, bit *i* set ⇒
`FLOG_MAINT_*` index *i* overdue) alongside the human-readable `MAINT_DUE`
string — the machine-readable form the `display/` touch panel's Controls-tab
Record button acts on, since the string is free text and not safe to parse
back into a subtype index.

---

## Internal Architecture

```
MQTT Broker
    │
    │  subscribe gen-ctl/cmd/#
    ▼
MqttTask (Core 0, 5 KB stack)
    │  mqtt_callback()
    │  xQueueSend → xWebCommandQueue
    ▼
StateMachineTask (Core 0)
    │  start/stop/reg commands
    ▼
ShiftRegTask → Hardware outputs

MqttTask also reads g_state (spinlock) and
sensor smoothed values every 100 ms,
publishes gen-ctl/state every 5 s or on state change.
```

- **Thread safety**: `esp-mqtt` runs its own client task; incoming `cmd/#` messages are handled there and only posted to `xWebCommandQueue` (FreeRTOS queue) or the journal queue, never acted on directly. `MqttTask` only publishes. State snapshot is taken under `g_state_spinlock` (portENTER_CRITICAL).
- **Reconnect**: the `esp-mqtt` client reconnects on its own every 5 seconds and resubscribes on each connect; `MqttTask` publishes only while connected. Each publish is bounded by a 5 s network timeout.
- **Watchdog**: MqttTask is registered with the ESP32 Task WDT (15 s timeout); disabled MQTT still feeds the WDT in its idle loop.

---

## Home Assistant

Full MQTT package (entities, Lovelace dashboard, example automations): repo directory [`ha/`](ha/README.md). Copy `ha/packages/gen_ctl.yaml` into Home Assistant `packages/`, then optionally import `ha/lovelace/gen_ctl_dashboard.yaml` and merge `ha/automations/gen_ctl_examples.yaml`.

Before anything else: the topics below must use the controller's actual `topic_prefix` (`GET /api/wifi/config` → `mqtt.topic_prefix`), and the broker must accept the controller's MQTT user — see `ha/README.md` "Prerequisites". Either mismatch fails silently on the HA side (entities stay `unknown`, commands go nowhere).

Minimal smoke-test entities — for checking the wire format only, **not** an install. They have no `unique_id` (not editable in the UI, not grouped into a device), cover a fraction of the state fields, and were once found running as a site's whole generator integration, against the wrong prefix. Use the `ha/` package for anything real:

```yaml
mqtt:
  sensor:
    - name: "Generator State"
      state_topic: "gen-ctl/state"
      value_template: "{{ value_json.state }}"

    - name: "Generator Power"
      state_topic: "gen-ctl/state"
      value_template: "{{ value_json.power }}"
      unit_of_measurement: "W"
      device_class: power

  button:
    - name: "Generator Start"
      command_topic: "gen-ctl/cmd/start"
      payload_press: "1"

    - name: "Generator Stop"
      command_topic: "gen-ctl/cmd/stop"
      payload_press: "1"

  switch:
    - name: "Generator Load Relay"
      state_topic: "gen-ctl/state"
      value_template: "{{ value_json.reg_load_relay }}"
      command_topic: "gen-ctl/cmd/load_relay"
      payload_on: "1"
      payload_off: "0"
      state_on: "1"
      state_off: "0"
```

---

## Quick Reference

```bash
# Start generator
mosquitto_pub -h 192.168.1.100 -t gen-ctl/cmd/start -m 1

# Stop generator
mosquitto_pub -h 192.168.1.100 -t gen-ctl/cmd/stop -m 1

# Enable load relay
mosquitto_pub -h 192.168.1.100 -t gen-ctl/cmd/load_relay -m 1

# Disable charger
mosquitto_pub -h 192.168.1.100 -t gen-ctl/cmd/charger -m 0

# Toggle choke
mosquitto_pub -h 192.168.1.100 -t gen-ctl/cmd/choke -m toggle

# Monitor state
mosquitto_sub -h 192.168.1.100 -t gen-ctl/state

# Monitor journal events / lifetime metrics
mosquitto_sub -h 192.168.1.100 -t gen-ctl/event
mosquitto_sub -h 192.168.1.100 -t gen-ctl/metrics

# Record an oil change (engine hours captured automatically)
mosquitto_pub -h 192.168.1.100 -t gen-ctl/cmd/maintenance -m oil
```
