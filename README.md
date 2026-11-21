# genset-ctl-freertos

Firmware for an ESP32-based generator (genset) controller: automatic
start/stop state machine, shift-register relay I/O, sensor monitoring, a
web dashboard, WebSocket and MQTT interfaces, and a persistent FRAM event
journal. A second, mostly independent sub-project (`display/`) is an
ESP32-S3 + LVGL touch panel that talks to the controller over HTTP/WebSocket,
with an ESP-NOW fallback link and LAN auto-discovery for when the network
path isn't available.

![GeneratorCTL tiny PCB](doc/production/pcb.png)

## Hardware

- **Controller**: ESP32 (D0WD-Q5), 16MB flash, dual OTA app partitions.
  - Output shift register (`SN74HC595N`) drives ignition, starter, choke,
    fuel pump, load relay, charger, etc.
  - Input shift register (`CD4021BE`) reads protection signals (overload,
    oil, overheat, output-present, PSU/generator/battery sense, a jumper
    for recovery mode).
  - Sensors: 2x ADS1115 (ADC), INA233 (battery voltage), MCP9600
    (head-temperature thermocouple), MCP39F511A (AC power/energy meter over
    UART), a tachometer input, DS3231 RTC, an SSD1306 OLED + rotary encoder
    for local control, and an FM24V10 FRAM chip for the event journal.
- **Display panel** (optional, `display/`): ESP32-S3 with an LVGL touch
  screen, showing live state, a journal view, and remote-control hardware
  I/O (physical start/stop buttons, autostart trigger, status outputs).

> **A note on the bill of materials.** The component selection for this
> board was not driven by engineering merit, cost, or availability. It was
> driven by a pressing need to finally do *something* with a drawer full of
> aging parts that had been "too good to throw away" for years. If you are
> building your own, please do not treat this BOM as a recommendation:
> pick modern parts on purpose instead of inheriting someone else's
> stockpile by accident.

## Features

- Generator auto start/stop state machine with retry logic, emergency-stop
  classification, and configurable safety limits (temperature, power).
- Web dashboard (HTTP + WebSocket on port 80) with live gauges/charts,
  manual register control, and a WiFi/MQTT/safety-limits configuration page.
- MQTT publish/subscribe interface, including a ready-made Home Assistant
  integration package (`ha/`).
- A 4080-record FRAM event journal (engine runs, charge cycles, alarms,
  maintenance, system events) with a paged/filterable web UI and lifetime
  metrics (run hours, starts, max readings, maintenance-due tracking).
- OTA firmware updates with bootloader-level rollback-on-crash protection,
  plus a one-time, normally-disabled endpoint for updating the bootloader
  itself on units with no physical USB access (see `BOOTLOADER-UPDATE.md`).
- A display panel with its own live WebSocket view, journal browsing, and
  hardware remote start/stop/autostart I/O.
- An encrypted ESP-NOW link and LAN UDP discovery so the display panel can
  find and control a controller without typing in an IP, including a
  fallback path when the WiFi network itself is down (see `ESPNOW.md`).

## Repository layout

| Path | What it is |
|---|---|
| `idf/` | ESP-IDF/`idf.py` build for the main controller (the sole build system; see `CLAUDE.md` for the full build-command reference) |
| `*.cpp` / `*.h` (repo root) | The controller's own source: per-subsystem modules (state machine, web/WS, MQTT, FRAM journal, device drivers, ESP-NOW, ...) |
| `data/` | Web dashboard static files (HTML/JS/CSS), served from the controller's filesystem |
| `display/` | ESP32-S3 + LVGL touch-panel firmware (Arduino/`arduino-cli` build) |
| `ha/` | Home Assistant MQTT integration package |
| `mqtt/` | Local Mosquitto broker config for testing the MQTT interface without a real broker |
| `doc/` | Datasheets and reference material for the hardware this firmware drives |
| `doc/production/` | PCB production files for the GeneratorCTL tiny board: KiCad project export (schematic + routed 4-layer PCB) and the Gerber/drill set for fab ordering |
| `retired/` | Superseded scripts/sources kept for reference, not part of the active build |

## Building and flashing

Requires ESP-IDF (`source <idf_path>/export.sh`) for the controller, and
`arduino-cli` for the display panel. See `CLAUDE.md` for the full set of
build variants (debug/nodebug, bootloader-update) and `idf/sdkconfig.defaults`
for the non-default Kconfig options this project needs.

**Main controller:**
```sh
cd idf
./compile.sh          # idf.py build, then archives the binary
./flash.sh [port]     # USB flash
./burn.sh             # OTA update over HTTP (target read from repo-root `host`)
```

**Display panel:**
```sh
cd display
./compile.sh
./upload.sh [port]
```

There is no automated test suite — verification is: compile clean, then
flash to real hardware and exercise it over serial/HTTP. This firmware
drives a running internal-combustion generator; treat changes to the state
machine or safety-limit logic accordingly.

## Interfaces

- **HTTP + WebSocket**: `http://<host>/` (dashboard), `ws://<host>/ws`
  (live state). See `CLAUDE.md`'s "Web/MQTT interfaces" section for the
  endpoint list.
- **MQTT**: topic structure, JSON schema and the state enum are documented
  in `MQTT.md`.
- **Home Assistant**: a ready-made MQTT package (entities, Lovelace
  dashboard, example automations) is in `ha/`, see `ha/README.md`.
- **ESP-NOW / LAN discovery**: wire format, provisioning and security model
  for the display panel's fallback link are in `ESPNOW.md`.
- **Bootloader update endpoint**: a rare, deliberately-disabled-by-default
  operation for USB-less units, documented in `BOOTLOADER-UPDATE.md`.

## Documentation

- `CLAUDE.md` — the detailed, continuously-maintained technical reference:
  full architecture, build internals, task layout, every subsystem, and the
  non-obvious gotchas found while building this. Written for contributors
  and AI coding assistants working in this repo.
- `MQTT.md`, `ESPNOW.md`, `BOOTLOADER-UPDATE.md`, `ha/README.md` — interface
  references for their respective subsystems.
- `display/README.md` — Arduino IDE board configuration for the display
  panel.

## License

MIT — see `LICENSE`.
