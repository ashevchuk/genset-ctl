# ESP-NOW Fallback Link — ESP32 Generator Controller

## Overview

ESP-NOW is a direct, infrastructure-free radio link between generator controllers and display panels (`display/`). It is a fallback for when the WebSocket path through the WiFi router/AP is unavailable — it does **not** extend range or replace normal WiFi coverage. It rides on the same WiFi radio both devices already use for their STA connection.

**Protocol v2 (2026-10-16):**
- One **fleet key** for the whole installation, like a WiFi passphrase.
- Each controller announces itself by name (`wifi.json` `"hostname"`) in a broadcast beacon.
- A display lists the generators it hears and the operator picks one. The display then joins it with a hello, and from then on state and commands flow unicast and encrypted.
- The display learns the selected generator's IP from its encrypted state, so the WebSocket target follows the selection.
- Switching a panel from the test rig to another generator therefore needs no MAC or IP typing. Enable ESP-NOW with the same key on that controller, then pick its name on the panel.

- Controller: `esp_now_link.h`/`.cpp` (`EspNowTask`).
- Display: `display/esp_now_client.h`/`.cpp`, UI in `display/ui.cpp`.
- A separate, much simpler companion mechanism — LAN UDP discovery, no key, no radio pairing — finds a generator directly over the panel's existing WiFi/IP network instead. See CLAUDE.md's "LAN UDP discovery" section (controller: `gen_discovery.h`/`.cpp`; display: `display/udp_discovery.h`/`.cpp`).

WebSocket stays the primary path. On the display:
- Received state packets only write `g_gen` while WS isn't delivering (`!ws_client_fresh()`: no report for 2.5 s).
- `ws_send()` falls back to `espnow_send_text()` under the same condition. Every existing caller (touchscreen buttons, `remote_io.cpp`'s hardware buttons) gets the fallback for free.
- Top bar: the label reads `GEN: CONNECTED` — green on WS, blue on ESP-NOW — or `GEN: --`. While ESP-NOW carries the data, the signal bars switch from the panel's own WiFi to the ESP-NOW link's RSSI, drawn in blue. Orange already means "WiFi connecting", so there is still one signal indicator, not two.
- The Status tab's `RSSI:` row stays the controller's own WiFi RSSI and shows `--` on ESP-NOW.
- `remote_io`'s IO15 "link up" output is true on either link.

A controller accepts up to 5 joined displays (`ESP_NOW_MAX_ENCRYPT_PEER_NUM` is 6, one spare). A display follows one selected generator at a time.

---

## Security model

- **Encryption:** all state and command traffic is unicast and CCMP-encrypted. The fleet key is used as both the PMK (`esp_now_set_pmk()`) and every peer's LMK.
- **What is in the clear:** the beacon (generator name) and the hello (target MAC). Neither can control anything.
- **Hellos and peer slots:** anyone in radio range can send a hello and take a peer slot on a controller. A full table evicts the least recently heard display. That is a nuisance-level DoS, no worse than jamming 2.4 GHz.
- **Command acceptance:** commands are accepted only from MACs in the controller's encrypted peer list.
- **Verified live 2026-10-16:**
  - With a deliberately wrong key on the display, reception froze for 10+ s; the right key restored it.
  - **An unencrypted command sent from a joined display's own MAC (i.e. spoofing an encrypted peer) is dropped.** It was ACKed at the radio level (`tx_ok` counted it), yet the register did not change. The same toggle sent encrypted was applied. `esp_now_recv_info_t` carries no "was encrypted" flag, so this property comes from ESP-NOW itself, not from this code. It was checked with a temporary test route, now removed.
- **FreeRTOS footprint (measured 2026-10-16 with a temporary high-water-mark route, since removed):** `EspNowTask` peaked at ~1.3 KB of its 3 KB stack, including the display-join path (`esp_now_add_peer` + log line). The receive callbacks run in the WiFi driver task and only copy data under short spinlocks and post with `xQueueSend(..., 0)`. The controller's NVS reads/writes of the home channel take `xFlashI2CGuard`, like every other flash access racing I2C. The panel's command-resend state is shared by the LVGL task, `loop()` and the WiFi driver task, and is kept under one spinlock, so a resend can never send a newer packet such as a toggle.
- **Not verified:** replay of a captured encrypted frame across a reboot. Commands go through the same `StateMachineTask` gating as every other source, including the unconditional `REG_START` refusal.
- **Over-the-air names:** the beacon name is sanitized on the display (printable ASCII, no `"`/`\`). Otherwise it could break the status JSON or LVGL's newline-separated dropdown options.
- **Trust model:** the same as a WPA2 PSK — one compromised device exposes the key for the whole installation.

## Channel handling

ESP-NOW and the STA connection share one radio and must be on the same channel. Both peers use `channel = 0` ("whatever the interface is on"). While associated, that is the AP's channel. The hard case is the AP being off: **any STA connect attempt with the AP absent sweeps every channel**. The `channel` hint in `wifi_sta_config_t` only sets where the sweep *starts*, so the radio still leaves the shared channel. And arduino-esp32's own auto-reconnect retries `NO_AP_FOUND` back to back, so with stock settings the radio is almost never on the shared channel during an outage.

`espnow_wifi_supervise()` (controller `esp_now_link.cpp`, panel `esp_now_client.cpp`, same logic on both) handles it:

- **Home channel** = channel + SSID of the last AP association, saved in NVS namespace `espnow` (keys `ch` u8, `ssid` str; raw NVS API on the controller, `Preferences` on the panel — same on-flash format), so it survives reboots. It follows the AP automatically if the AP's channel is ever changed while both devices are connected. Saved only with a non-empty SSID: the associated SSID (`wifi_net_ssid()` on the controller, `WiFi.SSID()` on the panel) is already empty in the loop pass that first sees the drop, and saving that wiped the SSID and broke recovery (found in the first live outage test).
- **While disconnected**: the radio is parked on the home channel (`esp_wifi_set_channel`), and every 5 s a single-channel scan of just that channel (120 ms) looks for the SSID. The radio never leaves the shared channel for it.
- **SSID seen again** → reconnect immediately, without leaving the home channel. Panel: `WiFi.begin(ssid, pass, home)`. Controller: `wifi_net_multi_start(home)` — a single-channel scan, and the scanned AP's channel is passed into the STA config. Before that, the controller's full rescan right as the AP returned took its radio off the shared channel for ~4 s and blanked the panel during recovery.
- **Full all-channel retries still happen, but rarely** (AP moved): controller on its existing 10 s…5 min backoff, panel every 5 min instead of every 15 s.
- **Controller only, and only with ESP-NOW enabled**: `wifi_net_set_auto_reconnect(false)`, so back-to-back sweeps don't run. Recovery is driven by `loop()`'s `wifi_net_multi_start()` retry plus the probe. With ESP-NOW disabled (e.g. production today) WiFi behavior is unchanged.
- **Panel data switchover**: ESP-NOW state is applied whenever WS hasn't delivered a report for 2.5 s (`ws_client_fresh()`), not only once the socket is closed. A dead AP leaves the socket "connected" for several seconds until beacon/TCP timeouts fire; keying on "disconnected" blanked the screen for that window. `ws_send()` uses the same test, so a command never goes into a silent socket.
- **Dead-WS detection must not use `gen.valid`**: ESP-NOW keeps `g_gen` fresh, so the old "data stale → reconnect WS" check never fires while ESP-NOW is up. A half-open WS socket (seen after a controller reboot: "connected", nothing arriving) then kept the panel on ESP-NOW indefinitely with WiFi fine. The panel now reconnects when WS itself has been silent for 10 s (`ws_client_silent_ms()`) and runs a WS heartbeat (ping 15 s, 2 missed pongs). A plain controller reboot was verified to recover; the silent-socket path itself has only been built, not reproduced.
- **Command resend**: `start`/`start_retry`/`stop` are resent up to 3× at 300 ms if the send callback reports failure. A register toggle is never resent — if only the ACK was lost, a resend would flip it back.
- **Picking a generator the panel has never been near before, at a site with no AP at all**: the "home channel" above is the panel's *own* usual AP, not the generator's. A panel that has only ever associated with a different network parks on that network's channel while disconnected — irrelevant at a site with no AP in range — so selecting a generator there could see its beacon once (channel visited by chance) but never complete the hello/ACK handshake, because the radio kept getting pulled back to the wrong channel every retry. Found live 2026-10-17. Fixed in `display/esp_now_client.cpp`: the channel a beacon was last heard on is remembered per generator; selecting one pins the radio there (`esp_wifi_set_channel()`) and `espnow_wifi_join_pinned()` makes `espnow_wifi_supervise()` and the panel's own AP full-sweep retry both stand down while that pin holds, so neither fights the join. Bounded by `ESPNOW_PIN_TIMEOUT_MS` (20 s) so a generator that never answers can't block the panel's own WiFi reconnection indefinitely.
  - **Two more bugs found live testing this on the test rig (AP present), same day, both now fixed:** (1) the pin's `esp_wifi_set_channel()` call was unconditional at first — calling it while the STA is actively associated tears the connection down (`[WiFi] Disconnected, reason=8`, confirmed live on every selection). It only ever needs to fire when `WiFi.status() != WL_CONNECTED`; with an AP present ESP-NOW already rides the STA's own associated channel, nothing to pin. (2) Independently, pressing the Config tab's "Scan" button while connected exposed a latent bug in `espnow_wifi_supervise()`'s `connected` branch, unrelated to the pin itself and pre-existing: `WiFi.channel()` returns whatever channel the active `WiFi.scanNetworks()` sweep is currently probing, not the true associated channel, so that branch (no scan guard) wrote the "home channel" as 1,2,3…14 — the scan's own sweep — instead of the AP's real channel. The next reconnect then used one of those bogus values as its channel hint, which cascaded into repeated "associated but no IP" radio restarts and an ESP-NOW deinit/reinit race (`esp_now_add_peer()` failing with `ESP_ERR_ESPNOW_NOT_INIT`) — the visible symptom being that selecting a real generator silently reverted to "none". Fixed by skipping that branch while `WiFi.scanComplete() == WIFI_SCAN_RUNNING`. Neither bug was ever exercised before this session, since nothing previously pressed Scan while connected *and* ESP-NOW-enabled at once.
  - **A third, unrelated pre-existing bug surfaced the same way, same day**: the Config tab's "Connect" button (`on_connect_request()`, `display.ino`) applies a new WiFi SSID/pass via `WiFi.disconnect(true)` — the `true` is `wifioff`, not `eraseap` (`WiFiSTAClass::disconnect(bool wifioff, bool eraseap, ...)`), so it calls `STA.end()` and tears down the WiFi driver — and ESP-NOW with it, same as the documented `WiFi.mode(WIFI_OFF)` radio-restart path elsewhere in this file, which already knows to call `espnow_client_init()` afterward. `on_connect_request()` never did. Every "Connect" press with ESP-NOW already enabled therefore left it permanently torn down (`esp_now_add_peer()`/`esp_now_send()` failing with `ESP_ERR_ESPNOW_NOT_INIT` from then on) until something else happened to reinitialize it. Fixed by adding the same `espnow_client_init()` call there.
  - Compiled and flashed to the test rig's panel; the no-AP join-pin scenario itself (the point of the original fix) still hasn't been re-tried in the field.

Keep the AP on a **fixed channel** (it is: OpenWrt `radio0.channel=13`). If the AP auto-picks a new channel while both devices are off or disconnected, they look on the old home channel and only find the AP on the next full retry (≤5 min).

**Verified live 2026-10-16** (test rig `.97`, panel `.204`, `wifi down` on the AP for 60–150 s, v1 and v2):
- ESP-NOW kept delivering ~87–95 % of state packets for the whole outage, and the panel UI ran on them throughout.
- Two operator ECO toggles pressed during the outage reached the controller (`tx_ok:2`, `tx_fail:0`).
- After `wifi up`, both were back on WS in ~6–17 s (the AP itself takes ~10 s to come up). The first attempt right after an AP restart can fail with `INVALID_PMKID`; the next probe retries ~10 s later.

---

## Configuration

### Controller — `/wifi.json`, key `espnow`

| Field | Format | Description |
|---|---|---|
| `enabled` | bool | Enable/disable ESP-NOW |
| `key` | 32 hex chars | Fleet key (16 bytes), the same on every device |

```json
"espnow": { "enabled": true, "key": "<32 hex chars>" }
```

The beacon name is the existing top-level `"hostname"` (e.g. `gen-ctl-test`) — give each generator a distinct one.

- **Editing:** `/wifi-config` → "ESP-NOW" section: Enabled + Fleet key, with a "Generate new key" button (browser `crypto.getRandomValues`). Save & Reboot applies it.
- **Validation:** `/api/wifi/save` refuses to save (HTTP 400, nothing written, no reboot) if ESP-NOW is enabled with a key that isn't 32 hex characters. Without that check, a bad key would save fine and silently disable the link at the next boot, since `espnow_config_load()` fails closed.
- **`upload_data.pl`:** it uploads `data/wifi.json` (a template without an `espnow` block) over the device's config, as it does for every other group. After running it, re-enable ESP-NOW and re-enter the key in `/wifi-config`.

### Display

- **Pairing script (repo root):** `./espnow_key.sh <generator-host> <panel-host> [--key-only]`.
  - It reads the key from the generator's `/api/wifi/config` and writes it to the panel.
  - Then it waits until the panel hears that generator's beacon and selects it by hostname.
  - It reports success only once encrypted state actually arrives. A key mismatch fails after 15 s instead of passing.
  - The key is never printed or passed on a command line (piped to `curl` on stdin).
  - Every step validates the JSON it gets, so a failed helper is an error, never "not found".
- **Key only:** `display/set_espnow_key.sh [panel-host]` (default `10.100.100.204`) — asks for the key with a hidden prompt, or reads it from stdin, validates it, writes it and checks the panel enabled ESP-NOW. `espnow_key.sh` uses it for its own key step. Under the hood: `POST http://<display>:8080/espnow` with `key=<32 hex>`.
- **Generator:** Config tab → "Generator (ESP-NOW)" dropdown, which lists every generator heard in the last 10 s with RSSI, plus the selected one marked "(not seen)" if it is out of range.
- **Generator without the touchscreen:** `POST /espnow` with `gen_mac=<mac>&gen_name=<name>` (an empty `gen_mac` clears the selection).
- **Storage:** saved to the panel's SPIFFS `/config.json` (`espnow_key`, `espnow_gen_mac`, `espnow_gen_name`); it survives OTA and USB flashing.
- **Generator Host:** selecting a generator also rewrites the Config tab's "Generator Host" once its first encrypted state arrives.

`GET http://<display>:8080/espnow` returns status without key material:

| Field | Meaning |
|---|---|
| `enabled` | ESP-NOW is up on the panel |
| `own_mac` | the panel's STA MAC |
| `gen_name` / `gen_mac` / `gen_ip` | the selected generator |
| `active` | state packets are arriving |
| `rx` | encrypted state packets received |
| `rx_applied` | of those, written to the UI because WS was silent |
| `last_rx_age_ms` | time since the last state packet |
| `rssi` | RSSI of the last state packet |
| `tx_ok` / `tx_fail` | command delivery, unicast only |
| `wifi_channel` | the panel's current WiFi channel |
| `outputs` | raw controller output mask from the last packet |
| `home_channel` | channel of the last AP association |
| `generators` | the list the dropdown shows |

---

## Wire format

Fixed-layout `__attribute__((packed))` structs, duplicated by hand in `esp_now_link.cpp` and `display/esp_now_client.cpp`. **This file is the source of truth**; nothing enforces the two copies matching at compile time. `proto_version` is `2` in every packet; a v1 device ignores v2 packets and vice versa. Packets are told apart by length, then magic. All are under the 250-byte ESP-NOW v1 payload cap.

### `EspNowBeaconPkt` — controller → broadcast, unencrypted, every 2 s (26 bytes)

| Field | Type | Notes |
|---|---|---|
| `magic` | `uint8_t` | `0xEB` |
| `proto_version` | `uint8_t` | `2` |
| `name` | `char[24]` | `wifi.json` `"hostname"`, NUL-padded |

### `EspNowHelloPkt` — display → broadcast, unencrypted (8 bytes)

| Field | Type | Notes |
|---|---|---|
| `magic` | `uint8_t` | `0xE1` |
| `proto_version` | `uint8_t` | `2` |
| `target` | `uint8_t[6]` | STA MAC of the controller being joined |

- **Display cadence:** every 2 s until state arrives, then every 30 s as a keepalive.
- **Controller side:** a matching hello adds or refreshes the sender as an encrypted peer. Peers silent for 120 s are dropped.
- **After a controller reboot:** the controller's peer table is empty, so the display stops receiving state and goes back to 2 s hellos, rejoining within seconds (seen live).

### `EspNowStatePkt` — controller → each joined display, encrypted, 1 Hz (63 bytes)

Values use the same sources and units as `ReportTask`'s WS `value` section.

| Field | Type | WS equivalent |
|---|---|---|
| `magic` | `uint8_t` | `0xE5` |
| `proto_version` | `uint8_t` | `2` |
| `state_id` | `uint8_t` | `state` enum (`rtos_config.h`); display maps it back to `state_name()` strings |
| `flags` | `uint8_t` | bit 0 = `!is_not_working()` (`switch.run`) |
| `reg_inputs` | `uint8_t` | raw `shift_reg_in` |
| `reg_outputs` | `uint16_t` | raw `shift_reg_outputs` |
| `rpm` | `uint32_t` | `RPM` |
| `freq_hz` | `float` | `FREQUENCY` |
| `bat_v` | `float` | `ADC1_1` |
| `fuel_l` | `float` | `ADC1_0` |
| `core_v` | `float` | `ADC1_2` |
| `head_temp_c` | `float` | `HEAD_TEMP` |
| `oil_temp_c` | `float` | `ADC0_0` |
| `ambient_temp_c` | `float` | `ADC0_1` |
| `inverter_temp_c` | `float` | `ADC0_2` |
| `alternator_temp_c` | `float` | `ADC0_3` |
| `vrms` | `float` | `VRMS` |
| `irms` | `float` | `IRMS` |
| `active_power_w` | `float` | `POWER_ACTIVE` |
| `sta_ip` | `uint32_t` | controller's STA IPv4 as `IPAddress` stores it, 0 if not connected |

- **Not carried:** apparent/reactive power, power factor, energy counters, `MAINT_DUE_MASK`, uptime. They keep their last WS value on the display.

### `EspNowCmdPkt` — display → controller, encrypted, on demand (5 bytes)

| Field | Type | Notes |
|---|---|---|
| `magic` | `uint8_t` | `0xEC` |
| `proto_version` | `uint8_t` | `2` |
| `cmd` | `uint8_t` | `START`=0, `START_RETRY`=1, `STOP`=2, `REG_SET`=3, `REG_TOGGLE`=4 |
| `reg` | `uint8_t` | for `REG_SET`/`REG_TOGGLE`: raw register index 0-15 |
| `state` | `uint8_t` | for `REG_SET`: 0/1 |

- **On the controller:** each command becomes a `WebCommand_t` on `xWebCommandQueue` with `source = FLOG_SRC_ESPNOW` (the journal shows `"espnow"`).
- **On the display:** it sends `START`/`START_RETRY`/`STOP`/`REG_TOGGLE`; `REG_SET` is accepted but unused.

---

## Not yet built

- On-screen key entry on the panel (today: `display/set_espnow_key.sh` / `espnow_key.sh`).
- Monitoring several generators at once on one panel.
