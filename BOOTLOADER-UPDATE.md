# Bootloader update over HTTP (no USB access)

Rare, single-use operation — for a controller with no physical USB access
(production). If USB is available, just use `idf/flash.sh` instead; it
already flashes the bootloader as part of every USB flash, no special
procedure needed. See `CLAUDE.md`'s OTA-rollback section for the full
mechanism writeup and the two real flash-driver bugs found getting this
working; this file is just the checklist.

**Why this exists at all**: OTA never touches the bootloader (only the app
partition + otadata). `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE` only protects
against a crashing OTA update if the bootloader physically on the chip
already has rollback support compiled in — which nothing OTA'd in ever
changes. A controller that was last flashed over USB before this option
existed needs its bootloader replaced once, and production has no USB port.

**Risk**: the bootloader region has no A/B redundancy at all (unlike the app
partitions). A power loss or reset between erase and write leaves it
permanently blank — recoverable only via USB/`esptool`. The write path
retries (3 attempts, full erase→write→verify each time) but there is no
software fix for that window, only minimizing it.

## Procedure

All commands from `idf/`, against the test rig first — **never attempt this
on production without having already confirmed it clean on the test rig**.

1. **Build + OTA in the endpoint-enabled app** (temporary — this is the only
   build that exposes `POST /api/bootloader/update`):
   ```
   ./compile_bootloader_update.sh
   ./burn_bootloader_update.sh
   ```
   Wait for the device to reboot and come back healthy
   (`GET /api/fw/partitions`) before continuing.

2. **Push the bootloader itself** — pick one:
   ```
   ./update_bootloader.sh          # normal, chatty bootloader (INFO log level)
   ./update_bootloader_silent.sh   # silent bootloader (CONFIG_BOOTLOADER_LOG_LEVEL_NONE)
   ```
   Each uploads the bootloader.bin from its matching build
   (`build/bootloader/bootloader.bin` / `build_nodebug/bootloader/bootloader.bin`)
   to `POST /api/bootloader/update?confirm=1`. A `200 OK. Rebooting...`
   means erase+write+verify succeeded and the device is rebooting to confirm
   the new bootloader boots at all.

   The app that was running when you called this (still `pending_verify`,
   not yet soaked 10 minutes) will bounce back to the previously-`valid` app
   slot on this reboot — expected, not a failure; the bootloader write
   itself is unaffected by which app slot ends up running. See
   `CLAUDE.md` for why.

3. **Confirm the device is healthy**: `GET /api/fw/partitions`, `GET /log`
   (no new `panic_summary`), and — if at all possible — watch `ttyACM*`
   through the reboot. Confirm the boot banner's own compile-time string
   matches the bootloader.bin you just uploaded (`strings
   build*/bootloader/bootloader.bin | grep 2026` against the serial log).

4. **OTA a normal (flag-off) build back in immediately** — do not leave the
   `BOOTLOADER_UPDATE_ENDPOINT_ENABLED=1` build running any longer than this
   operation needs, it's an unauthenticated write to the one region of flash
   with no recovery path if abused:
   ```
   ./burn.sh          # or ./burn_nodebug.sh
   ```

Only once this whole sequence is confirmed clean on the test rig, repeat it
against production — a separate, explicit decision each time, not an
automatic follow-on.
