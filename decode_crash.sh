#!/usr/bin/env bash
# decode_crash.sh — pulls the live coredump off the controller (GET
# /coredump, see CLAUDE.md's FRAM-journal section), matches it against the
# right build_archive/*.tar.xz by trying candidates until one's SHA256
# actually fits, and prints a full decoded backtrace — no manual venv/tar/
# esptool fumbling required. Companion to fram_log.h's FLOG_EV_TASK_WDT
# capture, which only ever gets you a task name; this is the real thing.
#
# Usage: ./decode_crash.sh [host]
#   host defaults to the contents of ./host (same convention as burn.sh).
set -uo pipefail   # not -e: a candidate build failing to match is expected,
                   # not a script bug — every failure path is handled explicitly.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

PROJECT="esp32-gen-ctl-ai-rtos"
ARCHIVE_DIR="$SCRIPT_DIR/build_archive"
VENV_DIR="$SCRIPT_DIR/.coredump_venv"

# GDB discovery: explicit $GDB env var wins, then whatever's on PATH, then
# the newest xtensa-esp32-elf-gdb under the Arduino IDE's own toolchain
# install (the arduino-cli/IDE package manager's own versioned layout —
# works regardless of which exact version got installed, unlike a pinned
# path). No hardcoded version/username so this works on any machine that
# has either ESP-IDF's or arduino-esp32's own toolchain installed.
if [ -z "${GDB:-}" ]; then
    GDB=$(command -v xtensa-esp32-elf-gdb 2>/dev/null || true)
fi
if [ -z "${GDB:-}" ]; then
    GDB=$(ls -1 "$HOME"/.arduino15/packages/esp32/tools/xtensa-esp-elf-gdb/*/bin/xtensa-esp32-elf-gdb 2>/dev/null | sort -V | tail -1)
fi
if [ -z "${GDB:-}" ] || [ ! -x "$GDB" ]; then
    echo "ERROR: xtensa-esp32-elf-gdb not found. Install the esp32 Arduino/ESP-IDF toolchain, or set GDB=/path/to/xtensa-esp32-elf-gdb." >&2
    exit 1
fi

WORK_DIR=$(mktemp -d)
trap 'rm -rf "$WORK_DIR"' EXIT

HOST="${1:-}"
if [ -z "$HOST" ]; then
    HOST=$(tr -d '[:space:]' < "$SCRIPT_DIR/host" 2>/dev/null)
fi
if [ -z "$HOST" ]; then
    echo "No host given and ./host is empty/missing." >&2
    exit 1
fi

echo "=== decode_crash.sh: target $HOST ==="
echo ""

# --- 1. Bootstrap esp-coredump in an isolated venv (first run only) ---------
# Arch's system Python refuses unmanaged "pip install" (PEP 668), and this
# tool has no reason to live anywhere but here anyway.
if [ ! -x "$VENV_DIR/bin/python" ]; then
    echo "--- First run: setting up esp-coredump in $VENV_DIR ---"
    if ! python3 -m venv "$VENV_DIR"; then
        echo "ERROR: could not create the venv (is python3-venv installed?)." >&2
        exit 1
    fi
    if ! "$VENV_DIR/bin/pip" install --quiet esp-coredump; then
        echo "ERROR: pip install esp-coredump failed — check network." >&2
        rm -rf "$VENV_DIR"
        exit 1
    fi
    echo ""
fi
PY="$VENV_DIR/bin/python"

# --- 2. Fetch the live coredump ----------------------------------------------
echo "--- Fetching /coredump from $HOST ---"
if ! curl -sf --max-time 20 "http://$HOST/coredump" -o "$WORK_DIR/coredump.bin"; then
    echo "ERROR: GET /coredump failed — is the controller reachable at $HOST?" >&2
    exit 1
fi
DUMP_SIZE=$(stat -c%s "$WORK_DIR/coredump.bin" 2>/dev/null || stat -f%z "$WORK_DIR/coredump.bin")
echo "Got ${DUMP_SIZE} bytes."
echo ""

# Captured now (right after the fetch) rather than at archiving time at the
# bottom of the script — this is "when we pulled it off the device", the
# thing worth preserving in the archive filename, not whenever matching
# happened to finish.
CRASH_ARCHIVE_DIR="$SCRIPT_DIR/crash_archive"
FETCH_STAMP=$(date +%Y%m%d_%H%M%S)      # for the archive filename
FETCH_DATE_HUMAN=$(date)                # for meta.txt — same moment, human-readable form
DUMP_SHA256=$(sha256sum "$WORK_DIR/coredump.bin" | cut -d' ' -f1)

# --- 3. Dedup against crash_archive/ — same raw coredump already saved? -----
# A re-run against a coredump that hasn't changed since the last run (nothing
# has crashed again) is common while poking at this script or re-checking an
# old crash — archiving a byte-identical duplicate every time would just
# bloat crash_archive/ for no reason. Sidecar .sha256 files (written by the
# archiving step below) let this check happen without untarring every
# archive on every run.
DUP_ARCHIVE=""
if [ -d "$CRASH_ARCHIVE_DIR" ]; then
    for sidecar in "$CRASH_ARCHIVE_DIR"/*.sha256; do
        [ -e "$sidecar" ] || continue
        if [ "$(cat "$sidecar" 2>/dev/null)" = "$DUMP_SHA256" ]; then
            DUP_ARCHIVE=$(basename "$sidecar" .sha256)
            break
        fi
    done
fi
if [ -n "$DUP_ARCHIVE" ]; then
    echo "This coredump is already archived as crash_archive/${DUP_ARCHIVE}.tar.xz — decoding again below, but not re-archiving."
    echo ""
fi
# --- 4. Pull the latest FLOG_EV_TASK_WDT or FLOG_EV_PANIC_SUMMARY record for
#        human-readable context -------------------------------------------
# /log's own "count" param is hard-capped at 50 server-side regardless of
# what's requested (esp32-gen-ctl-ai-rtos.ino: "if (count > 50) count = 50;")
# — a single request only ever sees the newest 50 records. A busy test
# session easily logs more than 50 events (maintenance/metrics/boot/etc.)
# between a crash and whenever this script runs, pushing the crash-context
# record out of that window — so this paginates backward via "before"
# through the whole ring (bounded by the log's own "oldest" seq) instead of
# trusting one page. A fetch/parse failure is reported as exactly that,
# never silently folded into "no record found" — those mean very different
# things.
#
# Checks BOTH event types, not just task_wdt — found live 2026-09-25:
# FLOG_EV_PANIC_SUMMARY (added 2026-09-24, see CLAUDE.md's FRAM-journal
# section) is the more general one, firing for ANY crash-shaped reset
# reason (a plain abort()/exception, not just a Task Watchdog timeout) and
# naming the actual crashed task + PC/exception cause — task_wdt is RTC-
# ISR-based and only fires for TWDT specifically (though it can list
# *multiple* starved tasks, which panic_summary can't). This script had
# only ever looked for task_wdt since before panic_summary existed, so any
# non-TWDT crash (exactly the kind a plain `abort()` produces) always fell
# through to "No task_wdt record found — decoding blind", even when a
# perfectly good panic_summary record was sitting right there naming the
# task, PC, and exception cause.
echo "--- Checking /log for the most recent task_wdt/panic_summary event ---"
CRASH_TS=0
BEFORE_ARG=""
PAGES=0
LOG_FAILED=0
CRASH_INFO=""
while :; do
    PAGES=$((PAGES + 1))
    if [ "$PAGES" -gt 100 ]; then   # ~5000 records — the whole 4080-record ring, with margin
        break
    fi
    URL="http://$HOST/log?count=50"
    [ -n "$BEFORE_ARG" ] && URL="${URL}&before=${BEFORE_ARG}"
    PAGE_JSON=$(curl -sf --max-time 15 "$URL" || true)
    if [ -z "$PAGE_JSON" ]; then
        LOG_FAILED=1
        break
    fi
    PAGE_RESULT=$(echo "$PAGE_JSON" | "$PY" -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    print("PARSE_ERROR")
    sys.exit(0)
events = d.get("events", []) if isinstance(d, dict) else (d if isinstance(d, list) else [])
oldest_boundary = d.get("oldest", 0) if isinstance(d, dict) else 0
if not events:
    print("EMPTY|" + str(oldest_boundary))
    sys.exit(0)
candidates = [r for r in events if r.get("type") in ("task_wdt", "panic_summary")]
min_seq = min(r.get("seq", 0) for r in events)
if candidates:
    # Newest of EITHER type wins - both are equally valid crash context,
    # and only one of them will actually exist for a given crash (task_wdt
    # for a TWDT timeout, panic_summary for anything else the coredump-to-
    # flash mechanism reacts to). Deliberately no apostrophes or quote
    # characters anywhere in this comment block: it lives inside the outer
    # shells own single-quoted string wrapping this whole python -c
    # argument, and one stray quote character here ends that string early,
    # turning everything after it into garbage shell tokens instead of
    # python source. bash -n happens to catch it as a syntax error; a
    # different stray quote could just as easily fail silently instead.
    r = max(candidates, key=lambda x: x.get("seq", 0))
    # Detail field (fram_log.h: char tasks[18]/task[16]) legitimately
    # contains real \r\n bytes between/after names (see the json_escape_into
    # fix elsewhere this session) — flatten to spaces here so it can never
    # break the "|"/line based parsing below; a multi-line value silently
    # corrupted the field offsets on first try (2026-08-25).
    if r.get("type") == "task_wdt":
        detail = "starved: " + str(r.get("tasks", "?")).replace("\n", " ").replace("\r", " ").strip()
        uptime = str(r.get("uptime_s", 0))
    else:
        detail = ("task: " + str(r.get("task", "?")).replace("\n", " ").replace("\r", " ").strip()
                   + "  pc: " + str(r.get("pc", "?"))
                   + "  exc_cause: " + str(r.get("exc_cause", "?")))
        uptime = "?"
    fields = [str(r.get("ts", 0)), uptime, r.get("type"), detail, str(min_seq), str(oldest_boundary)]
    print("FOUND|" + "|".join(fields))
else:
    print("NONE|" + str(min_seq) + "|" + str(oldest_boundary))
' 2>&1)
    # 2>&1 above (not 2>/dev/null) is deliberate: a Python-side error must
    # surface as visible garbage in PAGE_RESULT (caught by the "not FOUND/
    # EMPTY/NONE" check below) rather than silently produce an empty string
    # that reads identically to "the scan legitimately found nothing". A
    # real bug here once did exactly that: an f-string with backslash-
    # escaped quotes, fine in isolation, turned into a Python SyntaxError
    # once passed through bash's single-quoted -c '...' (which doesn't
    # process the backslashes at all, so Python received them literally) —
    # silenced by 2>/dev/null, this always reported "no task_wdt record"
    # regardless of whether one existed. Found 2026-08-25 only because the
    # user knew a real crash had happened and asked why it said otherwise.
    case "$PAGE_RESULT" in
        FOUND\|*|EMPTY\|*|NONE\|*) : ;;
        *)
            echo "  (log page $PAGES: python helper failed — ${PAGE_RESULT:-empty output})"
            LOG_FAILED=1
            break
            ;;
    esac
    TAG=$(echo "$PAGE_RESULT" | cut -d'|' -f1)
    if [ "$TAG" = "FOUND" ]; then
        CRASH_INFO=$(echo "$PAGE_RESULT" | cut -d'|' -f2-5)
        break
    fi
    if [ "$TAG" = "EMPTY" ]; then
        break   # nothing more in the ring
    fi
    # TAG = NONE: keep paging backward unless we've hit the ring's own oldest
    MIN_SEQ=$(echo "$PAGE_RESULT" | cut -d'|' -f2)
    OLDEST_BOUNDARY=$(echo "$PAGE_RESULT" | cut -d'|' -f3)
    if [ "$MIN_SEQ" -le "$OLDEST_BOUNDARY" ]; then
        break   # reached the start of the ring, nothing older to check
    fi
    BEFORE_ARG="$MIN_SEQ"
done

if [ -n "$CRASH_INFO" ]; then
    CRASH_TS=$(echo "$CRASH_INFO" | cut -d'|' -f1)
    CRASH_UPTIME=$(echo "$CRASH_INFO" | cut -d'|' -f2)
    CRASH_TYPE=$(echo "$CRASH_INFO" | cut -d'|' -f3)
    CRASH_DETAIL=$(echo "$CRASH_INFO" | cut -d'|' -f4-)
    CRASH_DATE=$(date -d "@$CRASH_TS" 2>/dev/null || date -r "$CRASH_TS" 2>/dev/null || echo "?")
    # Built once, reused everywhere else this needs printing (the MATCH
    # banner, crash_archive/'s backtrace.txt) so the task_wdt-vs-
    # panic_summary formatting difference (the latter has no uptime) only
    # has to be handled in one place. CRASH_UPTIME is the literal string
    # "?" for panic_summary (not empty), so this needs a real comparison,
    # not a bash ${VAR:+...} empty-check.
    if [ "$CRASH_UPTIME" = "?" ]; then
        CRASH_SUMMARY="Crash: $CRASH_DATE — ${CRASH_DETAIL}"
    else
        CRASH_SUMMARY="Crash: $CRASH_DATE (uptime ${CRASH_UPTIME}s) — ${CRASH_DETAIL}"
    fi
    echo "Latest ${CRASH_TYPE} record: ${CRASH_SUMMARY#Crash: }  (scanned $PAGES page(s))"
elif [ "$LOG_FAILED" -eq 1 ]; then
    echo "WARNING: /log fetch or parse failed partway through the scan ($PAGES page(s) in) — this is NOT the same as confirming no crash exists, just that the scan couldn't finish. Decoding blind."
else
    echo "No task_wdt/panic_summary record found after scanning the full ring ($PAGES page(s)) — decoding blind."
fi
echo ""

# --- 5. Order build_archive/ candidates: nearest-before-crash first ----------
# Names are ...-idf[-nodebug]_<yyyymmddhhmm>.tar.xz (firmware version).
# Leftover _YYYYMMDD_HHMMSS tars from before that rename are still parsed.
# Found live 2026-09-21: a ${PROJECT}_*.tar.xz glob silently skipped every
# idf.py archive because "-idf_" isn't "_" right after the project name —
# the run reported "no archived build matched" while never actually trying
# the one that would have matched. Glob is ${PROJECT}*.tar.xz on purpose.
mapfile -t ARCHIVES < <(ls "$ARCHIVE_DIR"/${PROJECT}*.tar.xz 2>/dev/null)
if [ ${#ARCHIVES[@]} -eq 0 ]; then
    echo "ERROR: no archived builds in $ARCHIVE_DIR — nothing to try." >&2
    echo "(Only builds made after 2026-08-25's archiving fix will be here.)" >&2
    exit 1
fi

ORDER_FILE="$WORK_DIR/order.txt"
: > "$ORDER_FILE"
for a in "${ARCHIVES[@]}"; do
    base=$(basename "$a" .tar.xz)
    # Trailing stamp, two conventions:
    #   legacy wall-clock of the tar: _YYYYMMDD_HHMMSS  (arduino-builder +
    #     idf.py archives through 2026-09-28)
    #   firmware version[]:           _YYYYMMDDhhmm     (archive_build.sh
    #     from 2026-09-28; same 12-digit stamp /log and fw_update show).
    #     Optional _<8hex> after that is a same-minute distinct-ELF suffix.
    # A fixed prefix-strip would leave the idf variant's "-idf" / "-nodebug"
    # stuck on the front of $stamp, corrupting the date parse below silently
    # (no error, just epoch=0 for every idf archive, same failure shape as
    # the glob bug above: looks like it ran fine, quietly gets the wrong answer).
    epoch=0
    if [[ "$base" =~ _([0-9]{8}_[0-9]{6})$ ]]; then
        stamp="${BASH_REMATCH[1]}"
        epoch=$(date -d "${stamp:0:8} ${stamp:9:2}:${stamp:11:2}:${stamp:13:2}" +%s 2>/dev/null)
    elif [[ "$base" =~ _([0-9]{12})(_[0-9a-fA-F]{8})?$ ]]; then
        ver="${BASH_REMATCH[1]}"
        epoch=$(date -d "${ver:0:4}-${ver:4:2}-${ver:6:2} ${ver:8:2}:${ver:10:2}:00" +%s 2>/dev/null)
    fi
    epoch=${epoch:-0}
    echo "$epoch $a" >> "$ORDER_FILE"
done

if [ "$CRASH_TS" -gt 0 ]; then
    # Builds compiled AFTER the crash can't be the culprit — try those still
    # first if nothing at-or-before it matches (a clock could be off), but
    # prefer nearest-at-or-before by absolute distance.
    # -k1,1n -k2,2n (not a bare "sort -n"): with every candidate build
    # compiled after the same crash (after=1 tied across all of them), plain
    # "sort -n" falls back to a lexicographic compare of the rest of the
    # line once its leading numeric field ties — "1727" sorts before "847"
    # as text — silently trying candidates in a near-nonsensical order.
    # Found live 2026-08-25 testing against a real crash.
    ORDERED=$(awk -v crash="$CRASH_TS" '{
        dist = $1 - crash; if (dist < 0) dist = -dist;
        after = ($1 > crash) ? 1 : 0;
        print after, dist, $2
    }' "$ORDER_FILE" | sort -k1,1n -k2,2n | cut -d' ' -f3-)
else
    # No crash timestamp known — just try newest-first.
    ORDERED=$(sort -k1,1rn "$ORDER_FILE" | cut -d' ' -f2-)
fi

# --- 6. Try each candidate until one's SHA256 actually matches ---------------
echo "--- Matching against build_archive/ ---"
FOUND=0
while IFS= read -r archive; do
    [ -z "$archive" ] && continue
    name=$(basename "$archive")
    echo "Trying $name ..."

    elf_member=$(tar -tJf "$archive" 2>/dev/null | grep '\.elf$' | head -1)
    if [ -z "$elf_member" ]; then
        echo "  (no .elf inside this archive, skipping)"
        continue
    fi
    rm -f "$WORK_DIR/candidate.elf"
    if ! tar -xJf "$archive" -O "$elf_member" > "$WORK_DIR/candidate.elf" 2>/dev/null; then
        echo "  (couldn't extract $elf_member, skipping)"
        continue
    fi

    OUT=$("$PY" -m esp_coredump info_corefile \
        --core "$WORK_DIR/coredump.bin" --core-format raw \
        --gdb "$GDB" \
        "$WORK_DIR/candidate.elf" 2>&1)

    if echo "$OUT" | grep -q "Invalid application image"; then
        echo "  SHA256 mismatch — not this build."
        continue
    fi
    if echo "$OUT" | grep -qi "^ERROR"; then
        echo "  Failed for a different reason:"
        echo "$OUT" | sed 's/^/    /'
        continue
    fi

    FOUND=1
    MATCH_NAME="$name"
    echo ""
    echo "=================================================================="
    echo "MATCH: $name"
    [ "$CRASH_TS" -gt 0 ] && echo "$CRASH_SUMMARY"
    echo "=================================================================="
    echo "$OUT"
    break
done <<< "$ORDERED"

if [ "$FOUND" -eq 0 ]; then
    echo ""
    echo "No archived build matched this coredump (SHA256 mismatch on every"
    echo "candidate in $ARCHIVE_DIR). The build that produced it predates"
    echo "archiving, or was never archived — it's not decodable."
fi

# --- 7. Archive the raw coredump (+ decoded backtrace, if we got one) -------
# Mirrors build_archive/'s own convention (idf/compile.sh: one tar.xz per
# build, xz -6 -T0). Archived even on a FOUND=0 run — an unresolved dump is
# still worth keeping: the moment a matching build_archive/ entry turns up
# (or decode_crash.sh's matching logic improves), crash_archive/'s raw
# coredump.bin can be re-decoded by hand instead of being lost the moment
# the next crash overwrites the live /coredump partition on the device.
if [ -n "$DUP_ARCHIVE" ]; then
    echo ""
    echo "(not archiving — duplicate of crash_archive/${DUP_ARCHIVE}.tar.xz)"
elif ! command -v xz >/dev/null 2>&1; then
    echo ""
    echo "WARNING: 'xz' not found — skipping crash archiving (decode result above is still valid)." >&2
else
    mkdir -p "$CRASH_ARCHIVE_DIR"
    STAGE="$WORK_DIR/archive_stage"
    mkdir -p "$STAGE"
    cp "$WORK_DIR/coredump.bin" "$STAGE/coredump.bin"
    {
        echo "host: $HOST"
        echo "fetched_at: $FETCH_DATE_HUMAN"
        echo "coredump_sha256: $DUMP_SHA256"
        echo "coredump_size_bytes: $DUMP_SIZE"
        if [ "$CRASH_TS" -gt 0 ]; then
            echo "crash_context_type: $CRASH_TYPE"
            echo "crash_context_at: $CRASH_DATE"
            echo "crash_context_uptime_s: $CRASH_UPTIME"
            echo "crash_context_detail: $CRASH_DETAIL"
        else
            echo "crash_context_at: unknown (no matching task_wdt/panic_summary /log record found)"
        fi
        if [ "$FOUND" -eq 1 ]; then
            echo "matched_build: $MATCH_NAME"
        else
            echo "matched_build: none (unresolved — no build_archive/ entry matched this SHA256)"
        fi
    } > "$STAGE/meta.txt"
    if [ "$FOUND" -eq 1 ]; then
        {
            echo "MATCH: $MATCH_NAME"
            [ "$CRASH_TS" -gt 0 ] && echo "$CRASH_SUMMARY"
            echo ""
            echo "$OUT"
        } > "$STAGE/backtrace.txt"
    fi
    CRASH_TAR="$CRASH_ARCHIVE_DIR/${PROJECT}-crash_${FETCH_STAMP}.tar.xz"
    tar -C "$STAGE" --use-compress-program="xz -6 -T0" -cf "$CRASH_TAR" .
    echo "$DUMP_SHA256" > "$CRASH_ARCHIVE_DIR/${PROJECT}-crash_${FETCH_STAMP}.sha256"
    echo ""
    echo "Archived crash -> $CRASH_TAR"
fi

if [ "$FOUND" -eq 0 ]; then
    exit 1
fi
