#!/usr/bin/env python3
"""HTTP + WebSocket load/soak test for the genset controller.

Read-only by design: it only issues GETs to read endpoints and never sends a
WebSocket text frame (a bare numeric string over /ws toggles a register).

Usage:
    ./stress_test.py [host] [--profile soak|burst] [--duration SECONDS] ...

Host defaults to the repo-root `host` file. Point it at the test rig only.

Verdict:
    PASS          no reboot, no crash-type journal event
    FAIL          reboot (new `boot` event or WS UPTIME going backwards) or a
                  new panic_summary / task_wdt / i2c_stuck event
    INCONCLUSIVE  the final journal check could not be completed, or no WS
                  report arrived (no heap/uptime data), so a clean run is not
                  proven
Exit status: 0 PASS, 1 FAIL, 2 INCONCLUSIVE or baseline unavailable.
"""

import argparse
import base64
import hashlib
import http.client
import json
import os
import random
import socket
import statistics
import struct
import sys
import threading
import time

PROFILES = {
    # Moderate mix for hours: looks for heap drift and long-run reboots.
    "soak": dict(duration=8 * 3600, http_workers=2, http_delay=2.0,
                 ws_normal=2, ws_churn=1, ws_mute=0, churn_period=60.0),
    # Short and dense: looks for TWDT/latency trouble under concurrency.
    # Total sessions stay below the 16-session httpd limit with a browser and
    # the panel also connected.
    "burst": dict(duration=600, http_workers=6, http_delay=0.0,
                  ws_normal=3, ws_churn=3, ws_mute=1, churn_period=5.0),
}

# (name, path, weight). Filtered /log requests force up to 400 FRAM reads via
# i2c_run_on_owner(); static files exercise the FFat/I2C flash guard.
HTTP_MIX = [
    ("log",          "/log?count=25", 4),
    ("log_maint",    "/log?group=maint&count=50", 2),
    ("log_alarm",    "/log?group=alarm&count=50", 2),
    ("metrics",      "/metrics", 3),
    ("chart_js",     "/static/js/chart.js", 1),
    ("dashboard",    "/static/dashboard.json", 1),
    ("journal_page", "/journal", 1),
]

CRASH_EVENT_TYPES = {"panic_summary", "task_wdt", "i2c_stuck"}
HTTP_TIMEOUT_S = 15.0
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
WS_SUBPROTOCOL = "arduino"
MONITOR_PERIOD_S = 30.0
FINAL_CHECK_BUDGET_S = 180.0
HEAP_DRIFT_WARN_BYTES = 8192


def log(msg):
    print(time.strftime("%H:%M:%S ") + msg, flush=True)


class Stats:
    def __init__(self):
        self.lock = threading.Lock()
        self.http = {}            # name -> {"lat": [], "ok": n, "http_err": n, "net_err": n}
        self.ws = dict(connects=0, connect_fail=0, drops=0, frames=0, pings=0, bad_frames=0)
        self.heap = []            # (t, free, min)
        self.uptime_last = None
        self.uptime_resets = []   # (t, previous, new)
        self.events = []          # journal events newer than the baseline
        self.monitor_fail = 0

    def http_result(self, name, latency, kind):
        with self.lock:
            s = self.http.setdefault(name, {"lat": [], "ok": 0, "http_err": 0, "net_err": 0})
            s[kind] += 1
            if kind == "ok":
                s["lat"].append(latency)

    def ws_count(self, key):
        with self.lock:
            self.ws[key] += 1

    def ws_report(self, report):
        value = report.get("value")
        date = report.get("date")
        now = time.time()
        with self.lock:
            if isinstance(value, dict) and "FREE_HEAP" in value and "MIN_HEAP" in value:
                self.heap.append((now, int(value["FREE_HEAP"]), int(value["MIN_HEAP"])))
            if isinstance(date, dict) and "UPTIME" in date:
                up = int(date["UPTIME"])
                # Several clients see the same report; only a clear drop counts.
                if self.uptime_last is not None and up + 5 < self.uptime_last:
                    self.uptime_resets.append((now, self.uptime_last, up))
                if self.uptime_last is None or up > self.uptime_last or up + 5 < self.uptime_last:
                    self.uptime_last = up


def http_get(host, path):
    """Return (status, body). Raises OSError/HTTPException on network failure."""
    conn = http.client.HTTPConnection(host, 80, timeout=HTTP_TIMEOUT_S)
    try:
        conn.request("GET", path, headers={"Connection": "close"})
        resp = conn.getresponse()
        return resp.status, resp.read()
    finally:
        conn.close()


def fetch_log_after(host, after):
    """Journal events with seq > after, newest first. Raises ValueError on a bad shape."""
    status, body = http_get(host, f"/log?count=50&after={after}")
    if status != 200:
        raise ValueError(f"/log returned HTTP {status}")
    doc = json.loads(body)
    if not isinstance(doc, dict) or "newest" not in doc or not isinstance(doc.get("events"), list):
        raise ValueError("/log response lacks newest/events")
    return doc


def http_worker(host, delay, stop, stats):
    names, paths, weights = zip(*HTTP_MIX)
    while not stop.is_set():
        i = random.choices(range(len(names)), weights=weights)[0]
        t0 = time.monotonic()
        try:
            status, _ = http_get(host, paths[i])
            stats.http_result(names[i], time.monotonic() - t0, "ok" if status == 200 else "http_err")
        except (OSError, http.client.HTTPException):
            stats.http_result(names[i], time.monotonic() - t0, "net_err")
            stop.wait(1.0)
        if delay:
            stop.wait(delay * random.uniform(0.5, 1.5))


class WsClient:
    """Minimal RFC 6455 client: receives reports, answers pings, sends nothing else."""

    def __init__(self, host):
        self.sock = socket.create_connection((host, 80), timeout=HTTP_TIMEOUT_S)
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall((
            f"GET /ws HTTP/1.1\r\nHost: {host}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\nSec-WebSocket-Protocol: {WS_SUBPROTOCOL}\r\n\r\n"
        ).encode())
        head = b""
        while b"\r\n\r\n" not in head:
            chunk = self.sock.recv(1024)
            if not chunk:
                raise ConnectionError("closed during handshake")
            head += chunk
        head, self.buf = head.split(b"\r\n\r\n", 1)
        lines = head.decode(errors="replace").split("\r\n")
        if " 101 " not in lines[0] + " ":
            raise ConnectionError(f"handshake refused: {lines[0]}")
        headers = {k.strip().lower(): v.strip() for k, _, v in (l.partition(":") for l in lines[1:])}
        expect = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        if headers.get("sec-websocket-accept") != expect:
            raise ConnectionError("bad Sec-WebSocket-Accept")

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("connection closed")
            self.buf += chunk
        data, self.buf = self.buf[:n], self.buf[n:]
        return data

    def recv_frame(self):
        b0, b1 = self._read(2)
        opcode, length = b0 & 0x0F, b1 & 0x7F
        if length == 126:
            length = struct.unpack(">H", self._read(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", self._read(8))[0]
        mask = self._read(4) if b1 & 0x80 else None
        payload = self._read(length)
        if mask:
            payload = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        return bool(b0 & 0x80), opcode, payload

    def send_control(self, opcode, payload=b""):
        mask = os.urandom(4)
        masked = bytes(c ^ mask[i % 4] for i, c in enumerate(payload))
        self.sock.sendall(bytes([0x80 | opcode, 0x80 | len(payload)]) + mask + masked)

    def close(self):
        try:
            self.send_control(0x8)
        except OSError:
            pass
        self.sock.close()


def ws_worker(host, mode, churn_period, stop, stats):
    """mode: normal (stays connected), churn (reconnects every churn_period),
    mute (never answers pings, so the server's heartbeat must drop it)."""
    while not stop.is_set():
        try:
            client = WsClient(host)
        except (OSError, ConnectionError):
            stats.ws_count("connect_fail")
            stop.wait(2.0)
            continue
        stats.ws_count("connects")
        deadline = time.monotonic() + churn_period if mode == "churn" else None
        message = b""
        try:
            client.sock.settimeout(1.0)
            while not stop.is_set():
                if deadline and time.monotonic() >= deadline:
                    break
                try:
                    fin, opcode, payload = client.recv_frame()
                except socket.timeout:
                    continue
                if opcode == 0x9:
                    stats.ws_count("pings")
                    if mode != "mute":
                        client.send_control(0xA, payload)
                elif opcode == 0x8:
                    raise ConnectionError("server closed")
                elif opcode in (0x0, 0x1):
                    message += payload
                    if fin:
                        stats.ws_count("frames")
                        try:
                            stats.ws_report(json.loads(message))
                        except (ValueError, TypeError):
                            stats.ws_count("bad_frames")
                        message = b""
        except (OSError, ConnectionError):
            # A socket.timeout mid-frame lands here too; treat it as a drop.
            stats.ws_count("drops")
            stop.wait(1.0)
        finally:
            client.close()


def monitor(host, baseline, stop, stats):
    seen = baseline
    while not stop.wait(MONITOR_PERIOD_S):
        try:
            doc = fetch_log_after(host, seen)
        except (OSError, http.client.HTTPException, ValueError) as e:
            with stats.lock:
                stats.monitor_fail += 1
            log(f"monitor: journal check failed ({e}); not counted as a reboot")
            continue
        new = sorted(doc["events"], key=lambda ev: ev["seq"])
        with stats.lock:
            stats.events.extend(new)
        for ev in new:
            if ev.get("type") == "boot" or ev.get("type") in CRASH_EVENT_TYPES:
                log(f"monitor: EVENT {json.dumps(ev)}")
        if new:
            seen = new[-1]["seq"]
        with stats.lock:
            heap = stats.heap[-1] if stats.heap else None
            ws = dict(stats.ws)
            reqs = sum(s["ok"] + s["http_err"] + s["net_err"] for s in stats.http.values())
            net = sum(s["net_err"] for s in stats.http.values())
        heap_s = f"free={heap[1]} min={heap[2]}" if heap else "no heap data yet"
        log(f"heap {heap_s} | http {reqs} reqs, {net} net errors | "
            f"ws {ws['frames']} frames, {ws['drops']} drops")
    return seen


def pct(values, p):
    if not values:
        return float("nan")
    values = sorted(values)
    return values[min(len(values) - 1, int(round(p / 100.0 * (len(values) - 1))))]


def final_journal_check(host, baseline, stats):
    """Return True once the journal past baseline has been read successfully."""
    seen = max([baseline] + [ev["seq"] for ev in stats.events])
    end = time.monotonic() + FINAL_CHECK_BUDGET_S
    while time.monotonic() < end:
        try:
            doc = fetch_log_after(host, seen)
            stats.events.extend(sorted(doc["events"], key=lambda ev: ev["seq"]))
            return True
        except (OSError, http.client.HTTPException, ValueError) as e:
            log(f"final journal check failed ({e}), retrying")
            time.sleep(10)
    return False


def report(stats, journal_ok, elapsed):
    print("\n=== HTTP ===")
    print(f"{'endpoint':<14}{'ok':>7}{'http_err':>9}{'net_err':>8}{'p50 ms':>9}{'p99 ms':>9}{'max ms':>9}")
    for name, _, _ in HTTP_MIX:
        s = stats.http.get(name)
        if not s:
            continue
        lat = [x * 1000 for x in s["lat"]]
        print(f"{name:<14}{s['ok']:>7}{s['http_err']:>9}{s['net_err']:>8}"
              f"{pct(lat, 50):>9.0f}{pct(lat, 99):>9.0f}{max(lat, default=float('nan')):>9.0f}")

    print("\n=== WebSocket ===")
    print(", ".join(f"{k}={v}" for k, v in stats.ws.items()))

    print("\n=== Heap (internal, from WS reports) ===")
    heap_warn = False
    if len(stats.heap) >= 20:
        n = max(5, len(stats.heap) // 10)
        first = statistics.median(h[1] for h in stats.heap[:n])
        last = statistics.median(h[1] for h in stats.heap[-n:])
        print(f"free: first-10% median={first:.0f} last-10% median={last:.0f} "
              f"drift={last - first:+.0f} B over {elapsed / 3600:.2f} h")
        print(f"min free ever (MIN_HEAP): {min(h[2] for h in stats.heap)}")
        heap_warn = first - last > HEAP_DRIFT_WARN_BYTES
    else:
        print("not enough samples")

    print("\n=== Journal events since start ===")
    boots = [ev for ev in stats.events if ev.get("type") == "boot"]
    crashes = [ev for ev in stats.events if ev.get("type") in CRASH_EVENT_TYPES]
    for ev in boots + crashes:
        print(json.dumps(ev))
    if not boots and not crashes:
        print("none")
    for t, prev, new in stats.uptime_resets:
        print(f"WS UPTIME went backwards at {time.strftime('%H:%M:%S', time.localtime(t))}: {prev} -> {new}")
    print(f"monitor checks failed: {stats.monitor_fail}")

    if boots or crashes or stats.uptime_resets:
        verdict, code = "FAIL", 1
    elif not journal_ok:
        verdict, code = "INCONCLUSIVE (final journal check never succeeded)", 2
    elif stats.ws["frames"] == 0:
        verdict, code = "INCONCLUSIVE (no WS report received)", 2
    else:
        verdict, code = "PASS", 0
    if heap_warn:
        verdict += f" + WARN: free heap dropped more than {HEAP_DRIFT_WARN_BYTES} B"
    print(f"\nVERDICT: {verdict}")
    return code


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("host", nargs="?", help="controller host (default: ./host file)")
    ap.add_argument("--profile", choices=PROFILES, default="soak")
    for key, typ in (("duration", float), ("http_workers", int), ("http_delay", float),
                     ("ws_normal", int), ("ws_churn", int), ("ws_mute", int), ("churn_period", float)):
        ap.add_argument("--" + key.replace("_", "-"), type=typ, help="override the profile value")
    args = ap.parse_args()

    cfg = dict(PROFILES[args.profile])
    for key in cfg:
        if getattr(args, key) is not None:
            cfg[key] = getattr(args, key)

    host = args.host
    if not host:
        with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "host")) as f:
            host = f.read().strip()
    if not host:
        sys.exit("empty host")

    try:
        baseline = int(fetch_log_after(host, 0)["newest"])
    except (OSError, http.client.HTTPException, ValueError) as e:
        print(f"cannot read baseline journal from {host}: {e}", file=sys.stderr)
        return 2
    log(f"target {host}, profile {args.profile}, journal baseline seq {baseline}, config {cfg}")

    stats = Stats()
    stop = threading.Event()
    threads = [threading.Thread(target=http_worker, args=(host, cfg["http_delay"], stop, stats), daemon=True)
               for _ in range(cfg["http_workers"])]
    for mode in ("normal", "churn", "mute"):
        threads += [threading.Thread(target=ws_worker, args=(host, mode, cfg["churn_period"], stop, stats),
                                     daemon=True) for _ in range(cfg["ws_" + mode])]
    mon = threading.Thread(target=monitor, args=(host, baseline, stop, stats), daemon=True)
    for t in threads + [mon]:
        t.start()

    t0 = time.monotonic()
    try:
        stop.wait(cfg["duration"])
    except KeyboardInterrupt:
        log("interrupted, finishing")
    stop.set()
    for t in threads + [mon]:
        t.join(timeout=HTTP_TIMEOUT_S + 5)
    elapsed = time.monotonic() - t0

    journal_ok = final_journal_check(host, baseline, stats)
    return report(stats, journal_ok, elapsed)


if __name__ == "__main__":
    sys.exit(main())
