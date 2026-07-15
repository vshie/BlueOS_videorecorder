#!/usr/bin/env python3
"""Calibrate WINCH_WIND_US / WINCH_UNWIND_US for matched shaft speed.

Fixed-duration open-loop legs at a sweep of PWM values on each side of
1500.  Counts falling edges (RPM proxy) and grabs MCM before/after snaps
of the W-disc.  Picks the wind/unwind pair whose edge counts are closest
at useful speed (asymmetric offsets allowed — ESC deadband is not
symmetric).

Extension is kept stopped for the duration.
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import lgpio
from smbus2 import SMBus

RELEASE_CH = 2
PCA9685_ADDR = 0x40
PCA9685_OE_GPIO = 4
ROT_GPIO = 10
STOP_US = 1500
GLITCH_FILTER_US = 100_000
MIN_INTER_EDGE_S = 0.25
RUN_S = 4.0

# Half-offsets from center. Wind = 1500 - d, unwind = 1500 + d.
OFFSETS = [50, 60, 70, 80, 90, 100, 110, 120]
RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"
OUT = Path(os.environ.get(
    "ROT_CAL_OUT",
    f"/home/pi/rot_cal_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}",
))


class Pca:
    def __init__(self):
        self.bus = SMBus(1)
        self._w(0xFD, 0x10)
        m = self._r(0x00)
        self._w(0x00, (m & ~0x80) | 0x10)
        time.sleep(0.001)
        self._w(0x00, 0x10 | 0x40 | 0x20)
        ps = max(3, min(255, int(math.ceil(24_576_000 / (4096 * 50)) - 1)))
        self._w(0xFE, ps)
        self._w(0x01, 0x04)
        self._w(0x00, 0x40 | 0x20 | 0x80)
        time.sleep(0.001)

    def _w(self, r, v):
        self.bus.write_byte_data(PCA9685_ADDR, r, v & 0xFF)

    def _r(self, r):
        return self.bus.read_byte_data(PCA9685_ADDR, r)

    def set_pulse(self, us):
        counts = int(round(max(0, min(20000, us)) / 20000 * 4096))
        counts = max(0, min(4095, counts))
        base = 0x06 + 4 * RELEASE_CH
        self.bus.write_i2c_block_data(
            PCA9685_ADDR, base, [0, 0, counts & 0xFF, (counts >> 8) & 0x1F]
        )

    def stop(self):
        self.set_pulse(STOP_US)

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        self.bus.close()


class Counter:
    def __init__(self, h):
        self.count = 0
        self.lock = threading.Lock()
        self._last = None
        lgpio.gpio_claim_alert(h, ROT_GPIO, lgpio.FALLING_EDGE, lgpio.SET_PULL_NONE)
        if hasattr(lgpio, "gpio_set_debounce_micros"):
            lgpio.gpio_set_debounce_micros(h, ROT_GPIO, GLITCH_FILTER_US)
        lgpio.callback(h, ROT_GPIO, lgpio.FALLING_EDGE, self._cb)

    def _cb(self, *a):
        now = time.monotonic()
        with self.lock:
            if self._last and (now - self._last) < MIN_INTER_EDGE_S:
                return
            self._last = now
            self.count += 1

    def get(self):
        with self.lock:
            return self.count


def snap(path: Path) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["docker", "rm", "-f", "rot-diag-ffmpeg"], capture_output=True)
    r = subprocess.run(
        ["docker", "run", "--rm", "--network", "host",
         "--name", "rot-diag-ffmpeg", "--entrypoint", "ffmpeg",
         "-v", f"{path.parent}:/out",
         "vshie/blueos-blueos_video_recorder:dropcam",
         "-y", "-loglevel", "error", "-rtsp_transport", "tcp",
         "-i", RTSP, "-frames:v", "1", "-q:v", "2", f"/out/{path.name}"],
        capture_output=True, text=True,
    )
    ok = path.exists() and path.stat().st_size > 1000
    print(f"  snap {'OK' if ok else 'FAIL'} {path.name}")
    return ok


def ext_ids():
    out = subprocess.check_output(
        ["docker", "ps", "--format", "{{.ID}} {{.Names}}"], text=True
    )
    ids = []
    for line in out.splitlines():
        p = line.split(None, 1)
        if len(p) == 2 and "videorecorder" in p[1] and "rot-diag-ffmpeg" not in p[1]:
            ids.append(p[0])
    return ids


def ensure_stopped(timeout=30):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ids = ext_ids()
        if not ids:
            return
        for i in ids:
            subprocess.run(["docker", "update", "--restart=no", i], capture_output=True)
            subprocess.run(["docker", "stop", "-t", "1", i], capture_output=True)
        time.sleep(0.4)
    raise SystemExit("cannot stop extension")


def run_fixed(pca, counter, pwm, seconds):
    c0 = counter.get()
    pca.set_pulse(pwm)
    time.sleep(seconds)
    pca.stop()
    time.sleep(0.35)
    edges = counter.get() - c0
    rpm = edges / seconds * 60.0
    return edges, rpm


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "snaps").mkdir(exist_ok=True)
    print(f"OUT={OUT}")
    ensure_stopped()
    time.sleep(1)
    ensure_stopped()

    stop = threading.Event()

    def guard():
        while not stop.is_set():
            for i in ext_ids():
                subprocess.run(["docker", "stop", "-t", "0", i], capture_output=True)
            stop.wait(0.5)

    threading.Thread(target=guard, daemon=True).start()

    if not snap(OUT / "snaps" / "00_preflight.jpg"):
        raise SystemExit("MCM RTSP down")

    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    subprocess.run(["raspi-gpio", "set", "3", "a0"], check=False)
    h = lgpio.gpiochip_open(0)
    try:
        lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
    except Exception:
        lgpio.gpiochip_close(h)
        h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)

    pca = Pca()
    pca.stop()
    counter = Counter(h)

    wind_rows = []
    unwind_rows = []

    print("\n=== WIND sweep (below 1500) ===")
    for d in OFFSETS:
        us = STOP_US - d
        snap(OUT / "snaps" / f"wind_{us}_before.jpg")
        edges, rpm = run_fixed(pca, counter, us, RUN_S)
        snap(OUT / "snaps" / f"wind_{us}_after.jpg")
        print(f"  wind {us}: edges={edges} rpm≈{rpm:.1f}")
        wind_rows.append({"us": us, "offset": d, "edges": edges, "rpm": round(rpm, 2)})
        time.sleep(0.4)

    print("\n=== UNWIND sweep (above 1500) ===")
    for d in OFFSETS:
        us = STOP_US + d
        snap(OUT / "snaps" / f"unwind_{us}_before.jpg")
        edges, rpm = run_fixed(pca, counter, us, RUN_S)
        snap(OUT / "snaps" / f"unwind_{us}_after.jpg")
        print(f"  unwind {us}: edges={edges} rpm≈{rpm:.1f}")
        unwind_rows.append({"us": us, "offset": d, "edges": edges, "rpm": round(rpm, 2)})
        time.sleep(0.4)

    # Prefer pairs with useful speed (edges >= 2 in RUN_S ≈ >=30 RPM)
    # and minimize |wind_edges - unwind_edges|, then |rpm_delta|.
    candidates = []
    for w in wind_rows:
        for u in unwind_rows:
            if min(w["edges"], u["edges"]) < 2:
                continue
            candidates.append({
                "wind_us": w["us"],
                "unwind_us": u["us"],
                "wind_edges": w["edges"],
                "unwind_edges": u["edges"],
                "wind_rpm": w["rpm"],
                "unwind_rpm": u["rpm"],
                "edge_delta": abs(w["edges"] - u["edges"]),
                "rpm_delta": round(abs(w["rpm"] - u["rpm"]), 2),
                # Prefer mid-range speeds over max (gentler on string)
                "speed_score": abs(((w["edges"] + u["edges"]) / 2) - 3.5),
            })
    if not candidates:
        # Fall back: any pair with closest edges
        for w in wind_rows:
            for u in unwind_rows:
                candidates.append({
                    "wind_us": w["us"],
                    "unwind_us": u["us"],
                    "wind_edges": w["edges"],
                    "unwind_edges": u["edges"],
                    "wind_rpm": w["rpm"],
                    "unwind_rpm": u["rpm"],
                    "edge_delta": abs(w["edges"] - u["edges"]),
                    "rpm_delta": round(abs(w["rpm"] - u["rpm"]), 2),
                    "speed_score": 0,
                })

    best = min(candidates, key=lambda c: (c["edge_delta"], c["rpm_delta"], c["speed_score"]))
    print("\n=== WIND table ===")
    for r in wind_rows:
        print(f"  {r['us']}: {r['edges']}e / {r['rpm']:.1f}rpm")
    print("=== UNWIND table ===")
    for r in unwind_rows:
        print(f"  {r['us']}: {r['edges']}e / {r['rpm']:.1f}rpm")
    print(f"\nBEST match: wind={best['wind_us']} ({best['wind_edges']}e/"
          f"{best['wind_rpm']:.1f}rpm)  unwind={best['unwind_us']} "
          f"({best['unwind_edges']}e/{best['unwind_rpm']:.1f}rpm)  "
          f"|Δe|={best['edge_delta']}")

    # Verification: re-run best pair back-to-back with snaps
    print("\n=== VERIFY best pair (back-to-back) ===")
    snap(OUT / "snaps" / "verify_wind_before.jpg")
    ve_w, vr_w = run_fixed(pca, counter, best["wind_us"], RUN_S)
    snap(OUT / "snaps" / "verify_wind_after.jpg")
    print(f"  verify wind   {best['wind_us']}: {ve_w}e / {vr_w:.1f}rpm")
    time.sleep(0.5)
    snap(OUT / "snaps" / "verify_unwind_before.jpg")
    ve_u, vr_u = run_fixed(pca, counter, best["unwind_us"], RUN_S)
    snap(OUT / "snaps" / "verify_unwind_after.jpg")
    print(f"  verify unwind {best['unwind_us']}: {ve_u}e / {vr_u:.1f}rpm")

    rec = {
        "WINCH_WIND_US": best["wind_us"],
        "WINCH_UNWIND_US": best["unwind_us"],
        "WINCH_WIND_RPM": int(round(best["wind_rpm"])),
        "WINCH_UNWIND_RPM": int(round(best["unwind_rpm"])),
        "run_s": RUN_S,
        "wind_rows": wind_rows,
        "unwind_rows": unwind_rows,
        "best": best,
        "verify": {
            "wind_edges": ve_w, "wind_rpm": round(vr_w, 2),
            "unwind_edges": ve_u, "unwind_rpm": round(vr_u, 2),
        },
    }
    (OUT / "calibration.json").write_text(json.dumps(rec, indent=2))
    print(f"\nRecommended: WINCH_WIND_US={rec['WINCH_WIND_US']} "
          f"WINCH_UNWIND_US={rec['WINCH_UNWIND_US']} "
          f"RPM≈{rec['WINCH_WIND_RPM']}/{rec['WINCH_UNWIND_RPM']}")
    print(f"Wrote {OUT / 'calibration.json'}")

    stop.set()
    pca.close()
    lgpio.gpiochip_close(h)
    print("Done — restart videorecorder when finished reviewing snaps.")


if __name__ == "__main__":
    main()
