#!/usr/bin/env python3
"""Standalone rotation diagnostic — mirrors the extension's method.

Requires the videorecorder extension to be FULLY STOPPED so GPIO 10 and
PCA9685 are free. Uses the same closed-loop approach as hardware.py:

  * PCA9685 ch2 pulse = WINCH_UNWIND_US (1560) / WINCH_WIND_US (1400)
  * GPIO 10 falling-edge = 1 rotation (analog 0→3.3 V ramp into digital
    Schmitt input; we count the snap-back crossing)
  * 100 ms lgpio debounce + 250 ms software min-inter-edge
  * stall abort at 2.5 s with zero edges

Also records a ~2 kHz digital sample of GPIO 10 and grabs MCM RTSP
snapshots after each move so we can visually verify the W-disc position
(camera stream stays up independent of the extension).
"""

from __future__ import annotations

import csv
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

# --- same constants as app/hardware.py ---
RELEASE_CH = 2
PCA9685_ADDR = 0x40
PCA9685_OE_GPIO = 4
ROT_GPIO = 10
WINCH_UNWIND_US = 1560
WINCH_WIND_US = 1400
RELEASE_STOP_US = 1500
GLITCH_FILTER_US = 100_000
MIN_INTER_EDGE_S = 0.25
STALL_S = 2.5
MAX_DURATION_S = 60.0

MODE1, MODE2, PRESCALE = 0x00, 0x01, 0xFE
LED0_ON_L, ALL_LED_OFF_H = 0x06, 0xFD
MODE1_RESTART, MODE1_EXTCLK, MODE1_AI, MODE1_SLEEP = 0x80, 0x40, 0x20, 0x10
MODE2_OUTDRV = 0x04
EXT_CLOCK_HZ = 24_576_000
STEPS = 4096
FREQ_HZ = 50

OUT = Path(os.environ.get(
    "ROT_DIAG_OUT",
    f"/home/pi/rot_diag_extstyle_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}",
))
RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"
SAMPLE_HZ = 2000

MOVES = [
    ("unwind", 1),
    ("wind", 1),
    ("unwind", 3),
    ("wind", 3),
    ("unwind", 5),
    ("wind", 5),
]


class Pca:
    def __init__(self):
        self.bus = SMBus(1)
        self._init()

    def _w(self, reg, val):
        self.bus.write_byte_data(PCA9685_ADDR, reg, val & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(PCA9685_ADDR, reg)

    def _init(self):
        self._w(ALL_LED_OFF_H, 0x10)
        mode1 = self._r(MODE1)
        self._w(MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP)
        time.sleep(0.001)
        self._w(MODE1, MODE1_SLEEP | MODE1_EXTCLK | MODE1_AI)
        raw = EXT_CLOCK_HZ / (STEPS * FREQ_HZ)
        prescale = max(3, min(255, int(math.ceil(raw) - 1)))
        self._w(PRESCALE, prescale)
        self._w(MODE2, MODE2_OUTDRV)
        self._w(MODE1, MODE1_EXTCLK | MODE1_AI | MODE1_RESTART)
        time.sleep(0.001)
        print(f"PCA9685 ready: prescale={prescale}")

    def set_pulse(self, ch: int, us: float) -> None:
        us = max(0.0, float(us))
        period_us = 1e6 / FREQ_HZ
        if us <= 0:
            on, off = 0x1000, 0
        elif us >= period_us:
            on, off = 0, 0x1000
        else:
            counts = int(round(us / period_us * STEPS))
            counts = max(0, min(STEPS - 1, counts))
            on, off = 0, counts
        base = LED0_ON_L + 4 * ch
        self.bus.write_i2c_block_data(
            PCA9685_ADDR, base,
            [on & 0xFF, (on >> 8) & 0x1F, off & 0xFF, (off >> 8) & 0x1F],
        )

    def close(self):
        try:
            self.set_pulse(RELEASE_CH, RELEASE_STOP_US)
        except Exception:
            pass
        self.bus.close()


class RotCounter:
    """Same counting model as HardwareController._on_rotation_edge."""

    def __init__(self, chip_handle: int):
        self.h = chip_handle
        self.count = 0
        self.lock = threading.Lock()
        self._last_edge_mono: float | None = None
        self.edge_log: list[tuple[float, int]] = []
        lgpio.gpio_claim_alert(
            self.h, ROT_GPIO, lgpio.FALLING_EDGE, lgpio.SET_PULL_NONE
        )
        if hasattr(lgpio, "gpio_set_debounce_micros"):
            lgpio.gpio_set_debounce_micros(self.h, ROT_GPIO, GLITCH_FILTER_US)
        lgpio.callback(self.h, ROT_GPIO, lgpio.FALLING_EDGE, self._cb)

    def _cb(self, chip, gpio, level, tick):
        now = time.monotonic()
        with self.lock:
            if self._last_edge_mono is not None:
                if (now - self._last_edge_mono) < MIN_INTER_EDGE_S:
                    return
            self._last_edge_mono = now
            self.count += 1
            self.edge_log.append((now, self.count))

    def get(self) -> int:
        with self.lock:
            return self.count


def snapshot(path: Path) -> bool:
    """Grab one JPEG from BlueOS MCM RTSP (independent of the extension)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    # Distinct container name so the extension-stop guard never kills us.
    subprocess.run(["docker", "rm", "-f", "rot-diag-ffmpeg"],
                   capture_output=True)
    r = subprocess.run(
        ["docker", "run", "--rm", "--network", "host",
         "--name", "rot-diag-ffmpeg",
         "--entrypoint", "ffmpeg",
         "-v", f"{path.parent}:/out",
         "vshie/blueos-blueos_video_recorder:dropcam",
         "-y", "-loglevel", "error",
         "-rtsp_transport", "tcp",
         "-i", RTSP,
         "-frames:v", "1", "-q:v", "2",
         f"/out/{path.name}"],
        capture_output=True, text=True,
    )
    ok = path.exists() and path.stat().st_size > 1000
    if not ok:
        print(f"  snapshot fail {path.name}: rc={r.returncode} err={r.stderr[:300]!r}")
    else:
        print(f"  snapshot OK {path.name} ({path.stat().st_size} bytes)")
    return ok


def _extension_container_ids() -> list[str]:
    """Return running extension container IDs (never rot-diag-ffmpeg)."""
    out = subprocess.check_output(
        ["docker", "ps", "--format", "{{.ID}} {{.Names}}"],
        text=True,
    )
    ids = []
    for line in out.splitlines():
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        cid, name = parts
        if "rot-diag-ffmpeg" in name:
            continue
        if "videorecorder" in name or (
            name.startswith("extension-") and "video" in name.lower()
        ):
            ids.append(cid)
    return ids


def ensure_extension_stopped(timeout_s: float = 30.0) -> None:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        ids = _extension_container_ids()
        if not ids:
            print("extension stopped")
            return
        for i in ids:
            subprocess.run(["docker", "update", "--restart=no", i],
                           capture_output=True)
            subprocess.run(["docker", "stop", "-t", "1", i],
                           capture_output=True)
        time.sleep(0.5)
    raise SystemExit("ERROR: could not keep extension stopped (kraken respawn)")


def run_for_rotations(pca: Pca, counter: RotCounter, pwm_us: int, target: int,
                      t0: float) -> dict:
    start = counter.get()
    t_start = time.monotonic()
    pca.set_pulse(RELEASE_CH, pwm_us)
    last_progress = t_start
    outcome = "timeout"
    while True:
        now = time.monotonic()
        delivered = counter.get() - start
        if delivered >= target:
            outcome = "target_reached"
            break
        if delivered > 0:
            last_progress = now
        elif (now - last_progress) >= STALL_S:
            outcome = "sensor_stalled"
            break
        if (now - t_start) >= MAX_DURATION_S:
            outcome = "timeout"
            break
        time.sleep(0.02)
    pca.set_pulse(RELEASE_CH, RELEASE_STOP_US)
    t_end = time.monotonic()
    delivered = counter.get() - start
    edges = [
        (et - t0) for et, c in counter.edge_log
        if t_start <= et <= t_end and (c - start) <= target
    ]
    return {
        "outcome": outcome,
        "commanded": target,
        "delivered": delivered,
        "error_rotations": delivered - target,
        "t_start": t_start - t0,
        "t_end": t_end - t0,
        "duration_s": t_end - t_start,
        "pwm_us": pwm_us,
        "edge_times": edges,
    }


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "snaps").mkdir(exist_ok=True)
    print(f"OUT={OUT}")

    ensure_extension_stopped()
    time.sleep(1.0)
    ensure_extension_stopped()

    stop_guard = threading.Event()

    def guard():
        while not stop_guard.is_set():
            for cid in _extension_container_ids():
                subprocess.run(["docker", "stop", "-t", "0", cid],
                               capture_output=True)
            stop_guard.wait(0.5)

    guard_th = threading.Thread(target=guard, daemon=True)
    guard_th.start()

    # MCM RTSP must work before we move anything
    print("MCM RTSP check…")
    if not snapshot(OUT / "snaps" / "00_start.jpg"):
        stop_guard.set()
        raise SystemExit("ERROR: cannot snapshot from MCM RTSP — aborting")

    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    subprocess.run(["raspi-gpio", "set", "3", "a0"], check=False)

    h = lgpio.gpiochip_open(0)
    try:
        lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
    except Exception:
        lgpio.gpiochip_close(h)
        h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
    print("PCA9685 ~OE driven LOW")

    pca = Pca()
    pca.set_pulse(RELEASE_CH, RELEASE_STOP_US)
    counter = RotCounter(h)

    samples: list[tuple[float, int, str, str]] = []
    phase = ["idle"]
    move_id = [""]
    stop_samp = threading.Event()
    t0 = time.monotonic()

    def sample_loop():
        period = 1.0 / SAMPLE_HZ
        nxt = time.monotonic()
        while not stop_samp.is_set():
            lvl = lgpio.gpio_read(h, ROT_GPIO)
            samples.append((time.monotonic() - t0, lvl, phase[0], move_id[0]))
            nxt += period
            dt = nxt - time.monotonic()
            if dt > 0:
                time.sleep(dt)
            else:
                nxt = time.monotonic()

    th = threading.Thread(target=sample_loop, daemon=True)
    th.start()
    time.sleep(0.3)

    moves_meta = []
    for i, (direction, n) in enumerate(MOVES, 1):
        mid = f"{i:02d}_{direction}_{n}"
        pwm = WINCH_UNWIND_US if direction == "unwind" else WINCH_WIND_US
        print(f"\n=== {mid} pwm={pwm} ===")
        move_id[0] = mid
        phase[0] = "pre"
        time.sleep(0.4)
        phase[0] = "moving"
        meta = run_for_rotations(pca, counter, pwm, n, t0)
        meta["move_id"] = mid
        meta["direction"] = direction
        moves_meta.append(meta)
        print(
            f"  outcome={meta['outcome']} delivered={meta['delivered']}/{n} "
            f"err={meta['error_rotations']:+d} dur={meta['duration_s']:.2f}s "
            f"edges={len(meta['edge_times'])}"
        )
        phase[0] = "settle"
        time.sleep(1.0)  # let disc stop before photo
        phase[0] = "snap"
        snapshot(OUT / "snaps" / f"{mid}.jpg")
        phase[0] = "idle"
        move_id[0] = ""
        time.sleep(0.3)

    stop_samp.set()
    th.join(timeout=2)
    stop_guard.set()
    guard_th.join(timeout=2)
    pca.close()
    lgpio.gpiochip_close(h)

    with open(OUT / "samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "gpio10", "phase", "move_id"])
        w.writerows(samples)
    with open(OUT / "moves.json", "w") as f:
        json.dump(moves_meta, f, indent=2)

    print("\n=== SUMMARY ===")
    for m in moves_meta:
        print(
            f"  {m['move_id']:20s} {m['outcome']:16s} "
            f"got={m['delivered']}/{m['commanded']} err={m['error_rotations']:+d} "
            f"dur={m['duration_s']:.2f}s"
        )
    print(f"\nDone → {OUT}")
    print("Compare snaps/*/W-arrow clock position — integer counts can still be wrong by ~½ turn.")
    print("Restart the videorecorder extension when finished.")


if __name__ == "__main__":
    main()
