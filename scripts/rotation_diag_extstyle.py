#!/usr/bin/env python3
"""Standalone rotation diagnostic — mirrors the extension's method.

Requires the videorecorder extension to be FULLY STOPPED so GPIO 10 and
PCA9685 are free. Uses the same closed-loop approach as hardware.py:

  * PCA9685 ch2 pulse = WINCH_UNWIND_US (1560) / WINCH_WIND_US (1400)
  * GPIO 10 falling-edge = 1 rotation (analog 0→3.3 V ramp into digital
    Schmitt input; we count the snap-back crossing)
  * 100 ms lgpio debounce + 250 ms software min-inter-edge
  * stall abort at 2.5 s with zero edges

Also records a ~2 kHz digital sample of GPIO 10 so we can see the
threshold crossings (the hacky on/off view of the analog sweep).
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

# PCA9685 regs (same as pca9685.py)
MODE1, MODE2, PRESCALE = 0x00, 0x01, 0xFE
LED0_ON_L, ALL_LED_OFF_H = 0x06, 0xFD
MODE1_RESTART, MODE1_EXTCLK, MODE1_AI, MODE1_SLEEP = 0x80, 0x40, 0x20, 0x10
MODE2_OUTDRV = 0x04
FULL_BIT = 0x1000
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
        # Shut all outputs, then external clock @ 50 Hz (mirrors pca9685.py)
        self._w(ALL_LED_OFF_H, 0x10)  # full-off all
        mode1 = self._r(MODE1)
        self._w(MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP)
        time.sleep(0.001)
        self._w(MODE1, MODE1_SLEEP | MODE1_EXTCLK | MODE1_AI)
        # prescale = ceil(ext/(4096*freq)) - 1
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
            on, off = FULL_BIT, 0
        elif us >= period_us:
            on, off = 0, FULL_BIT
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
        self.edge_log: list[tuple[float, int]] = []  # (mono_t, count_after)
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
    """Grab one JPEG via ffmpeg ONLY (never start the extension entrypoint)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    r = subprocess.run(
        ["docker", "run", "--rm", "--network", "host",
         "--entrypoint", "ffmpeg",
         "-v", f"{path.parent}:/out",
         "vshie/blueos-blueos_video_recorder:dropcam",
         "-y", "-loglevel", "error", "-rtsp_transport", "tcp",
         "-i", RTSP, "-frames:v", "1", "-q:v", "2", f"/out/{path.name}"],
        capture_output=True, text=True,
    )
    ok = path.exists() and path.stat().st_size > 1000
    if not ok:
        print(f"  snapshot fail {path.name}: rc={r.returncode} err={r.stderr[:200]!r}")
    return ok


def shutil_which(cmd: str):
    from shutil import which
    return which(cmd)


def run_for_rotations(pca: Pca, counter: RotCounter, pwm_us: int, target: int,
                      sampler_phase: list, t0: float) -> dict:
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
    # edge times relative to t0
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


def ensure_extension_stopped(timeout_s: float = 30.0) -> None:
    """Stop every videorecorder container and wait until none remain."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        ids = subprocess.check_output(
            ["docker", "ps", "-q", "--filter",
             "ancestor=vshie/blueos-blueos_video_recorder:dropcam"],
            text=True,
        ).split()
        if not ids:
            print("extension stopped (no videorecorder containers)")
            return
        for i in ids:
            subprocess.run(["docker", "update", "--restart=no", i],
                           capture_output=True)
            subprocess.run(["docker", "stop", "-t", "1", i],
                           capture_output=True)
        time.sleep(0.5)
    raise SystemExit("ERROR: could not keep videorecorder stopped (kraken respawn)")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "snaps").mkdir(exist_ok=True)
    print(f"OUT={OUT}")

    ensure_extension_stopped()
    # Brief settle so GPIO/I2C are released
    time.sleep(1.0)
    ensure_extension_stopped()

    # Keep kraken from reclaiming GPIO mid-test
    stop_guard = threading.Event()

    def guard():
        while not stop_guard.is_set():
            ids = subprocess.check_output(
                ["docker", "ps", "-q", "--filter",
                 "ancestor=vshie/blueos-blueos_video_recorder:dropcam"],
                text=True,
            ).split()
            for i in ids:
                subprocess.run(["docker", "stop", "-t", "0", i],
                               capture_output=True)
            stop_guard.wait(0.5)

    guard_th = threading.Thread(target=guard, daemon=True)
    guard_th.start()

    # Recover I2C pinmux (ardupilot may have stolen GPIO 2)
    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    subprocess.run(["raspi-gpio", "set", "3", "a0"], check=False)

    # OE low so PCA9685 outputs drive
    h = lgpio.gpiochip_open(0)
    try:
        lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
    except Exception:
        # already claimed / chip index — try chip 4 on some Pis
        lgpio.gpiochip_close(h)
        h = lgpio.gpiochip_open(4)
        lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
    print("PCA9685 ~OE driven LOW")

    pca = Pca()
    pca.set_pulse(RELEASE_CH, RELEASE_STOP_US)

    # Separate chip handle for alert+read: claim input for sampling via
    # gpio_read on the alert-claimed line (lgpio allows read on alert pins).
    counter = RotCounter(h)

    # High-rate digital sample of threshold crossings
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

    snapshot(OUT / "snaps" / "00_start.jpg")
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
        meta = run_for_rotations(pca, counter, pwm, n, phase, t0)
        meta["move_id"] = mid
        meta["direction"] = direction
        moves_meta.append(meta)
        print(
            f"  outcome={meta['outcome']} delivered={meta['delivered']}/{n} "
            f"err={meta['error_rotations']:+d} dur={meta['duration_s']:.2f}s "
            f"edges={len(meta['edge_times'])}"
        )
        phase[0] = "settle"
        time.sleep(0.8)
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
    with open(OUT / "edges.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["move_id", "direction", "commanded", "edge_index", "t_s"])
        for m in moves_meta:
            for j, et in enumerate(m["edge_times"], 1):
                w.writerow([m["move_id"], m["direction"], m["commanded"], j, f"{et:.6f}"])

    # Pulse-width summary of digital lows during moves (threshold view of analog sweep)
    print("\n=== GPIO low-run widths during moves (ms @ 2 kHz) ===")
    for m in moves_meta:
        win = [s for s in samples if m["t_start"] <= s[0] <= m["t_end"]]
        levels = [s[1] for s in win]
        runs = []
        if levels:
            cur, n = levels[0], 1
            for v in levels[1:]:
                if v == cur:
                    n += 1
                else:
                    runs.append((cur, n))
                    cur, n = v, 1
            runs.append((cur, n))
        lows_ms = [n / SAMPLE_HZ * 1000 for lvl, n in runs if lvl == 0]
        print(
            f"  {m['move_id']}: low_pulses={len(lows_ms)} "
            f"widths_ms={[round(x,1) for x in lows_ms[:12]]} "
            f"max={round(max(lows_ms),1) if lows_ms else 0}"
        )

    print("\n=== SUMMARY ===")
    for m in moves_meta:
        print(
            f"  {m['move_id']:20s} {m['outcome']:16s} "
            f"got={m['delivered']}/{m['commanded']} err={m['error_rotations']:+d} "
            f"dur={m['duration_s']:.2f}s"
        )
    print(f"\nDone → {OUT}")
    print("NOTE: restart the videorecorder extension when finished.")


if __name__ == "__main__":
    main()
