#!/usr/bin/env python3
"""A/B test rotation stop strategies with MCM camera verification.

Extension must be kept stopped (script guards that). Strategies:

  baseline   — stop on Nth falling edge (current behaviour)
  strat1     — after a direction reverse, ignore the first edge; then
               overrun by ~0.25 of the last inter-edge period (phase/
               inertia compensation without reversing)
  strat2     — after N edges in the commanded direction, if that
               direction was not WIND, reverse to WIND and creep until
               one wind-edge (canonical approach). Wind moves already
               stop on a wind-edge.

Sequence per strategy (camera snap after each move):
  start → U1 → W1 → U3 → W3 → U5 → W5

Success metric: after every wind leg, W-marker should land near the
same clock position; after every unwind+canonical (strat2) likewise.
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
WINCH_UNWIND_US = 1560
WINCH_WIND_US = 1400
RELEASE_STOP_US = 1500
GLITCH_FILTER_US = 100_000
MIN_INTER_EDGE_S = 0.25
STALL_S = 2.5
MAX_DURATION_S = 60.0
CANONICAL_DIR = "wind"  # strat2 always finishes approaching this way

MODE1, MODE2, PRESCALE = 0x00, 0x01, 0xFE
LED0_ON_L, ALL_LED_OFF_H = 0x06, 0xFD
MODE1_RESTART, MODE1_EXTCLK, MODE1_AI, MODE1_SLEEP = 0x80, 0x40, 0x20, 0x10
MODE2_OUTDRV = 0x04
EXT_CLOCK_HZ = 24_576_000
STEPS = 4096
FREQ_HZ = 50

RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"
OUT = Path(os.environ.get(
    "ROT_AB_OUT",
    f"/home/pi/rot_ab_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}",
))

MOVES = [
    ("unwind", 1),
    ("wind", 1),
    ("unwind", 3),
    ("wind", 3),
    ("unwind", 5),
    ("wind", 5),
]
STRATEGIES = ("baseline", "strat1", "strat2")


def pwm_for(direction: str) -> int:
    return WINCH_UNWIND_US if direction == "unwind" else WINCH_WIND_US


class Pca:
    def __init__(self):
        self.bus = SMBus(1)
        self._w(ALL_LED_OFF_H, 0x10)
        mode1 = self._r(MODE1)
        self._w(MODE1, (mode1 & ~MODE1_RESTART) | MODE1_SLEEP)
        time.sleep(0.001)
        self._w(MODE1, MODE1_SLEEP | MODE1_EXTCLK | MODE1_AI)
        prescale = max(3, min(255, int(math.ceil(EXT_CLOCK_HZ / (STEPS * FREQ_HZ)) - 1)))
        self._w(PRESCALE, prescale)
        self._w(MODE2, MODE2_OUTDRV)
        self._w(MODE1, MODE1_EXTCLK | MODE1_AI | MODE1_RESTART)
        time.sleep(0.001)

    def _w(self, reg, val):
        self.bus.write_byte_data(PCA9685_ADDR, reg, val & 0xFF)

    def _r(self, reg):
        return self.bus.read_byte_data(PCA9685_ADDR, reg)

    def set_pulse(self, us: float) -> None:
        period_us = 1e6 / FREQ_HZ
        counts = int(round(max(0.0, min(period_us, float(us))) / period_us * STEPS))
        counts = max(0, min(STEPS - 1, counts))
        base = LED0_ON_L + 4 * RELEASE_CH
        self.bus.write_i2c_block_data(
            PCA9685_ADDR, base,
            [0, 0, counts & 0xFF, (counts >> 8) & 0x1F],
        )

    def stop(self):
        self.set_pulse(RELEASE_STOP_US)

    def close(self):
        try:
            self.stop()
        except Exception:
            pass
        self.bus.close()


class RotCounter:
    def __init__(self, h):
        self.h = h
        self.count = 0
        self.lock = threading.Lock()
        self._last = None
        self.edge_times: list[float] = []
        lgpio.gpio_claim_alert(h, ROT_GPIO, lgpio.FALLING_EDGE, lgpio.SET_PULL_NONE)
        if hasattr(lgpio, "gpio_set_debounce_micros"):
            lgpio.gpio_set_debounce_micros(h, ROT_GPIO, GLITCH_FILTER_US)
        lgpio.callback(h, ROT_GPIO, lgpio.FALLING_EDGE, self._cb)

    def _cb(self, *args):
        now = time.monotonic()
        with self.lock:
            if self._last is not None and (now - self._last) < MIN_INTER_EDGE_S:
                return
            self._last = now
            self.count += 1
            self.edge_times.append(now)

    def get(self) -> int:
        with self.lock:
            return self.count


def snapshot(path: Path) -> bool:
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


def extension_ids() -> list[str]:
    out = subprocess.check_output(
        ["docker", "ps", "--format", "{{.ID}} {{.Names}}"], text=True
    )
    ids = []
    for line in out.splitlines():
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        cid, name = parts
        if "rot-diag-ffmpeg" in name:
            continue
        if "videorecorder" in name:
            ids.append(cid)
    return ids


def ensure_stopped(timeout_s=30.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        ids = extension_ids()
        if not ids:
            return
        for i in ids:
            subprocess.run(["docker", "update", "--restart=no", i], capture_output=True)
            subprocess.run(["docker", "stop", "-t", "1", i], capture_output=True)
        time.sleep(0.4)
    raise SystemExit("cannot keep extension stopped")


def wait_edges(counter: RotCounter, start: int, need: int, pca: Pca,
               pwm: int, label: str) -> tuple[str, float, list[float]]:
    """Drive pwm until need new edges. Returns (outcome, duration, edge_times)."""
    t0 = time.monotonic()
    pca.set_pulse(pwm)
    last_progress = t0
    edges: list[float] = []
    outcome = "timeout"
    while True:
        now = time.monotonic()
        got = counter.get() - start
        if got > len(edges):
            # newly arrived edges
            with counter.lock:
                edges = [t for t in counter.edge_times if t >= t0][:need]
            last_progress = now
        if got >= need:
            outcome = "target_reached"
            break
        if got == 0 and (now - last_progress) >= STALL_S:
            outcome = "sensor_stalled"
            break
        if (now - t0) >= MAX_DURATION_S:
            break
        time.sleep(0.02)
    pca.stop()
    return outcome, time.monotonic() - t0, edges


def run_move(pca: Pca, counter: RotCounter, direction: str, n: int,
             strategy: str, prev_direction: str | None) -> dict:
    pwm = pwm_for(direction)
    t_wall0 = time.monotonic()
    detail = {"strategy": strategy, "direction": direction, "commanded": n,
              "pwm_us": pwm, "prev_direction": prev_direction}

    if strategy == "baseline":
        start = counter.get()
        outcome, dur, edges = wait_edges(counter, start, n, pca, pwm, "main")
        detail.update(outcome=outcome, duration_s=dur, edges=len(edges),
                      phase="main_only")

    elif strategy == "strat1":
        # Ignore first edge after a reverse (common false/near-edge count).
        ignore = 1 if (prev_direction and prev_direction != direction) else 0
        need = n + ignore
        start = counter.get()
        outcome, dur, edges = wait_edges(counter, start, need, pca, pwm, "main")
        # Overrun ~0.25 of last inter-edge (same direction) for coast/phase.
        overrun_s = 0.0
        if outcome == "target_reached" and len(edges) >= 2:
            interval = edges[-1] - edges[-2]
            overrun_s = max(0.05, min(0.8, 0.25 * interval))
            pca.set_pulse(pwm)
            time.sleep(overrun_s)
            pca.stop()
        elif outcome == "target_reached" and len(edges) == 1:
            # single-edge move: use nominal 52 RPM → 0.25 turn ≈ 0.29 s
            overrun_s = 0.29
            pca.set_pulse(pwm)
            time.sleep(overrun_s)
            pca.stop()
        detail.update(outcome=outcome, duration_s=time.monotonic() - t_wall0,
                      edges_raw=len(edges), ignored=ignore,
                      counted=max(0, len(edges) - ignore),
                      overrun_s=overrun_s, phase="ignore_first+overrun")

    elif strategy == "strat2":
        # Drive commanded direction for N edges, then if needed finish on
        # canonical WIND edge.
        start = counter.get()
        outcome, dur_main, edges = wait_edges(counter, start, n, pca, pwm, "main")
        finish = {"needed": False, "outcome": None, "duration_s": 0.0, "edges": 0}
        if outcome == "target_reached" and direction != CANONICAL_DIR:
            finish["needed"] = True
            # Brief pause so the shaft settles before reverse approach
            time.sleep(0.15)
            start2 = counter.get()
            out2, dur2, edges2 = wait_edges(
                counter, start2, 1, pca, pwm_for(CANONICAL_DIR), "canonical"
            )
            finish.update(outcome=out2, duration_s=dur2, edges=len(edges2))
            if out2 != "target_reached":
                outcome = f"canonical_{out2}"
        detail.update(outcome=outcome, duration_s=time.monotonic() - t_wall0,
                      edges_main=len(edges), finish=finish,
                      phase="canonical_wind_finish")
    else:
        raise ValueError(strategy)

    return detail


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    print(f"OUT={OUT}")
    ensure_stopped()
    time.sleep(1)
    ensure_stopped()

    stop_guard = threading.Event()

    def guard():
        while not stop_guard.is_set():
            for cid in extension_ids():
                subprocess.run(["docker", "stop", "-t", "0", cid], capture_output=True)
            stop_guard.wait(0.5)

    threading.Thread(target=guard, daemon=True).start()

    if not snapshot(OUT / "00_preflight.jpg"):
        raise SystemExit("MCM RTSP unavailable")

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
    counter = RotCounter(h)
    results = []

    for strat in STRATEGIES:
        print(f"\n######## STRATEGY {strat} ########")
        snap_dir = OUT / strat
        snap_dir.mkdir(exist_ok=True)
        snapshot(snap_dir / "00_start.jpg")
        prev = None
        for i, (direction, n) in enumerate(MOVES, 1):
            mid = f"{i:02d}_{direction}_{n}"
            print(f"=== {strat} {mid} ===")
            meta = run_move(pca, counter, direction, n, strat, prev)
            meta["move_id"] = mid
            print(f"  {json.dumps({k: meta[k] for k in meta if k != 'finish'}, default=str)}")
            if "finish" in meta:
                print(f"  finish={meta['finish']}")
            time.sleep(0.8)
            snapshot(snap_dir / f"{mid}.jpg")
            results.append(meta)
            prev = direction
            time.sleep(0.3)
        # park
        pca.stop()
        time.sleep(1.0)

    stop_guard.set()
    pca.close()
    lgpio.gpiochip_close(h)
    (OUT / "results.json").write_text(json.dumps(results, indent=2))
    print(f"\nDone → {OUT}")
    print("Compare W-marker across strat*/02_wind_1, 04_wind_3, 06_wind_5")
    print("and strat*/01_unwind_1, 03_unwind_3, 05_unwind_5")
    print("Restart videorecorder extension when finished.")


if __name__ == "__main__":
    main()
