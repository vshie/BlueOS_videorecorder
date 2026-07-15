#!/usr/bin/env python3
"""Validate the polled rotation sensor: prove no IRQ storm + counting.

Runs ON the Pi.  Uses the *real* backend path
(``gpio_backend.GpioLine.claim_input`` / ``read``) plus the exact
temporal-hysteresis edge detector from ``hardware.py``, while driving the
winch through the PCA9685.  For every phase it samples the kernel interrupt
count for GPIO10 (``/proc/interrupts`` line 57, ``pinctrl-bcm2835 10``) so
we can *prove* that polling arms no interrupt and therefore cannot produce
the ``irq/NN-lg`` storm that was wedging the Pi.

Requires (on the Pi):  lgpio, smbus2, and app/gpio_backend.py copied next to
this script (deploy step handles that).  The DropCam extension must be
stopped so it does not fight us for GPIO10 / the PCA9685.

Phases:
  0. STATIONARY (motor stopped, 8 s): the case that used to storm.  Counts
     must stay ~0 and IRQ57 must not grow.
  1. UNWIND 6 s and WIND 6 s continuous: report counted edges + intervals.
  2. UNWIND-x3 reproduction: three back-to-back "run until 3 edges" legs
     (cap 8 s each) with before/after MCM snaps, to check consistency.

Every phase prints the IRQ57 delta; on the polled path it should be 0.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/pi/rotpoll")

import lgpio  # noqa: E402
from smbus2 import SMBus  # noqa: E402

from gpio_backend import GpioLine  # noqa: E402

# ── Config (mirrors app/hardware.py) ────────────────────────────────────
ROT_GPIO = 10
RELEASE_CH = 2
PCA9685_ADDR = 0x40
PCA9685_OE_GPIO = 4
STOP_US = 1500
UNWIND_US = 1550
WIND_US = 1450
POLL_HZ = 500
CONFIRM_SAMPLES = 4
MIN_INTER_S = 0.25
PULL = "down"
RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"
EXT_IMAGE = "vshie/blueos-blueos_video_recorder:dropcam"

OUT = Path(
    f"/home/pi/rot_poll_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}"
)


# ── PCA9685 (same init as scripts/raw_rotation_edges.py) ────────────────
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


# ── Polled edge detector (identical logic to hardware._rotation_poll_loop) ─
class PollDetector:
    """Samples GPIO10 via GpioLine.read with temporal-hysteresis debounce."""

    def __init__(self, line: GpioLine):
        self.line = line
        self.lock = threading.Lock()
        self.count = 0
        self.edges = []          # monotonic seconds of accepted edges
        self._last_acc = None
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)

    def reset(self):
        with self.lock:
            self.count = 0
            self.edges = []
            self._last_acc = None

    def snapshot(self):
        with self.lock:
            return self.count, list(self.edges)

    def _register(self, tick_s):
        with self.lock:
            if self._last_acc is not None and (tick_s - self._last_acc) < MIN_INTER_S:
                return
            self._last_acc = tick_s
            self.count += 1
            self.edges.append(tick_s)

    def _loop(self):
        period = 1.0 / float(POLL_HZ)
        confirm = int(CONFIRM_SAMPLES)
        try:
            level = self.line.read(ROT_GPIO)
        except Exception:
            level = 0
        candidate = None
        run = 0
        next_t = time.monotonic()
        while not self._stop.is_set():
            next_t += period
            try:
                raw = self.line.read(ROT_GPIO)
            except Exception:
                if self._stop.wait(period):
                    break
                continue
            if raw == level:
                candidate = None
                run = 0
            else:
                if raw == candidate:
                    run += 1
                else:
                    candidate = raw
                    run = 1
                if run >= confirm:
                    prev = level
                    level = raw
                    candidate = None
                    run = 0
                    if prev == 1 and level == 0:
                        self._register(time.monotonic())
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                if self._stop.wait(sleep_s):
                    break
            else:
                next_t = time.monotonic()


# ── Helpers ─────────────────────────────────────────────────────────────
def irq57() -> int:
    """Sum of all per-CPU counts for IRQ 57 (pinctrl GPIO10), or -1."""
    try:
        with open("/proc/interrupts") as f:
            for line in f:
                if re.match(r"\s*57:", line):
                    parts = line.split()
                    total = 0
                    for tok in parts[1:]:
                        if tok.isdigit():
                            total += int(tok)
                        else:
                            break
                    return total
    except Exception:
        pass
    return -1


def ext_ids():
    try:
        out = subprocess.check_output(
            ["docker", "ps", "--format", "{{.ID}} {{.Names}}"], text=True
        )
    except Exception:
        return []
    ids = []
    for line in out.splitlines():
        p = line.split(None, 1)
        if len(p) == 2 and "videorecorder" in p[1] and "rot-" not in p[1]:
            ids.append(p[0])
    return ids


def ensure_stopped():
    for i in ext_ids():
        subprocess.run(["docker", "update", "--restart=no", i], capture_output=True)
        subprocess.run(["docker", "stop", "-t", "1", i], capture_output=True)


def snap(path: Path) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(["docker", "rm", "-f", "rot-poll-ffmpeg"], capture_output=True)
    subprocess.run(
        [
            "docker", "run", "--rm", "--network", "host",
            "--name", "rot-poll-ffmpeg", "--entrypoint", "ffmpeg",
            "-v", f"{path.parent}:/out", EXT_IMAGE,
            "-y", "-loglevel", "error", "-rtsp_transport", "tcp",
            "-i", RTSP, "-frames:v", "1", "-q:v", "2", f"/out/{path.name}",
        ],
        capture_output=True,
    )
    return path.exists() and path.stat().st_size > 1000


def intervals(edges):
    return [round(edges[i] - edges[i - 1], 3) for i in range(1, len(edges))]


# ── Phases ──────────────────────────────────────────────────────────────
def phase_stationary(det, seconds=8.0):
    print(f"\n=== STATIONARY (motor stopped) {seconds:.0f}s ===")
    i0 = irq57()
    det.reset()
    time.sleep(seconds)
    n, edges = det.snapshot()
    i1 = irq57()
    print(f"  counted={n} (expect 0)  IRQ57 delta={i1 - i0} (expect 0)")
    return {"phase": "stationary", "counted": n, "irq57_delta": i1 - i0}


def phase_continuous(det, pca, pwm, label, seconds=6.0):
    print(f"\n=== {label}: pwm={pwm} for {seconds:.0f}s ===")
    i0 = irq57()
    det.reset()
    t0 = time.monotonic()
    pca.set_pulse(pwm)
    time.sleep(seconds)
    pca.stop()
    time.sleep(0.35)
    elapsed = time.monotonic() - t0
    n, edges = det.snapshot()
    i1 = irq57()
    iv = intervals(edges)
    rpm = (60.0 / (sum(iv) / len(iv))) if iv else 0.0
    print(
        f"  counted={n} elapsed={elapsed:.2f}s ~{rpm:.0f} RPM  "
        f"IRQ57 delta={i1 - i0} (expect 0)"
    )
    print(f"  intervals_s={iv}")
    return {
        "phase": label, "pwm": pwm, "counted": n, "elapsed_s": round(elapsed, 2),
        "rpm": round(rpm, 1), "irq57_delta": i1 - i0, "intervals_s": iv,
    }


def phase_run_until(det, pca, pwm, target, label, cap_s=8.0):
    """Drive until ``target`` edges are counted or cap; mirror release-by-rotations."""
    print(f"\n=== {label}: unwind until {target} edges (cap {cap_s:.0f}s) ===")
    snap(OUT / "snaps" / f"{label}_before.jpg")
    i0 = irq57()
    det.reset()
    t0 = time.monotonic()
    pca.set_pulse(pwm)
    delivered = 0
    while time.monotonic() - t0 < cap_s:
        delivered, _ = det.snapshot()
        if delivered >= target:
            break
        time.sleep(0.01)
    pca.stop()
    time.sleep(0.4)
    elapsed = time.monotonic() - t0
    delivered, edges = det.snapshot()
    i1 = irq57()
    snap(OUT / "snaps" / f"{label}_after.jpg")
    outcome = "target_reached" if delivered >= target else "timed_out"
    print(
        f"  delivered={delivered}/{target} outcome={outcome} "
        f"elapsed={elapsed:.2f}s  IRQ57 delta={i1 - i0}"
    )
    print(f"  intervals_s={intervals(edges)}")
    return {
        "phase": label, "pwm": pwm, "target": target, "delivered": delivered,
        "outcome": outcome, "elapsed_s": round(elapsed, 2),
        "irq57_delta": i1 - i0, "intervals_s": intervals(edges),
    }


def main():
    OUT.mkdir(parents=True)
    (OUT / "snaps").mkdir()
    print(f"OUT={OUT}")
    ensure_stopped()
    time.sleep(0.5)

    # Keep the extension down for the whole run (belt-and-suspenders; the
    # operator already disabled it, but a leftover restart policy could bite).
    guard_stop = threading.Event()

    def guard():
        while not guard_stop.is_set():
            for i in ext_ids():
                subprocess.run(["docker", "stop", "-t", "0", i], capture_output=True)
            guard_stop.wait(0.5)

    threading.Thread(target=guard, daemon=True).start()

    # Bring up OE + release channel routing (same as raw capture script).
    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)

    line = GpioLine()
    line.claim_output(PCA9685_OE_GPIO, 0)   # enable PCA9685 outputs (active-low)
    line.claim_input(ROT_GPIO, pull=PULL)   # NO alert -> NO interrupt

    print(
        f"GPIO10 claimed as INPUT (pull={PULL}); "
        f"initial level={line.read(ROT_GPIO)}  IRQ57={irq57()}"
    )

    pca = Pca()
    pca.stop()

    det = PollDetector(line)
    det.start()

    results = []
    try:
        results.append(phase_stationary(det, 8.0))
        time.sleep(0.5)
        results.append(phase_continuous(det, pca, UNWIND_US, "unwind_6s", 6.0))
        time.sleep(1.0)
        results.append(phase_continuous(det, pca, WIND_US, "wind_6s", 6.0))
        time.sleep(1.0)
        for k in (1, 2, 3):
            results.append(
                phase_run_until(det, pca, UNWIND_US, 3, f"unwind_x3_run{k}", 8.0)
            )
            time.sleep(1.0)
    finally:
        pca.stop()
        det.stop()
        pca.close()
        i_final = irq57()
        line.release(ROT_GPIO)
        line.release(PCA9685_OE_GPIO)
        line.close()
        guard_stop.set()

    summary = {
        "poll_hz": POLL_HZ, "confirm_samples": CONFIRM_SAMPLES,
        "min_inter_s": MIN_INTER_S, "pull": PULL,
        "irq57_final": i_final, "results": results,
    }
    (OUT / "results.json").write_text(json.dumps(summary, indent=2))
    print(f"\nWrote {OUT / 'results.json'}")
    print(f"FINAL IRQ57={i_final}")


if __name__ == "__main__":
    main()
