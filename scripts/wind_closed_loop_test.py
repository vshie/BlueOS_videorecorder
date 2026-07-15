#!/usr/bin/env python3
"""Validate the wind-direction fix end-to-end (extension stopped).

Mirrors hardware.release_run_for_rotations' closed loop: a polled falling-edge
detector (POLL_HZ/CONFIRM/MIN_INTER identical to hardware.py) + the "run until
N edges or stall_threshold with 0 edges" logic, driving WIND at the new
1415 µs.  Runs several back-to-back "wind 3" legs from whatever pose the prior
leg left, so we exercise the worst case (a leg starting just after an edge).

Pass criteria: every leg delivers 3/3 with outcome target_reached, and none
aborts as sensor_stalled.
"""
from __future__ import annotations
import json, math, subprocess, threading, time
from datetime import datetime, timezone
from pathlib import Path
import lgpio
from smbus2 import SMBus

RELEASE_CH = 2
PCA9685_ADDR = 0x40
PCA9685_OE_GPIO = 4
ROT_GPIO = 10
STOP_US = 1500
UNWIND_US = 1550
WIND_US = 1415          # NEW calibrated value
POLL_HZ = 500
CONFIRM_SAMPLES = 4
MIN_INTER_S = 0.25
STALL_THRESHOLD_S = 4.0  # NEW value
CAP_S = 8.0
OUT = Path(f"/home/pi/rot_windcl_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}")


class Pca:
    def __init__(self):
        self.bus = SMBus(1)
        self._w(0xFD, 0x10); m = self._r(0x00)
        self._w(0x00, (m & ~0x80) | 0x10); time.sleep(0.001)
        self._w(0x00, 0x10 | 0x40 | 0x20)
        ps = max(3, min(255, int(math.ceil(24_576_000 / (4096 * 50)) - 1)))
        self._w(0xFE, ps); self._w(0x01, 0x04)
        self._w(0x00, 0x40 | 0x20 | 0x80); time.sleep(0.001)
    def _w(self, r, v): self.bus.write_byte_data(PCA9685_ADDR, r, v & 0xFF)
    def _r(self, r): return self.bus.read_byte_data(PCA9685_ADDR, r)
    def set_pulse(self, us):
        c = max(0, min(4095, int(round(max(0, min(20000, us)) / 20000 * 4096))))
        self.bus.write_i2c_block_data(PCA9685_ADDR, 0x06 + 4*RELEASE_CH, [0, 0, c & 0xFF, (c >> 8) & 0x1F])
    def stop(self): self.set_pulse(STOP_US)
    def close(self):
        try: self.stop()
        except Exception: pass
        self.bus.close()


class PollDetector:
    def __init__(self, h):
        self.h = h; self.lock = threading.Lock()
        self.count = 0; self._last = None
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)
    def start(self): self._t.start()
    def stop(self): self._stop.set(); self._t.join(1.0)
    def reset(self):
        with self.lock: self.count = 0; self._last = None
    def snapshot(self):
        with self.lock: return self.count
    def _reg(self, t):
        with self.lock:
            if self._last is not None and (t - self._last) < MIN_INTER_S: return
            self._last = t; self.count += 1
    def _loop(self):
        period = 1.0/POLL_HZ; confirm = CONFIRM_SAMPLES
        level = lgpio.gpio_read(self.h, ROT_GPIO); cand = None; run = 0
        nxt = time.monotonic()
        while not self._stop.is_set():
            nxt += period
            raw = lgpio.gpio_read(self.h, ROT_GPIO)
            if raw == level: cand = None; run = 0
            else:
                if raw == cand: run += 1
                else: cand = raw; run = 1
                if run >= confirm:
                    prev = level; level = raw; cand = None; run = 0
                    if prev == 1 and level == 0: self._reg(time.monotonic())
            s = nxt - time.monotonic()
            if s > 0:
                if self._stop.wait(s): break
            else: nxt = time.monotonic()


def ext_ids():
    out = subprocess.check_output(["docker", "ps", "--format", "{{.ID}} {{.Names}}"], text=True)
    return [l.split(None, 1)[0] for l in out.splitlines() if "videorecorder" in l and "rot-" not in l]

def ensure_stopped():
    for i in ext_ids():
        subprocess.run(["docker", "update", "--restart=no", i], capture_output=True)
        subprocess.run(["docker", "stop", "-t", "1", i], capture_output=True)

def restart_ext():
    for line in subprocess.check_output(["docker", "ps", "-a", "--format", "{{.ID}} {{.Names}}"], text=True).splitlines():
        p = line.split(None, 1)
        if len(p) == 2 and "videorecorder" in p[1] and "rot-" not in p[1]:
            subprocess.run(["docker", "update", "--restart=unless-stopped", p[0]], capture_output=True)
            subprocess.run(["docker", "start", p[0]], capture_output=True)
            return p[1]
    return None


def leg(det, pca, pwm, target, label):
    det.reset(); t0 = time.monotonic(); pca.set_pulse(pwm)
    outcome = "running"; delivered = 0
    while True:
        delivered = det.snapshot()
        now = time.monotonic()
        if delivered >= target: outcome = "target_reached"; break
        if delivered == 0 and (now - t0) >= STALL_THRESHOLD_S: outcome = "sensor_stalled"; break
        if now - t0 >= CAP_S: outcome = "timed_out"; break
        time.sleep(0.01)
    pca.stop(); time.sleep(0.4)
    delivered = det.snapshot()
    el = time.monotonic() - t0
    print(f"  {label}: delivered={delivered}/{target} outcome={outcome} elapsed={el:.2f}s")
    return {"label": label, "pwm": pwm, "target": target, "delivered": delivered,
            "outcome": outcome, "elapsed_s": round(el, 2)}


def main():
    OUT.mkdir(parents=True)
    ensure_stopped(); time.sleep(1.0)
    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    rows = []
    try:
        h = lgpio.gpiochip_open(0)
        try: lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        except Exception:
            lgpio.gpiochip_close(h); h = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        lgpio.gpio_claim_input(h, ROT_GPIO, lgpio.SET_PULL_DOWN)
        pca = Pca(); pca.stop(); time.sleep(0.5)
        det = PollDetector(h); det.start()
        print(f"=== WIND closed-loop x5 (pwm={WIND_US}, target 3, stall={STALL_THRESHOLD_S}s) ===")
        for k in range(1, 6):
            rows.append(leg(det, pca, WIND_US, 3, f"wind3_run{k}"))
            time.sleep(1.0)
        print(f"=== UNWIND closed-loop x2 (pwm={UNWIND_US}, target 3) — regression check ===")
        for k in range(1, 3):
            rows.append(leg(det, pca, UNWIND_US, 3, f"unwind3_run{k}"))
            time.sleep(1.0)
        det.stop(); pca.close(); lgpio.gpiochip_close(h)
    finally:
        name = restart_ext(); print(f"restarted extension: {name}")
    ok = all(r["outcome"] == "target_reached" and r["delivered"] == r["target"] for r in rows)
    (OUT / "results.json").write_text(json.dumps({"pass": ok, "rows": rows}, indent=2))
    print(f"\nPASS={ok}")


if __name__ == "__main__":
    main()
