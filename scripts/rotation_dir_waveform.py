#!/usr/bin/env python3
"""Characterise the rotation-sensor digital waveform in BOTH directions.

Runs ON the Pi with the DropCam extension stopped (so it doesn't fight us for
GPIO10 / the PCA9685).  Claims GPIO10 as a plain input (no interrupt) and
samples the digital level at high rate while driving the winch UNWIND then
WIND at the calibrated PWMs.  Writes a CSV of (t_s, level) per direction and
prints, for each, the count and timing of rising vs falling transitions plus
how long each level dwells (fast "snap" vs slow "ramp").

Goal: see whether the once-per-rev fast snap is a FALLING edge in unwind but a
RISING edge in wind (which the falling-only detector would miss).
"""
from __future__ import annotations
import json, math, subprocess, time
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
WIND_US = 1450
SAMPLE_HZ = 2000
RUN_S = 6.0
OUT = Path(f"/home/pi/rot_wave_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}")


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


def ext_ids():
    try:
        out = subprocess.check_output(["docker", "ps", "--format", "{{.ID}} {{.Names}}"], text=True)
    except Exception:
        return []
    return [l.split(None, 1)[0] for l in out.splitlines()
            if "videorecorder" in l and "rot-" not in l]


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


def sample_run(h, pca, pwm, label):
    print(f"\n=== {label}: pwm={pwm} for {RUN_S}s @ {SAMPLE_HZ}Hz ===")
    period = 1.0 / SAMPLE_HZ
    samples = []  # (t, level)
    pca.set_pulse(pwm)
    t0 = time.monotonic()
    nxt = t0
    while time.monotonic() - t0 < RUN_S:
        nxt += period
        lv = lgpio.gpio_read(h, ROT_GPIO)
        samples.append((time.monotonic() - t0, lv))
        s = nxt - time.monotonic()
        if s > 0:
            time.sleep(s)
    pca.stop()
    # Analyse transitions with a small temporal-hysteresis (debounce ~4ms).
    confirm = max(1, int(0.004 * SAMPLE_HZ))
    lvl = samples[0][1]
    cand = None; run = 0
    trans = []  # (t, kind) kind = 'F' or 'R'
    for t, raw in samples:
        if raw == lvl:
            cand = None; run = 0
        else:
            if raw == cand: run += 1
            else: cand = raw; run = 1
            if run >= confirm:
                kind = 'F' if (lvl == 1 and raw == 0) else 'R'
                trans.append((round(t, 4), kind)); lvl = raw; cand = None; run = 0
    falls = [t for t, k in trans if k == 'F']
    rises = [t for t, k in trans if k == 'R']
    fi = [round(falls[i]-falls[i-1], 3) for i in range(1, len(falls))]
    ri = [round(rises[i]-rises[i-1], 3) for i in range(1, len(rises))]
    frac_high = sum(1 for _, lv in samples if lv == 1) / len(samples)
    print(f"  samples={len(samples)} frac_high={frac_high:.2f}")
    print(f"  FALLING edges={len(falls)} intervals_s={fi}")
    print(f"  RISING  edges={len(rises)} intervals_s={ri}")
    print(f"  transition sequence: {''.join(k for _,k in trans)}")
    (OUT / f"{label}.csv").write_text(
        "t_s,level\n" + "\n".join(f"{t:.4f},{lv}" for t, lv in samples)
    )
    return {"label": label, "pwm": pwm, "falling": len(falls), "rising": len(rises),
            "frac_high": round(frac_high, 3), "fall_iv": fi, "rise_iv": ri,
            "seq": "".join(k for _, k in trans)}


def main():
    OUT.mkdir(parents=True)
    print(f"OUT={OUT}")
    ensure_stopped(); time.sleep(1.0)
    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    try:
        h = lgpio.gpiochip_open(0)
        try:
            lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        except Exception:
            lgpio.gpiochip_close(h); h = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        lgpio.gpio_claim_input(h, ROT_GPIO, lgpio.SET_PULL_DOWN)
        pca = Pca(); pca.stop(); time.sleep(0.5)
        rows = []
        rows.append(sample_run(h, pca, UNWIND_US, "unwind_1550"))
        time.sleep(1.5)
        for us in (1420, 1415, 1410, 1405):
            rows.append(sample_run(h, pca, us, f"wind_{us}"))
            time.sleep(1.5)
        # RPM helper from falling-edge intervals.
        for r in rows:
            iv = r["fall_iv"]
            r["rpm"] = round(60.0/(sum(iv)/len(iv)), 1) if iv else 0.0
        print("\n=== RPM summary (target unwind RPM) ===")
        for r in rows:
            print(f"  {r['label']:12s} pwm={r['pwm']} falls={r['falling']} rpm={r['rpm']}")
        (OUT / "summary.json").write_text(json.dumps(rows, indent=2))
        pca.close()
        lgpio.gpiochip_close(h)
    finally:
        name = restart_ext()
        print(f"\nrestarted extension: {name}")
    print(f"Wrote {OUT}")


if __name__ == "__main__":
    main()
