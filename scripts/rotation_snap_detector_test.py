#!/usr/bin/env python3
"""Validate the direction-dependent snap detector + return-to-position stats.

Mirrors the NEW app/hardware.py logic (extension image not yet rebuilt):
count the FALLING snap when unwinding and the RISING snap when winding, keying
on the fast dead-zone snap that lands at the same mechanical angle both ways.
Canonical wind-finish is OFF.

A single 2 kHz sampler maintains independent falling/rising counters for BOTH a
500 Hz-decimated polled detector (what the firmware runs) and a 2 kHz reference
(ground truth).  For each leg we read the direction-appropriate polled counter.

Test: baseline of 3 cycles, each = unwind 25 rev then wind 25 rev.  After every
leg (and once at the very start) we grab an MCM still (camera tilt 1230 us) so
the wedge park-angle can be measured offline for:
  * direction consistency  (do unwind-parks and wind-parks coincide -> snaps
    aligned, ~½-turn offset gone?)
  * return-to-position      (end-of-cycle vs start, and park-angle spread)

Extension is stopped for the run (a guard thread keeps it down) and restarted
at the end.  MCM RTSP stays up regardless, so stills/recording work.
"""
from __future__ import annotations
import json, math, subprocess, threading, time
from datetime import datetime, timezone
from pathlib import Path
import lgpio
from smbus2 import SMBus

RELEASE_CH = 2
TILT_CH = 0
TILT_US = 1230
PCA9685_ADDR = 0x40
PCA9685_OE_GPIO = 4
ROT_GPIO = 10
STOP_US = 1500
UNWIND_US = 1550
WIND_US = 1415
POLL_HZ = 500
CONFIRM_SAMPLES = 4
MIN_INTER_S = 0.25
SAMPLE_HZ = 2000
DECIMATE = int(SAMPLE_HZ / POLL_HZ)
REF_CONFIRM = int(0.004 * SAMPLE_HZ)
REF_MIN_INTER_S = 0.10
STALL_THRESHOLD_S = 4.0        # abort if no NEW edge for this long
CAP_S = 45.0
REVS = 25
CYCLES = 3
IMG = "vshie/blueos-blueos_video_recorder:dropcam"
RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"
STAMP = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
OUT = Path(f"/home/pi/rot_snap_{STAMP}")
SNAPS = OUT / "snaps"


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
    def set_channel(self, ch, us):
        c = max(0, min(4095, int(round(max(0, min(20000, us)) / 20000 * 4096))))
        self.bus.write_i2c_block_data(PCA9685_ADDR, 0x06 + 4*ch, [0, 0, c & 0xFF, (c >> 8) & 0x1F])
    def set_pulse(self, us): self.set_channel(RELEASE_CH, us)
    def set_tilt(self, us): self.set_channel(TILT_CH, us)
    def stop(self): self.set_pulse(STOP_US)
    def close(self):
        try: self.stop()
        except Exception: pass
        self.bus.close()


class DualSampler(threading.Thread):
    def __init__(self, h):
        super().__init__(daemon=True)
        self.h = h; self.lock = threading.Lock()
        self.pf = 0; self.pr = 0; self.rf = 0; self.rr = 0
        self.pf_edges = []; self.pr_edges = []
        self._stopev = threading.Event()
    def stop(self): self._stopev.set(); self.join(1.0)
    def snap(self):
        with self.lock:
            return (self.pf, self.pr, self.rf, self.rr,
                    list(self.pf_edges), list(self.pr_edges))
    def run(self):
        period = 1.0 / SAMPLE_HZ; h = self.h
        lr = lgpio.gpio_read(h, ROT_GPIO); cr = None; rn = 0; lf_r = None; lr_r = None
        lp = lr; cp = None; rp = 0; lf_p = None; lr_p = None; deci = 0
        nxt = time.monotonic()
        while not self._stopev.is_set():
            nxt += period
            raw = lgpio.gpio_read(h, ROT_GPIO); t = time.monotonic()
            # reference (2 kHz)
            if raw == lr: cr = None; rn = 0
            else:
                if raw == cr: rn += 1
                else: cr = raw; rn = 1
                if rn >= REF_CONFIRM:
                    pv = lr; lr = raw; cr = None; rn = 0
                    if pv == 1 and lr == 0 and (lf_r is None or t-lf_r >= REF_MIN_INTER_S):
                        lf_r = t
                        with self.lock: self.rf += 1
                    elif pv == 0 and lr == 1 and (lr_r is None or t-lr_r >= REF_MIN_INTER_S):
                        lr_r = t
                        with self.lock: self.rr += 1
            # polled (decimated ~500 Hz)
            deci += 1
            if deci >= DECIMATE:
                deci = 0
                if raw == lp: cp = None; rp = 0
                else:
                    if raw == cp: rp += 1
                    else: cp = raw; rp = 1
                    if rp >= CONFIRM_SAMPLES:
                        pv = lp; lp = raw; cp = None; rp = 0
                        if pv == 1 and lp == 0 and (lf_p is None or t-lf_p >= MIN_INTER_S):
                            lf_p = t
                            with self.lock: self.pf += 1; self.pf_edges.append(t)
                        elif pv == 0 and lp == 1 and (lr_p is None or t-lr_p >= MIN_INTER_S):
                            lr_p = t
                            with self.lock: self.pr += 1; self.pr_edges.append(t)
            s = nxt - time.monotonic()
            if s > 0: self._stopev.wait(s)
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


def grab(name):
    SNAPS.mkdir(parents=True, exist_ok=True)
    subprocess.run(["docker", "rm", "-f", "rot-snap"], capture_output=True)
    subprocess.run(
        ["docker", "run", "--rm", "--network", "host", "--name", "rot-snap",
         "--entrypoint", "ffmpeg", "-v", f"{SNAPS}:/out", IMG,
         "-y", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", RTSP,
         "-frames:v", "1", "-q:v", "2", f"/out/{name}"],
        capture_output=True,
    )
    p = SNAPS / name
    return p.exists() and p.stat().st_size > 3000


def leg(s, pca, direction, target, label):
    """direction +1 unwind (falling snap), -1 wind (rising snap)."""
    pwm = UNWIND_US if direction > 0 else WIND_US
    pf0, pr0, rf0, rr0, _, _ = s.snap()
    base_poll = pf0 if direction > 0 else pr0
    base_ref = rf0 if direction > 0 else rr0
    t0 = time.monotonic(); last_edge_t = t0
    pca.set_pulse(pwm)
    outcome = "running"; delivered = 0
    while True:
        pf, pr, rf, rr, pfe, pre = s.snap()
        cur = (pf if direction > 0 else pr) - base_poll
        now = time.monotonic()
        if cur > delivered:
            delivered = cur; last_edge_t = now
        if delivered >= target: outcome = "target_reached"; break
        if (now - last_edge_t) >= STALL_THRESHOLD_S: outcome = "stalled"; break
        if (now - t0) >= CAP_S: outcome = "timed_out"; break
        time.sleep(0.005)
    pca.stop(); time.sleep(0.6)
    pf, pr, rf, rr, pfe, pre = s.snap()
    dp = (pf if direction > 0 else pr) - base_poll
    dr = (rf if direction > 0 else rr) - base_ref
    edges = (pfe if direction > 0 else pre)
    leg_edges = edges[-dp:] if dp >= 2 else []
    if len(leg_edges) >= 2:
        iv = [leg_edges[i]-leg_edges[i-1] for i in range(1, len(leg_edges))]
        rpm = 60.0/(sum(iv)/len(iv))
    else:
        rpm = 0.0
    el = time.monotonic() - t0
    dname = "unwind" if direction > 0 else "wind"
    print(f"  {label:12s} {dname:6s} pwm={pwm} target={target} "
          f"polled={dp:2d} ref={dr:2d} rpm={rpm:5.1f} t={el:5.1f}s {outcome}"
          f"{'  MISMATCH' if dp != dr else ''}")
    return {"label": label, "dir": dname, "pwm": pwm, "target": target,
            "polled": dp, "ref": dr, "rpm": round(rpm, 1),
            "elapsed_s": round(el, 1), "outcome": outcome, "match": dp == dr}


def main():
    OUT.mkdir(parents=True); SNAPS.mkdir(parents=True, exist_ok=True)
    print(f"OUT={OUT}")
    ensure_stopped(); time.sleep(1.0)
    guard_stop = threading.Event()
    def guard():
        while not guard_stop.is_set():
            for i in ext_ids():
                subprocess.run(["docker", "stop", "-t", "0", i], capture_output=True)
            guard_stop.wait(1.5)
    threading.Thread(target=guard, daemon=True).start()
    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    rows = []; grabs = []
    try:
        h = lgpio.gpiochip_open(0)
        try: lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        except Exception:
            lgpio.gpiochip_close(h); h = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        lgpio.gpio_claim_input(h, ROT_GPIO, lgpio.SET_PULL_DOWN)
        pca = Pca(); pca.stop(); pca.set_tilt(TILT_US); print(f"tilt -> {TILT_US} us")
        time.sleep(1.0)
        s = DualSampler(h); s.start(); time.sleep(0.3)

        ok = grab("00_start.jpg"); grabs.append(("00_start.jpg", ok))
        print(f"start still: {ok}")
        for c in range(1, CYCLES+1):
            print(f"\n=== CYCLE {c}: unwind {REVS}, wind {REVS} ===")
            r = leg(s, pca, +1, REVS, f"c{c}_unwind"); rows.append(r)
            g = f"c{c}_1_after_unwind.jpg"; grabs.append((g, grab(g)))
            time.sleep(0.8)
            r = leg(s, pca, -1, REVS, f"c{c}_wind"); rows.append(r)
            g = f"c{c}_2_after_wind.jpg"; grabs.append((g, grab(g)))
            time.sleep(0.8)

        s.stop(); pca.close(); lgpio.gpiochip_close(h)
    finally:
        guard_stop.set()
        name = restart_ext(); print(f"restarted extension: {name}")

    all_match = all(r["match"] for r in rows)
    all_full = all(r["outcome"] == "target_reached" and r["polled"] == r["target"] for r in rows)
    summary = {"revs": REVS, "cycles": CYCLES, "all_counts_match": all_match,
               "all_legs_full": all_full, "rows": rows, "grabs": grabs}
    (OUT / "results.json").write_text(json.dumps(summary, indent=2))
    print(f"\nALL polled==reference: {all_match}")
    print(f"ALL legs delivered target: {all_full}")
    print(f"snaps dir: {SNAPS}")
    print(f"Wrote {OUT/'results.json'}")


if __name__ == "__main__":
    main()
