#!/usr/bin/env python3
"""Speed + counting-accuracy matrix for the rotation sensor (extension stopped).

For each PWM/direction we drive the winch closed-loop (stop after N edges) while
a single 2 kHz sampler thread maintains TWO independent falling-edge counters on
GPIO10:

  * REFERENCE  — 2 kHz stream, 4 ms debounce (ground truth for "true" edges).
  * POLLED     — the 2 kHz stream decimated to 500 Hz and run through the EXACT
                 detector logic from app/hardware.py (CONFIRM_SAMPLES=4 -> 8 ms
                 temporal hysteresis, MIN_INTER_S=0.25 refractory).

The closed loop stops on the POLLED count (mirrors the extension).  Comparing
POLLED vs REFERENCE over the same window catches under/over-counting at speed.
RPM is derived from POLLED edge intervals.

Tests:
  A. 10-rev closed loop each way (unwind 1550, wind 1415).
  B. Speed matrix, paired unwind/wind (net travel ~0), 5 edges/leg, covering
     jog speeds up through the recipe release (2000 µs) / recipe wind (1000 µs).

Legs alternate direction so cumulative line travel stays bounded; a final
rebalance leg corrects any residual net.
"""
from __future__ import annotations
import json, math, subprocess, threading, time
from datetime import datetime, timezone
from pathlib import Path
import lgpio
from smbus2 import SMBus

RELEASE_CH = 2
TILT_CH = 0                 # PCA9685 ch0 = camera tilt servo (app/hardware.py)
TILT_US = 1230             # operator-requested tilt to frame the W-disc
PCA9685_ADDR = 0x40
PCA9685_OE_GPIO = 4
ROT_GPIO = 10
STOP_US = 1500
IMG = "vshie/blueos-blueos_video_recorder:dropcam"
RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"

# Mirrors app/hardware.py
UNWIND_US = 1550
WIND_US = 1415
POLL_HZ = 500
CONFIRM_SAMPLES = 4
MIN_INTER_S = 0.25
STALL_THRESHOLD_S = 4.0

SAMPLE_HZ = 2000
DECIMATE = int(SAMPLE_HZ / POLL_HZ)          # 4  -> 500 Hz polled emulation
REF_CONFIRM = int(0.004 * SAMPLE_HZ)         # 8  -> 4 ms reference debounce
REF_MIN_INTER_S = 0.10
CAP_S = 12.0
OUT = Path(f"/home/pi/rot_matrix_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}")


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
    """2 kHz sampler maintaining reference + decimated-polled falling counters."""
    def __init__(self, h):
        super().__init__(daemon=True)
        self.h = h
        self.lock = threading.Lock()
        self.ref_count = 0
        self.poll_count = 0
        self.poll_edges = []   # monotonic times (polled)
        self.ref_edges = []
        self._stopev = threading.Event()

    def stop(self): self._stopev.set(); self.join(1.0)

    def snap(self):
        with self.lock:
            return self.ref_count, self.poll_count, list(self.poll_edges)

    def run(self):
        period = 1.0 / SAMPLE_HZ
        h = self.h
        lvl_r = lgpio.gpio_read(h, ROT_GPIO); cand_r = None; run_r = 0; last_r = None
        lvl_p = lvl_r; cand_p = None; run_p = 0; last_p = None; deci = 0
        nxt = time.monotonic()
        while not self._stopev.is_set():
            nxt += period
            raw = lgpio.gpio_read(h, ROT_GPIO)
            t = time.monotonic()
            # reference (2 kHz)
            if raw == lvl_r:
                cand_r = None; run_r = 0
            else:
                if raw == cand_r: run_r += 1
                else: cand_r = raw; run_r = 1
                if run_r >= REF_CONFIRM:
                    prev = lvl_r; lvl_r = raw; cand_r = None; run_r = 0
                    if prev == 1 and lvl_r == 0 and (last_r is None or t - last_r >= REF_MIN_INTER_S):
                        last_r = t
                        with self.lock:
                            self.ref_count += 1; self.ref_edges.append(t)
            # polled (decimated to ~500 Hz)
            deci += 1
            if deci >= DECIMATE:
                deci = 0
                if raw == lvl_p:
                    cand_p = None; run_p = 0
                else:
                    if raw == cand_p: run_p += 1
                    else: cand_p = raw; run_p = 1
                    if run_p >= CONFIRM_SAMPLES:
                        prev = lvl_p; lvl_p = raw; cand_p = None; run_p = 0
                        if prev == 1 and lvl_p == 0 and (last_p is None or t - last_p >= MIN_INTER_S):
                            last_p = t
                            with self.lock:
                                self.poll_count += 1; self.poll_edges.append(t)
            s = nxt - time.monotonic()
            if s > 0:
                self._stopev.wait(s)
            else:
                nxt = time.monotonic()


REC_T0 = None   # monotonic at recording start, for leg offset printing


def record_start(dur_s, path):
    subprocess.run(["docker", "rm", "-f", "rot-matrix-rec"], capture_output=True)
    subprocess.run(
        ["docker", "run", "-d", "--name", "rot-matrix-rec", "--network", "host",
         "--entrypoint", "ffmpeg", "-v", "/home/pi:/out", IMG,
         "-y", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", RTSP,
         "-t", str(int(dur_s)), "-c", "copy", "-f", "matroska", f"/out/{path}"],
        capture_output=True,
    )


def record_stop():
    subprocess.run(["docker", "stop", "-t", "5", "rot-matrix-rec"], capture_output=True)
    subprocess.run(["docker", "rm", "-f", "rot-matrix-rec"], capture_output=True)


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


def leg(sampler, pca, pwm, direction, target, label):
    """Closed-loop: stop after `target` POLLED edges. direction: +1 unwind, -1 wind."""
    r0, p0, _ = sampler.snap()
    t0 = time.monotonic()
    pca.set_pulse(pwm)
    outcome = "running"
    edges_at_start = None
    while True:
        r, p, pe = sampler.snap()
        dp = p - p0
        now = time.monotonic()
        if edges_at_start is None and dp >= 1:
            edges_at_start = len(pe) - 1  # index of first edge in this leg
        if dp >= target:
            outcome = "target_reached"; break
        if dp == 0 and (now - t0) >= STALL_THRESHOLD_S:
            outcome = "sensor_stalled"; break
        if now - t0 >= CAP_S:
            outcome = "timed_out"; break
        time.sleep(0.005)
    pca.stop(); time.sleep(0.45)
    r, p, pe = sampler.snap()
    dp = p - p0; dr = r - r0
    el = time.monotonic() - t0
    # RPM from polled edge intervals within this leg (last `dp` edges).
    leg_edges = pe[-dp:] if dp >= 2 else []
    if len(leg_edges) >= 2:
        iv = [leg_edges[i]-leg_edges[i-1] for i in range(1, len(leg_edges))]
        rpm = 60.0 / (sum(iv)/len(iv))
    else:
        rpm = 0.0
    dname = "unwind" if direction > 0 else "wind"
    voff = round(t0 - REC_T0, 2) if REC_T0 is not None else None
    print(f"  {label:16s} {dname:6s} pwm={pwm} target={target:2d} "
          f"polled={dp:2d} ref={dr:2d} rpm={rpm:5.1f} t={el:5.2f}s "
          f"vid@{voff}s {outcome}{'  MISMATCH' if dp != dr else ''}")
    return {"label": label, "dir": dname, "pwm": pwm, "target": target,
            "polled": dp, "ref": dr, "rpm": round(rpm, 1),
            "elapsed_s": round(el, 2), "video_offset_s": voff,
            "outcome": outcome, "match": dp == dr}


def main():
    global REC_T0
    OUT.mkdir(parents=True)
    print(f"OUT={OUT}")
    ensure_stopped(); time.sleep(1.0)
    subprocess.run(["raspi-gpio", "set", "2", "a0"], check=False)
    rows = []
    net = 0
    rec_name = "matrix_full.mkv"
    try:
        h = lgpio.gpiochip_open(0)
        try: lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        except Exception:
            lgpio.gpiochip_close(h); h = lgpio.gpiochip_open(4)
            lgpio.gpio_claim_output(h, PCA9685_OE_GPIO, 0)
        lgpio.gpio_claim_input(h, ROT_GPIO, lgpio.SET_PULL_DOWN)
        pca = Pca(); pca.stop()
        pca.set_tilt(TILT_US)                 # frame the W-disc for the camera
        print(f"tilt servo -> {TILT_US} us")
        time.sleep(1.0)
        s = DualSampler(h); s.start(); time.sleep(0.3)

        # Start one continuous recording covering the whole run (MCM RTSP is up
        # even with the extension stopped).  Leg rows carry video_offset_s.
        record_start(170, rec_name)
        time.sleep(3.0)                       # ffmpeg RTSP connect warm-up
        REC_T0 = time.monotonic()
        print(f"recording -> /home/pi/{rec_name} (offsets relative to now)")

        print("\n=== PART A: 10-rev closed loop each direction (wind-in first) ===")
        r = leg(s, pca, WIND_US, -1, 10, "A_wind10");     rows.append(r); net -= r["polled"]; time.sleep(1.5)
        r = leg(s, pca, UNWIND_US, +1, 10, "A_unwind10"); rows.append(r); net += r["polled"]; time.sleep(1.5)

        print(f"\n=== PART B: speed matrix (paired, net~0), 5 edges/leg ===  net so far={net}")
        pairs = [
            (UNWIND_US, 1415),   # ~55 RPM matched
            (1600, 1350),
            (1700, 1250),
            (1850, 1100),
            (2000, 1000),        # recipe release / recipe wind extremes
        ]
        for uu, ww in pairs:
            r = leg(s, pca, uu, +1, 5, f"B_unwind_{uu}"); rows.append(r); net += r["polled"]; time.sleep(0.8)
            r = leg(s, pca, ww, -1, 5, f"B_wind_{ww}");   rows.append(r); net -= r["polled"]; time.sleep(0.8)

        # Rebalance net line travel back toward zero.
        if abs(net) >= 2:
            print(f"\n=== REBALANCE net={net} ===")
            if net > 0:
                r = leg(s, pca, WIND_US, -1, abs(net), "rebalance_wind"); net -= r["polled"]
            else:
                r = leg(s, pca, UNWIND_US, +1, abs(net), "rebalance_unwind"); net += r["polled"]
            rows.append(r)
        print(f"\nfinal net (unwind-positive) ≈ {net} rev")

        s.stop(); pca.close(); lgpio.gpiochip_close(h)
    finally:
        record_stop()
        name = restart_ext(); print(f"restarted extension: {name}")
        print(f"video: /home/pi/{rec_name}")

    partA = [r for r in rows if r["label"].startswith("A_")]
    a_ok = all(r["outcome"] == "target_reached" and r["polled"] == r["target"] and r["match"] for r in partA)
    all_match = all(r["match"] for r in rows)
    summary = {"partA_pass": a_ok, "all_counts_match": all_match,
               "final_net_rev": net, "rows": rows}
    (OUT / "results.json").write_text(json.dumps(summary, indent=2))
    print(f"\nPART A pass (10/10 both ways, counts match): {a_ok}")
    print(f"ALL legs polled==reference: {all_match}")
    print(f"Wrote {OUT/'results.json'}")


if __name__ == "__main__":
    main()
