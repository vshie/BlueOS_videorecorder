#!/usr/bin/env python3
"""Verify the direction-reversal phantom-edge fix: 3 commanded -> 3 physical.

Mirrors the NEW app/hardware.py logic INCLUDING the reversal-skip:
  * count the FALLING snap when unwinding, the RISING snap when winding, and
  * on the FIRST move after a direction change (and the first move of the run),
    drop exactly one snap -- the phantom the shaft makes when it re-crosses the
    mark it was parked on.

Ground truth without the camera's help: a 2 kHz reference sampler timestamps
EVERY mark passage.  The phantom is unmistakable -- it lands ~0.1 s after the
shaft starts moving, whereas a real revolution takes ~1.1 s.  So for each leg:

    physical_revs = (reference snaps this leg) - (1 if first snap is a phantom)

and we assert physical_revs == 3 for every leg, back-to-back and on reversals.

The camera adds an independent visual: the whole run is recorded to an MP4 (MCM
RTSP stays up even with the extension stopped) and a still is grabbed before and
after every leg so park angles can be eyeballed / measured offline.

Scenario (net travel kept small; wind first to take up slack):
    A) wind 3, wind 3, wind 3            (same dir back-to-back)
    B) unwind 3, unwind 3, unwind 3      (same dir back-to-back)
    C) wind 3, unwind 3, wind 3, unwind 3 (every leg a reversal)
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
STALL_THRESHOLD_S = 4.0
CAP_S = 20.0
TARGET = 3
PHANTOM_GUARD_S = 0.4          # mirrors ROTATION_REVERSAL_GUARD_S in hardware.py
IMG = "vshie/blueos-blueos_video_recorder:dropcam"
RTSP = "rtsp://127.0.0.1:8554/video_2"
STAMP = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
OUT = Path(f"/home/pi/rot_revfix_{STAMP}")
SNAPS = OUT / "snaps"
VIDEO = OUT / "run.mp4"


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
    """2 kHz reference + 500 Hz-decimated polled snap counters, per polarity.

    Records the monotonic time of every reference snap (falling in rf_t, rising
    in rr_t) so per-leg latencies can be reconstructed as ground truth.
    """
    def __init__(self, h):
        super().__init__(daemon=True)
        self.h = h; self.lock = threading.Lock()
        self.pf = 0; self.pr = 0; self.rf = 0; self.rr = 0
        self.rf_t = []; self.rr_t = []
        self._stopev = threading.Event()
    def stop(self): self._stopev.set(); self.join(1.0)
    def snap(self):
        with self.lock:
            return (self.pf, self.pr, self.rf, self.rr,
                    list(self.rf_t), list(self.rr_t))
    def run(self):
        period = 1.0 / SAMPLE_HZ; h = self.h
        lr = lgpio.gpio_read(h, ROT_GPIO); cr = None; rn = 0; lf_r = None; lr_r = None
        lp = lr; cp = None; rp = 0; lf_p = None; lr_p = None; deci = 0
        nxt = time.monotonic()
        while not self._stopev.is_set():
            nxt += period
            raw = lgpio.gpio_read(h, ROT_GPIO); t = time.monotonic()
            if raw == lr: cr = None; rn = 0
            else:
                if raw == cr: rn += 1
                else: cr = raw; rn = 1
                if rn >= REF_CONFIRM:
                    pv = lr; lr = raw; cr = None; rn = 0
                    if pv == 1 and lr == 0 and (lf_r is None or t-lf_r >= REF_MIN_INTER_S):
                        lf_r = t
                        with self.lock: self.rf += 1; self.rf_t.append(t)
                    elif pv == 0 and lr == 1 and (lr_r is None or t-lr_r >= REF_MIN_INTER_S):
                        lr_r = t
                        with self.lock: self.rr += 1; self.rr_t.append(t)
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
                            with self.lock: self.pf += 1
                        elif pv == 0 and lp == 1 and (lr_p is None or t-lr_p >= MIN_INTER_S):
                            lr_p = t
                            with self.lock: self.pr += 1
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


def start_recording():
    OUT.mkdir(parents=True, exist_ok=True)
    subprocess.run(["docker", "rm", "-f", "rot-rec"], capture_output=True)
    proc = subprocess.Popen(
        ["docker", "run", "--rm", "--network", "host", "--name", "rot-rec",
         "--entrypoint", "ffmpeg", "-v", f"{OUT}:/out", IMG,
         "-y", "-loglevel", "error", "-rtsp_transport", "tcp", "-i", RTSP,
         "-c", "copy", "-movflags", "+frag_keyframe+empty_moov+default_base_moof",
         "-f", "mp4", "/out/run.mp4"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    return proc

def stop_recording(proc):
    subprocess.run(["docker", "stop", "-t", "2", "rot-rec"], capture_output=True)
    try: proc.wait(timeout=8)
    except Exception:
        try: proc.kill()
        except Exception: pass


def leg(s, pca, direction, last_dir, label, rec_t0):
    """Drive one leg. direction +1 unwind (falling), -1 wind (rising).

    Emulates the firmware: count the direction's polled snap, and on a reversal
    (or first move) skip the first one (phantom). Ground truth from the 2 kHz
    reference snap timestamps.
    Returns a result dict.
    """
    pwm = UNWIND_US if direction > 0 else WIND_US
    pf0, pr0, rf0, rr0, rft0, rrt0 = s.snap()
    base_poll = pf0 if direction > 0 else pr0
    base_ref_n = rf0 if direction > 0 else rr0
    skip_armed = (direction != last_dir)          # reversal or first move
    phantom_skipped = False
    first_snap_t = None
    wall0 = time.time()
    t0 = time.monotonic(); last_edge_t = t0
    pca.set_pulse(pwm)
    outcome = "running"; delivered = 0
    while True:
        pf, pr, rf, rr, rft, rrt = s.snap()
        raw_polled = (pf if direction > 0 else pr) - base_poll
        # Firmware reversal-skip, time-guarded: drop the first snap only if it
        # arrives within the guard window (a phantom re-cross); a late first
        # snap is a real revolution and must be kept.
        if raw_polled >= 1 and first_snap_t is None:
            first_snap_t = time.monotonic()
            if skip_armed and (first_snap_t - t0) < PHANTOM_GUARD_S:
                phantom_skipped = True
        eff = raw_polled - (1 if phantom_skipped else 0)
        now = time.monotonic()
        if eff > delivered:
            delivered = eff; last_edge_t = now
        if delivered >= TARGET:
            outcome = "target_reached"; break
        if (now - last_edge_t) >= STALL_THRESHOLD_S and raw_polled == 0 and (now - t0) >= STALL_THRESHOLD_S:
            outcome = "stalled"; break
        if (now - t0) >= CAP_S:
            outcome = "timed_out"; break
        time.sleep(0.004)
    pca.stop(); time.sleep(0.7)

    pf, pr, rf, rr, rft, rrt = s.snap()
    ref_times = (rft if direction > 0 else rrt)
    leg_ref = [t for t in ref_times if t >= t0]          # snaps since motion start
    lat = [round(t - t0, 3) for t in leg_ref]
    n_ref = len(leg_ref)
    phantom = bool(lat) and lat[0] < PHANTOM_GUARD_S
    physical_revs = n_ref - (1 if phantom else 0)
    raw_polled = (pf if direction > 0 else pr) - base_poll
    # rpm from the real-rev snaps (drop phantom)
    real_t = leg_ref[1:] if phantom else leg_ref
    if len(real_t) >= 2:
        iv = [real_t[i]-real_t[i-1] for i in range(1, len(real_t))]
        rpm = 60.0/(sum(iv)/len(iv))
    else:
        rpm = 0.0
    dname = "unwind" if direction > 0 else "wind"
    ok = (physical_revs == TARGET)
    print(f"  {label:14s} {dname:6s} reversal={str(skip_armed):5s} "
          f"ref_snaps={n_ref} phantom={str(phantom):5s} "
          f"PHYSICAL={physical_revs} polled_raw={raw_polled} rpm={rpm:5.1f} "
          f"{outcome} {'OK' if ok else '*** FAIL ***'}")
    print(f"                 ref snap latencies (s from motion start): {lat}")
    return {"label": label, "dir": dname, "pwm": pwm, "target": TARGET,
            "reversal": skip_armed, "ref_snaps": n_ref, "phantom": phantom,
            "physical_revs": physical_revs, "polled_raw": raw_polled,
            "rpm": round(rpm, 1), "outcome": outcome, "ok": ok,
            "ref_latencies_s": lat,
            "vid_start_s": round(wall0 - rec_t0, 2),
            "vid_stop_s": round(time.time() - rec_t0, 2)}


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

    rows = []; grabs = []; rec = None
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

        rec = start_recording(); rec_t0 = time.time(); time.sleep(2.5)
        print(f"recording -> {VIDEO}")

        # (direction, label) sequence: same-dir back-to-back, then reversals
        seq = [
            (-1, "A1_wind"), (-1, "A2_wind"), (-1, "A3_wind"),
            (+1, "B1_unwind"), (+1, "B2_unwind"), (+1, "B3_unwind"),
            (-1, "C1_wind"), (+1, "C2_unwind"), (-1, "C3_wind"), (+1, "C4_unwind"),
        ]
        ok = grab("00_start.jpg"); grabs.append(("00_start.jpg", ok))
        last_dir = 0
        for i, (d, label) in enumerate(seq, 1):
            r = leg(s, pca, d, last_dir, label, rec_t0)
            rows.append(r)
            g = f"{i:02d}_{label}.jpg"; grabs.append((g, grab(g)))
            last_dir = d
            time.sleep(1.2)

        time.sleep(1.0)
        s.stop(); pca.close(); lgpio.gpiochip_close(h)
    finally:
        if rec is not None:
            stop_recording(rec)
        guard_stop.set()
        name = restart_ext(); print(f"restarted extension: {name}")

    all_ok = all(r["ok"] for r in rows)
    same_dir_ok = all(r["ok"] for r in rows if not r["reversal"])
    reversal_ok = all(r["ok"] for r in rows if r["reversal"])
    summary = {"target": TARGET, "all_ok": all_ok,
               "same_dir_ok": same_dir_ok, "reversal_ok": reversal_ok,
               "rows": rows, "grabs": grabs, "video": str(VIDEO)}
    (OUT / "results.json").write_text(json.dumps(summary, indent=2))
    print(f"\nSAME-DIR legs all 3/3: {same_dir_ok}")
    print(f"REVERSAL legs all 3/3: {reversal_ok}")
    print(f"ALL legs 3 physical revs: {all_ok}")
    print(f"video: {VIDEO}")
    print(f"snaps dir: {SNAPS}")
    print(f"Wrote {OUT/'results.json'}")


if __name__ == "__main__":
    main()
