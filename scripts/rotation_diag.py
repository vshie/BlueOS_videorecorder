#!/usr/bin/env python3
"""Rotation-sensor diagnostic for DeckHand.

Commands 1, 3, and 5 rotations in both directions (unwind then wind),
while sampling GPIO 10 (rotation sensor) at ~1 kHz via pigpio and
capturing a camera still of the white W-disc after each move.

Outputs (under OUT_DIR):
  samples.csv          wall_time_s, gpio10, rotation_count, phase, move_id
  edges.csv            counted edges from telemetry jumps
  moves.json           per-move summary (commanded vs delivered, duration)
  snaps/*.jpg          disc photos after each move (+ start reference)
  plot_signal.png      GPIO waveform with move windows + edge markers
"""

from __future__ import annotations

import csv
import json
import os
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import pigpio
import requests

EXT = "http://127.0.0.1:5423"
RTSP = "rtsp://127.0.0.1:8554/video_stream__dev_video2"
GPIO = 10
SAMPLE_HZ = 1000
OUT_DIR = Path(os.environ.get(
    "ROT_DIAG_OUT",
    f"/home/pi/rot_diag_{datetime.now(timezone.utc).strftime('%Y%m%d_%H%M%S')}",
))

# (direction, rotations) — unwind first, then wind, for each N
MOVES = [
    ("unwind", 1),
    ("wind", 1),
    ("unwind", 3),
    ("wind", 3),
    ("unwind", 5),
    ("wind", 5),
]


class Sampler:
    def __init__(self, pi: pigpio.pi):
        self.pi = pi
        self.lock = threading.Lock()
        self.rows: list[tuple[float, int, int | None, str, str]] = []
        self._stop = threading.Event()
        self.phase = "idle"
        self.move_id = ""
        self.rot_count: int | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)

    def set_meta(self, phase: str, move_id: str = "") -> None:
        self.phase = phase
        self.move_id = move_id

    def set_rot_count(self, n: int | None) -> None:
        self.rot_count = n

    def _run(self) -> None:
        period = 1.0 / SAMPLE_HZ
        t0 = time.monotonic()
        next_t = t0
        while not self._stop.is_set():
            now = time.monotonic()
            level = self.pi.read(GPIO)
            with self.lock:
                self.rows.append(
                    (now - t0, level, self.rot_count, self.phase, self.move_id)
                )
            next_t += period
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # fell behind — resync
                next_t = time.monotonic()


def telemetry() -> dict:
    return requests.get(f"{EXT}/telemetry", timeout=5).json()


def release_get() -> dict:
    return requests.get(f"{EXT}/release", timeout=5).json()


def release_rotate(direction: str, rotations: int) -> dict:
    return requests.post(
        f"{EXT}/release",
        json={"action": "rotate", "direction": direction, "rotations": rotations},
        timeout=10,
    ).json()


def release_stop() -> None:
    requests.post(f"{EXT}/release", json={"action": "stop"}, timeout=5)


def wait_idle(timeout_s: float = 90.0) -> None:
    t0 = time.monotonic()
    # give the run a moment to start
    time.sleep(0.15)
    while time.monotonic() - t0 < timeout_s:
        try:
            if not release_get().get("running"):
                return
        except Exception:
            pass
        time.sleep(0.05)
    raise TimeoutError("release still running after timeout")


def snapshot(path: Path) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    # ffmpeg lives inside the videorecorder container on BlueOS hosts.
    cid = subprocess.check_output(
        ["docker", "ps", "--filter", "name=videorecorder", "--format", "{{.ID}}"],
        text=True,
    ).strip().splitlines()[0]
    # Write into a bind-mounted path if available; else docker cp after.
    # /app/videorecordings is typically bind-mounted to host storage.
    container_path = f"/tmp/{path.name}"
    cmd = [
        "docker", "exec", cid,
        "ffmpeg", "-y", "-loglevel", "error",
        "-rtsp_transport", "tcp",
        "-i", RTSP,
        "-frames:v", "1",
        "-q:v", "2",
        container_path,
    ]
    for attempt in range(3):
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            subprocess.run(
                ["docker", "cp", f"{cid}:{container_path}", str(path)],
                check=False, capture_output=True,
            )
            if path.exists() and path.stat().st_size > 1000:
                return True
        time.sleep(0.4)
    print(f"  snapshot FAILED: {path.name} stderr={r.stderr[:200]!r}")
    return False


def plot_signal(rows, moves_meta, out_png: Path) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed — skipping plot_signal.png")
        return

    t = [r[0] for r in rows]
    g = [r[1] for r in rows]
    fig, ax = plt.subplots(figsize=(16, 5))
    ax.step(t, g, where="post", linewidth=0.6, color="#1a5f7a", label="GPIO10")
    ax.set_ylim(-0.1, 1.3)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("GPIO 10 level")
    ax.set_title("Rotation sensor (GPIO 10) during 1/3/5-turn wind & unwind jogs")

    colors = {
        "unwind": "#e67e22",
        "wind": "#2980b9",
    }
    for m in moves_meta:
        c = colors.get(m["direction"], "#888")
        ax.axvspan(m["t_start"], m["t_end"], alpha=0.15, color=c)
        ax.axvline(m["t_start"], color=c, linewidth=0.8, alpha=0.7)
        mid = (m["t_start"] + m["t_end"]) / 2
        ax.text(
            mid, 1.15,
            f"{m['direction'][0].upper()}{m['commanded']}\n"
            f"got {m['delivered']}",
            ha="center", va="top", fontsize=8, color=c,
        )
        for et in m.get("edge_times", []):
            ax.plot(et, 0.5, "v", color="crimson", markersize=5)

    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    print(f"wrote {out_png}")


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    snap_dir = OUT_DIR / "snaps"
    snap_dir.mkdir(exist_ok=True)

    print(f"OUT_DIR={OUT_DIR}")
    release_stop()
    time.sleep(0.3)

    tel0 = telemetry()
    print(
        f"sensor_available={tel0.get('rotation_sensor_available')} "
        f"count={tel0.get('rotation_count')} rpm={tel0.get('rotation_rpm')}"
    )

    pi = pigpio.pi()
    if not pi.connected:
        raise SystemExit("pigpio daemon not connected")

    sampler = Sampler(pi)
    sampler.set_rot_count(tel0.get("rotation_count"))
    sampler.start()
    t_mono0 = time.monotonic()

    # Reference snap before any motion
    sampler.set_meta("snap_start", "start")
    snapshot(snap_dir / "00_start.jpg")
    time.sleep(0.5)

    moves_meta: list[dict] = []
    rot_poll_stop = threading.Event()

    def rot_poller() -> None:
        while not rot_poll_stop.is_set():
            try:
                n = telemetry().get("rotation_count")
                sampler.set_rot_count(n)
            except Exception:
                pass
            time.sleep(0.05)

    poll_thread = threading.Thread(target=rot_poller, daemon=True)
    poll_thread.start()

    for i, (direction, n) in enumerate(MOVES, 1):
        move_id = f"{i:02d}_{direction}_{n}"
        print(f"\n=== MOVE {move_id} ===")
        count_before = telemetry().get("rotation_count") or 0
        sampler.set_meta("pre", move_id)
        time.sleep(0.4)

        t_start = time.monotonic() - t_mono0
        sampler.set_meta("moving", move_id)
        resp = release_rotate(direction, n)
        print(f"  POST rotate -> {resp}")
        wait_idle()
        t_end = time.monotonic() - t_mono0
        release_stop()

        # settle for photo + let last edge land
        sampler.set_meta("settle", move_id)
        time.sleep(0.8)
        count_after = telemetry().get("rotation_count") or 0
        delivered = count_after - count_before

        # edge times: falling transitions in sample buffer during move window
        with sampler.lock:
            window = [r for r in sampler.rows if t_start <= r[0] <= t_end]
        edge_times = []
        for a, b in zip(window, window[1:]):
            if a[1] == 1 and b[1] == 0:
                edge_times.append(b[0])

        meta = {
            "move_id": move_id,
            "direction": direction,
            "commanded": n,
            "count_before": count_before,
            "count_after": count_after,
            "delivered": delivered,
            "error_rotations": delivered - n,
            "t_start": t_start,
            "t_end": t_end,
            "duration_s": t_end - t_start,
            "falling_edges_in_gpio_trace": len(edge_times),
            "edge_times": edge_times,
        }
        moves_meta.append(meta)
        print(
            f"  commanded={n} delivered={delivered} "
            f"err={delivered - n:+d} dur={meta['duration_s']:.2f}s "
            f"gpio_falling={len(edge_times)}"
        )

        sampler.set_meta("snap", move_id)
        snapshot(snap_dir / f"{move_id}.jpg")
        sampler.set_meta("idle", "")
        time.sleep(0.5)

    rot_poll_stop.set()
    poll_thread.join(timeout=2)
    sampler.set_meta("done", "")
    time.sleep(0.3)
    sampler.stop()
    pi.stop()
    release_stop()

    # Write samples.csv
    samples_path = OUT_DIR / "samples.csv"
    with samples_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "gpio10", "rotation_count", "phase", "move_id"])
        with sampler.lock:
            w.writerows(sampler.rows)
    print(f"wrote {samples_path} ({len(sampler.rows)} rows)")

    edges_path = OUT_DIR / "edges.csv"
    with edges_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["move_id", "direction", "commanded", "edge_index", "t_s"])
        for m in moves_meta:
            for j, et in enumerate(m["edge_times"], 1):
                w.writerow([m["move_id"], m["direction"], m["commanded"], j, f"{et:.6f}"])
    print(f"wrote {edges_path}")

    moves_path = OUT_DIR / "moves.json"
    # strip long edge lists from printed summary duplicate — keep in file
    moves_path.write_text(json.dumps(moves_meta, indent=2))
    print(f"wrote {moves_path}")

    plot_signal(sampler.rows, moves_meta, OUT_DIR / "plot_signal.png")

    print("\n=== SUMMARY ===")
    for m in moves_meta:
        print(
            f"  {m['move_id']:20s} cmd={m['commanded']} "
            f"got={m['delivered']} err={m['error_rotations']:+d} "
            f"gpio_edges={m['falling_edges_in_gpio_trace']} "
            f"dur={m['duration_s']:.2f}s"
        )
    print(f"\nDone. Results in {OUT_DIR}")


if __name__ == "__main__":
    main()
