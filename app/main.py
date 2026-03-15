"""
DropCam - Standalone BlueOS Video Recording Extension

Records H264 video from /dev/video2 into .mp4 files (recorded as power-cut-safe
MPEG-TS, then remuxed to MP4 on stop for VLC/subtitle compatibility), or captures
stills at a configurable interval. Controls a camera tilt servo, lumen light, and
RGB status LED. Supports auto-start via saved recording recipes.
"""

from flask import Flask, Response, jsonify, request, send_file
import io
import json
import os
import re
import subprocess
import shlex
import shutil
import signal
import threading
import time
import logging
import zipfile
from datetime import datetime

app = Flask(__name__)
app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# ── Imports from local modules ───────────────────────────────────────────
from hardware import hw
from scheduler import scheduler
from system_telemetry import (
    get_cpu_temperature, get_cpu_voltage, get_cpu_clock_mhz,
    get_cpu_load_avg, is_time_synced, get_disk_free_mb, get_all_telemetry,
)
from recipes import (
    init_default_recipes, list_recipes, get_recipe,
    save_recipe, delete_recipe, calculate_sweep_time,
)
import usb_storage

# ── Constants ────────────────────────────────────────────────────────────
VIDEO_DIR = "/app/videorecordings"
CONFIG_FILE = os.path.join(VIDEO_DIR, "recipes", "dropcam_config.json")
_OLD_CONFIG_FILE = os.path.join(VIDEO_DIR, "dropcam_config.json")
VIDEO_DEVICE = "/dev/video2"
AUDIO_DEVICE = "hw:Camera,0"

# ── Recording state ──────────────────────────────────────────────────────
gst_process = None
recording = False
start_time = None
current_video_file = None
current_ass_file = None
current_events_file = None

ass_thread = None
stop_ass_thread = False

gst_stderr_thread = None
stop_gst_stderr_thread = False
gst_error_count = 0
gst_warning_count = 0

watchdog_thread = None
stop_watchdog_thread = False
file_stall_count = 0

stills_thread = None
stop_stills_thread = False
stills_dir = None
stills_count = 0

remux_active = False
remux_filename = ""
remux_progress = 0
remux_stage = ""

active_recipe_for_recording = None
image_rotation = 0

usb_recording = False
usb_failover_count = 0
recording_base_dir = None

# ── Config ───────────────────────────────────────────────────────────────

def load_config():
    defaults = {"active_recipe_id": None, "rotation_degrees": 0}

    # One-time migration: move config from old location to recipes/ subfolder
    if not os.path.exists(CONFIG_FILE) and os.path.exists(_OLD_CONFIG_FILE):
        try:
            os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
            os.rename(_OLD_CONFIG_FILE, CONFIG_FILE)
            logger.info(f"Migrated config from {_OLD_CONFIG_FILE} to {CONFIG_FILE}")
        except Exception as e:
            logger.warning(f"Config migration failed: {e}")

    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r") as f:
                raw = f.read().strip()
            if raw:
                saved = json.loads(raw)
                defaults.update(saved)
            else:
                logger.warning("Config file exists but is empty (possible power-cut corruption)")
    except json.JSONDecodeError as e:
        logger.warning(f"Config file corrupt (possible power-cut corruption): {e}")
    except Exception as e:
        logger.warning(f"Could not load config: {e}")
    return defaults


def save_config(cfg):
    """Atomic config write: write to temp file then rename to prevent corruption."""
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        tmp = CONFIG_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(cfg, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, CONFIG_FILE)
    except Exception as e:
        logger.error(f"Could not save config: {e}")

_cfg = load_config()
image_rotation = _cfg.get("rotation_degrees", 0)

# ── ASS subtitle generation (system telemetry) ──────────────────────────

def create_ass_file(video_path):
    base = os.path.splitext(video_path)[0]
    ass_path = base + ".ass"
    with open(ass_path, "w") as f:
        f.write("[Script Info]\n")
        f.write("Title: DropCam Telemetry\n")
        f.write("ScriptType: v4.00+\n")
        f.write("WrapStyle: 0\n")
        f.write("ScaledBorderAndShadow: yes\n")
        f.write("YCbCr Matrix: TV.601\n")
        f.write("PlayResX: 1920\n")
        f.write("PlayResY: 1080\n\n")
        f.write("[V4+ Styles]\n")
        f.write("Format: Name, Fontname, Fontsize, PrimaryColour, SecondaryColour, "
                "OutlineColour, BackColour, Bold, Italic, Underline, StrikeOut, "
                "ScaleX, ScaleY, Spacing, Angle, BorderStyle, Outline, Shadow, "
                "Alignment, MarginL, MarginR, MarginV, Encoding\n")
        f.write("Style: Default,Arial,20,&H00FFFFFF,&H000000FF,&H00000000,"
                "&H00000000,0,0,0,0,100,100,0,0,1,2,1,2,10,10,20,1\n\n")
        f.write("[Events]\n")
        f.write("Format: Layer, Start, End, Style, Name, MarginL, MarginR, MarginV, Effect, Text\n")
    return ass_path


def format_ass_ts(seconds):
    h = int(seconds // 3600)
    m = int((seconds % 3600) // 60)
    s = int(seconds % 60)
    cs = int((seconds - int(seconds)) * 100)
    return f"{h:02d}:{m:02d}:{s:02d}.{cs:02d}"


def parse_ass_ts(ts):
    parts = ts.strip().split(":")
    h, m = int(parts[0]), int(parts[1])
    sp = parts[2].split(".")
    s = int(sp[0])
    cs = int(sp[1]) if len(sp) > 1 else 0
    return h * 3600 + m * 60 + s + cs / 100.0


def update_ass_file():
    global stop_ass_thread, current_ass_file
    rate = 2
    while not stop_ass_thread and recording and current_ass_file:
        try:
            if start_time:
                elapsed = (datetime.now() - start_time).total_seconds()
                t0 = format_ass_ts(elapsed)
                t1 = format_ass_ts(elapsed + 1 / rate)

                cpu_t = get_cpu_temperature()
                cpu_v = get_cpu_voltage()
                cpu_c = get_cpu_clock_mhz()
                cpu_load = get_cpu_load_avg()
                servo = hw.get_servo_position()
                light = hw.get_light_brightness()
                rname = active_recipe_for_recording["name"] if active_recipe_for_recording else "Manual"
                ok = "OK" if file_stall_count == 0 else f"STALL({file_stall_count})"
                now_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

                parts = []
                parts.append(f"Time:{now_str}")
                parts.append(f"Servo:{servo}us")
                parts.append(f"Light:{light}%")
                parts.append(f"Recipe:{rname}")
                parts.append(f"Rec:{ok}")
                if cpu_t is not None:
                    parts.append(f"CPU:{cpu_t}C")
                if cpu_v is not None:
                    parts.append(f"{cpu_v}V")
                if cpu_c is not None:
                    parts.append(f"{int(cpu_c)}MHz")
                if cpu_load is not None:
                    parts.append(f"Load:{cpu_load:.2f}")
                text = " | ".join(parts)

                line = f"Dialogue: 0,{t0},{t1},Default,,0,0,0,,{text}\n"
                with open(current_ass_file, "a") as f:
                    f.write(line)

            time.sleep(1 / rate)
        except Exception as e:
            logger.error(f"ASS subtitle error: {e}")
            time.sleep(1)


def adjust_ass_timing(ass_path, video_duration):
    """Finalize ASS subtitle timing: scale to match video duration and chain
    each line's end time to the next line's start so subtitles never disappear."""
    try:
        with open(ass_path, "r") as f:
            lines = f.readlines()
        header, dialogues = [], []
        max_t = 0
        for line in lines:
            if line.startswith("Dialogue:"):
                dialogues.append(line)
                parts = line.split(",", 9)
                if len(parts) >= 3:
                    max_t = max(max_t, parse_ass_ts(parts[2]))
            else:
                header.append(line)
        if not dialogues or max_t == 0:
            return

        scale = video_duration / max_t if abs(video_duration / max_t - 1.0) >= 0.005 else 1.0

        parsed = []
        for line in dialogues:
            parts = line.split(",", 9)
            if len(parts) >= 3:
                t0 = parse_ass_ts(parts[1]) * scale
                t1 = parse_ass_ts(parts[2]) * scale
                parsed.append((t0, t1, parts))

        with open(ass_path, "w") as f:
            for line in header:
                f.write(line)
            for i, (t0, t1, parts) in enumerate(parsed):
                parts[1] = format_ass_ts(t0)
                if i + 1 < len(parsed):
                    parts[2] = format_ass_ts(parsed[i + 1][0])
                else:
                    parts[2] = format_ass_ts(video_duration)
                f.write(",".join(parts))
        if scale != 1.0:
            logger.info(f"ASS timing: scaled by {scale:.4f}, chained end times")
        else:
            logger.info("ASS timing: chained end times (no scaling needed)")
    except Exception as e:
        logger.error(f"ASS timing adjust error: {e}")

# ── Events log ───────────────────────────────────────────────────────────

def create_events_file(video_path):
    base = os.path.splitext(video_path)[0]
    path = base + "_events.ndjson"
    open(path, "w").close()
    return path


def log_event(event_type, detail=""):
    if not current_events_file:
        return
    try:
        evt = {
            "ts": time.time(),
            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
            "event": event_type,
            "detail": str(detail),
        }
        with open(current_events_file, "a") as f:
            f.write(json.dumps(evt) + "\n")
    except Exception:
        pass

# ── GStreamer stderr monitor ─────────────────────────────────────────────

def gst_stderr_monitor(process):
    global gst_error_count, gst_warning_count
    for line in iter(process.stderr.readline, b""):
        if stop_gst_stderr_thread:
            break
        decoded = line.decode("utf-8", errors="replace").strip()
        if not decoded:
            continue
        upper = decoded.upper()
        if "ERROR" in upper:
            gst_error_count += 1
            logger.error(f"GST_ERROR: {decoded}")
            log_event("gst_error", decoded)
        elif "WARNING" in upper:
            gst_warning_count += 1
            logger.warning(f"GST_WARN: {decoded}")
            log_event("gst_warning", decoded)

# ── Health watchdog ──────────────────────────────────────────────────────

def _usb_failover():
    """Kill current recording on USB and restart on local SD card."""
    global gst_process, recording, start_time, current_video_file
    global current_ass_file, current_events_file
    global ass_thread, stop_ass_thread
    global gst_stderr_thread, stop_gst_stderr_thread
    global stills_thread, stop_stills_thread, stills_dir
    global usb_recording, usb_failover_count, recording_base_dir

    logger.warning("USB failover: storage lost, switching to local SD")
    log_event("usb_failover", "USB storage lost during recording, restarting on local SD")
    usb_failover_count += 1

    saved_recipe = active_recipe_for_recording
    saved_mode = (saved_recipe or {}).get("mode", "video")
    saved_interval = (saved_recipe or {}).get("still_interval_s", 1.0)
    saved_rotation = image_rotation

    stop_ass_thread = True
    if ass_thread and ass_thread.is_alive():
        ass_thread.join(timeout=2)

    stop_stills_thread = True
    if stills_thread and stills_thread.is_alive():
        stills_thread.join(timeout=3)

    stop_gst_stderr_thread = True
    if gst_stderr_thread and gst_stderr_thread.is_alive():
        gst_stderr_thread.join(timeout=2)

    if gst_process:
        try:
            gst_process.kill()
            gst_process.wait(timeout=5)
        except Exception:
            pass

    recording = False
    start_time = None
    gst_process = None
    current_video_file = None
    current_ass_file = None
    current_events_file = None
    stills_dir = None
    usb_recording = False
    recording_base_dir = None

    hw.led_warning()
    time.sleep(1)

    ok = _start_recording_internal(
        mode=saved_mode,
        still_interval_s=saved_interval,
        rotation=saved_rotation,
        force_local=True,
    )
    if ok:
        logger.info("USB failover: recording resumed on local SD")
    else:
        logger.error("USB failover: failed to restart recording on local SD")
        hw.led_warning()


def recording_health_watchdog():
    global file_stall_count
    last_size = 0
    file_stall_count = 0

    while not stop_watchdog_thread and recording:
        try:
            if usb_recording and not usb_storage.is_healthy():
                _usb_failover()
                return

            if current_video_file and os.path.exists(current_video_file):
                sz = os.path.getsize(current_video_file)
                growth = sz - last_size
                if last_size > 0 and growth == 0:
                    file_stall_count += 1
                    log_event("file_stall", f"No growth for {file_stall_count} intervals ({sz} bytes)")
                    hw.led_warning()
                else:
                    if file_stall_count > 0 and recording:
                        hw.led_recording()
                    file_stall_count = 0
                last_size = sz

            if stills_dir and os.path.isdir(stills_dir):
                pass

            if gst_process and gst_process.poll() is not None:
                log_event("process_died", f"GStreamer exit code {gst_process.returncode}")

            check_path = recording_base_dir or VIDEO_DIR
            disk_free = get_disk_free_mb(check_path)
            if disk_free is not None and disk_free < 1024:
                logger.warning(f"Disk space low: {disk_free} MB, stopping")
                log_event("disk_full", f"{disk_free} MB remaining")
                _stop_recording_internal()
                hw.led_warning()
                return

            time.sleep(5)
        except Exception as e:
            logger.error(f"Watchdog error: {e}")
            time.sleep(5)

# ── Stills capture ───────────────────────────────────────────────────────

def _capture_still(output_path, rotation=0):
    """Capture a single JPEG frame from the USB camera."""
    try:
        tmp = output_path + ".tmp.jpg"
        cmd = [
            "ffmpeg", "-y", "-f", "v4l2", "-input_format", "h264",
            "-video_size", "1920x1080", "-i", VIDEO_DEVICE,
            "-frames:v", "1", "-q:v", "2", tmp,
        ]
        subprocess.run(cmd, timeout=10, capture_output=True)
        if rotation and rotation != 0:
            _rotate_image(tmp, rotation)
        os.rename(tmp, output_path)
        return True
    except Exception as e:
        logger.error(f"Still capture failed: {e}")
        if os.path.exists(output_path + ".tmp.jpg"):
            try:
                os.remove(output_path + ".tmp.jpg")
            except Exception:
                pass
    return False


def _rotate_image(path, degrees):
    try:
        from PIL import Image
        img = Image.open(path)
        rotated = img.rotate(-degrees, expand=True)
        rotated.save(path, "JPEG", quality=92)
    except ImportError:
        logger.warning("Pillow not available, skipping rotation")
    except Exception as e:
        logger.error(f"Image rotation failed: {e}")


def stills_capture_loop(interval_s, rotation):
    global stop_stills_thread, stills_count
    while not stop_stills_thread and recording:
        stills_count += 1
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        prefix = (active_recipe_for_recording or {}).get("still_prefix", "").strip()
        fname = f"{prefix}_{ts}.jpg" if prefix else f"{ts}.jpg"
        fpath = os.path.join(stills_dir, fname)
        _capture_still(fpath, rotation)
        wait_start = time.monotonic()
        while not stop_stills_thread and (time.monotonic() - wait_start) < interval_s:
            time.sleep(min(0.05, interval_s))

# ── Get video duration ───────────────────────────────────────────────────

def get_video_duration(path):
    """Return (duration, start_time) tuple, or (None, 0.0) on failure."""
    try:
        cmd = [
            "ffprobe", "-v", "error",
            "-show_entries", "format=duration,start_time",
            "-of", "default=noprint_wrappers=1:nokey=0", path,
        ]
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        if r.returncode == 0:
            vals = {}
            for line in r.stdout.strip().splitlines():
                if "=" in line:
                    k, v = line.split("=", 1)
                    vals[k.strip()] = float(v.strip())
            return vals.get("duration"), vals.get("start_time", 0.0)
    except Exception as e:
        logger.error(f"ffprobe error: {e}")
    return None, 0.0

# ── Core recording start/stop ────────────────────────────────────────────

def start_recording_with_recipe(recipe):
    """Called by the scheduler or directly. Returns True on success."""
    global active_recipe_for_recording, image_rotation
    active_recipe_for_recording = recipe
    image_rotation = recipe.get("rotation_degrees", 0)
    return _start_recording_internal(
        mode=recipe.get("mode", "video"),
        still_interval_s=recipe.get("still_interval_s", 1.0),
        rotation=image_rotation,
    )


def _start_recording_internal(mode="video", still_interval_s=1.0, rotation=0,
                               force_local=False):
    global gst_process, recording, start_time, current_video_file
    global current_ass_file, current_events_file
    global ass_thread, stop_ass_thread
    global gst_stderr_thread, stop_gst_stderr_thread, gst_error_count, gst_warning_count
    global watchdog_thread, stop_watchdog_thread, file_stall_count
    global stills_thread, stop_stills_thread, stills_dir, stills_count
    global usb_recording, recording_base_dir

    if recording:
        return False

    os.makedirs(VIDEO_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if active_recipe_for_recording:
        safe_name = re.sub(r'[^a-zA-Z0-9_-]', '', active_recipe_for_recording.get("name", "recipe").replace(' ', '_'))
    else:
        safe_name = None

    use_usb = (not force_local) and usb_storage.is_usable()
    if use_usb:
        subfolder = f"{safe_name}_{timestamp}" if safe_name else f"manual_{timestamp}"
        rec_dir = usb_storage.get_recording_dir(subfolder)
        usb_recording = True
        logger.info(f"Recording to USB: {rec_dir}")
    else:
        rec_dir = VIDEO_DIR
        usb_recording = False

    recording_base_dir = rec_dir

    if mode == "video":
        if use_usb:
            basename = f"{safe_name}_{timestamp}" if safe_name else f"manual_{timestamp}"
        else:
            basename = f"recipe_{safe_name}_{timestamp}" if safe_name else f"manual_{timestamp}"
        filename = basename + ".ts"
        filepath = os.path.join(rec_dir, filename)
        current_video_file = filepath

        pipeline = (
            f"v4l2src do-timestamp=true device={VIDEO_DEVICE} ! "
            "video/x-h264,width=1920,height=1080,framerate=30/1 ! "
            "h264parse ! queue ! mux. "
            f"alsasrc device={AUDIO_DEVICE} ! "
            "audio/x-raw,format=S16LE,rate=44100,channels=1 ! "
            "audioconvert ! audioresample ! avenc_aac ! queue ! mux. "
            f"mpegtsmux name=mux ! filesink location={filepath}"
        )
        command = ["gst-launch-1.0", "-e"] + shlex.split(pipeline)

        try:
            gst_process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.PIPE
            )
            start_time = datetime.now()
            logger.info(f"GStreamer command: {' '.join(command)}")
            if gst_process.poll() is not None:
                out, err = gst_process.communicate()
                logger.error(f"GStreamer failed: {err.decode()}")
                gst_process = None
                start_time = None
                return False
            time.sleep(2)
            if gst_process.poll() is not None:
                out, err = gst_process.communicate()
                logger.error(f"GStreamer died during startup: {err.decode()}")
                gst_process = None
                start_time = None
                return False
        except Exception as e:
            logger.error(f"Failed to start GStreamer: {e}")
            gst_process = None
            start_time = None
            return False

        current_ass_file = create_ass_file(filepath)
        current_events_file = create_events_file(filepath)

        stop_gst_stderr_thread = False
        gst_error_count = 0
        gst_warning_count = 0
        gst_stderr_thread = threading.Thread(
            target=gst_stderr_monitor, args=(gst_process,), daemon=True
        )
        gst_stderr_thread.start()

    elif mode == "stills":
        if use_usb:
            stills_dir = rec_dir
        elif safe_name:
            stills_dir = os.path.join(VIDEO_DIR, f"stills_recipe_{safe_name}_{timestamp}")
        else:
            stills_dir = os.path.join(VIDEO_DIR, f"stills_manual_{timestamp}")
        os.makedirs(stills_dir, exist_ok=True)
        stills_count = 0
        current_video_file = None

        marker = os.path.join(stills_dir, "session.json")
        current_events_file = os.path.join(stills_dir, "events.ndjson")
        open(current_events_file, "w").close()
        current_ass_file = None

        with open(marker, "w") as f:
            json.dump({"started": timestamp, "interval_s": still_interval_s, "rotation": rotation}, f)

        start_time = datetime.now()
        recording = True
        stop_stills_thread = False
        stills_thread = threading.Thread(
            target=stills_capture_loop, args=(still_interval_s, rotation), daemon=True
        )
        stills_thread.start()
    else:
        logger.error(f"Unknown mode: {mode}")
        return False

    recording = True

    if current_ass_file:
        stop_ass_thread = False
        ass_thread = threading.Thread(target=update_ass_file, daemon=True)
        ass_thread.start()

    stop_watchdog_thread = False
    file_stall_count = 0
    watchdog_thread = threading.Thread(target=recording_health_watchdog, daemon=True)
    watchdog_thread.start()

    hw.led_recording()
    storage_label = "USB" if usb_recording else "local"
    log_event("recording_started", f"mode={mode}, storage={storage_label}")
    logger.info(f"Recording started: mode={mode}, storage={storage_label}")
    return True


def _run_ffmpeg_remux(ts_path, mp4_path, ts_size):
    """Run the ffmpeg copy-remux, tracking progress.  Returns True on success."""
    global remux_progress
    size_gib = ts_size / (1024 ** 3)
    timeout_s = int(180 + size_gib * 180)

    cmd = ["ffmpeg", "-y", "-i", ts_path, "-c", "copy"]
    if size_gib <= 4:
        cmd += ["-movflags", "+faststart"]
    else:
        logger.info(f"Skipping +faststart for {size_gib:.1f} GiB file to reduce remux time")
    cmd.append(mp4_path)

    logger.info(f"Remuxing {size_gib:.1f} GiB TS→MP4 (timeout {timeout_s}s)…")
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    deadline = time.monotonic() + timeout_s
    while proc.poll() is None:
        if time.monotonic() > deadline:
            proc.kill()
            proc.wait()
            raise subprocess.TimeoutExpired(cmd, timeout_s)
        if ts_size > 0:
            try:
                written = os.path.getsize(mp4_path)
                remux_progress = min(99, int(written * 100 / ts_size))
            except OSError:
                pass
        time.sleep(2)

    if proc.returncode == 0 and os.path.exists(mp4_path):
        remux_progress = 100
        return True
    logger.error(f"Remux to MP4 failed (rc={proc.returncode})")
    if os.path.exists(mp4_path):
        try:
            os.remove(mp4_path)
        except OSError:
            pass
    return False


def _copy_file_with_progress(src, dst, label, total_bytes):
    """Copy src to dst, updating remux_progress 0-100 and remux_stage."""
    global remux_progress, remux_stage
    remux_stage = label
    remux_progress = 0
    buf_size = 4 * 1024 * 1024  # 4 MB
    copied = 0
    with open(src, "rb") as fin, open(dst, "wb") as fout:
        while True:
            chunk = fin.read(buf_size)
            if not chunk:
                break
            fout.write(chunk)
            copied += len(chunk)
            if total_bytes > 0:
                remux_progress = min(99, int(copied * 100 / total_bytes))
    remux_progress = 100


def _remux_to_mp4(ts_path, was_usb=False, usb_rec_dir=None):
    """Remux a .ts file to .mp4 with ffmpeg (copy, no re-encode).

    For USB recordings, picks the fastest strategy that fits:
      1. In-place on USB (if USB free >= ts_size)
      2. Remux via local SD, then transfer back (if local SD has room)
      3. Skip remux and keep .ts (if neither has room)

    Returns the final .mp4 path on success, or the original .ts path on failure.
    """
    global remux_active, remux_filename, remux_progress, remux_stage

    show_led = not recording
    if show_led:
        hw.flash_led(0, 255, 0, rate_hz=0.5)

    ts_size = 0
    try:
        ts_size = os.path.getsize(ts_path)
    except OSError:
        pass

    remux_filename = os.path.basename(ts_path)
    remux_progress = 0
    remux_stage = ""
    remux_active = True

    try:
        if was_usb and usb_rec_dir:
            return _remux_usb(ts_path, ts_size, usb_rec_dir, show_led)
        else:
            return _remux_local(ts_path, ts_size, show_led)
    except Exception as e:
        logger.error(f"Remux exception: {e}")
    finally:
        remux_active = False
        remux_filename = ""
        remux_progress = 0
        remux_stage = ""
        if show_led:
            hw.led_idle()
    return ts_path


def _remux_local(ts_path, ts_size, show_led):
    """Standard in-place remux for local SD recordings."""
    global remux_progress, remux_stage
    mp4_path = os.path.splitext(ts_path)[0] + ".mp4"
    remux_stage = "Remuxing TS→MP4"
    if _run_ffmpeg_remux(ts_path, mp4_path, ts_size):
        os.remove(ts_path)
        logger.info(f"Remuxed to MP4: {os.path.basename(mp4_path)}")
        return mp4_path
    return ts_path


def _remux_usb(ts_path, ts_size, usb_rec_dir, show_led):
    """Smart remux for USB recordings.  Picks the best strategy based on space."""
    global remux_progress, remux_stage

    usb_free = usb_storage.get_free_mb()
    usb_free_bytes = (usb_free or 0) * 1024 * 1024
    local_free = get_disk_free_mb(VIDEO_DIR)
    local_free_bytes = (local_free or 0) * 1024 * 1024

    mp4_on_usb = os.path.splitext(ts_path)[0] + ".mp4"
    ts_basename = os.path.splitext(os.path.basename(ts_path))[0]
    size_gib = ts_size / (1024 ** 3)

    # Strategy 1: enough USB space to hold both .ts and .mp4 simultaneously
    if usb_free_bytes >= ts_size:
        logger.info(f"USB remux strategy: in-place ({size_gib:.1f} GiB, "
                    f"{usb_free:.0f} MB free)")
        remux_stage = "Remuxing TS→MP4 on USB"
        if _run_ffmpeg_remux(ts_path, mp4_on_usb, ts_size):
            os.remove(ts_path)
            logger.info(f"Remuxed in-place on USB: {os.path.basename(mp4_on_usb)}")
            return mp4_on_usb
        return ts_path

    # Strategy 2: remux via local SD, then transfer back
    if local_free_bytes >= ts_size:
        logger.info(f"USB remux strategy: via local SD ({size_gib:.1f} GiB, "
                    f"USB {usb_free:.0f} MB / local {local_free:.0f} MB free)")

        local_tmp_mp4 = os.path.join(VIDEO_DIR, ts_basename + ".mp4")

        # Step 1: remux .ts (USB) → .mp4 (local SD)
        remux_stage = "Remuxing TS→MP4 via local SD"
        if not _run_ffmpeg_remux(ts_path, local_tmp_mp4, ts_size):
            return ts_path

        # Step 2: delete .ts from USB to free space
        remux_stage = "Removing TS from USB"
        remux_progress = 0
        try:
            os.remove(ts_path)
            logger.info(f"Deleted TS from USB: {os.path.basename(ts_path)}")
        except OSError as e:
            logger.error(f"Failed to delete TS from USB: {e}")

        # Step 3: copy .mp4 from local SD back to USB
        mp4_size = os.path.getsize(local_tmp_mp4)
        _copy_file_with_progress(
            local_tmp_mp4, mp4_on_usb,
            "Transferring MP4 to USB",
            mp4_size,
        )
        logger.info(f"Transferred MP4 to USB: {os.path.basename(mp4_on_usb)}")

        # Step 4: clean up local temp
        remux_stage = "Cleaning up"
        try:
            os.remove(local_tmp_mp4)
        except OSError:
            pass

        return mp4_on_usb

    # Strategy 3: neither has room — skip remux
    logger.warning(f"USB remux: insufficient space on both USB ({usb_free:.0f} MB) "
                   f"and local ({local_free:.0f} MB) for {size_gib:.1f} GiB file. "
                   f"Keeping .ts on USB.")
    remux_stage = "Skipped — insufficient space"
    remux_progress = 0
    time.sleep(3)
    return ts_path


def _stop_recording_internal():
    global gst_process, recording, start_time, current_video_file
    global current_ass_file, current_events_file
    global ass_thread, stop_ass_thread
    global gst_stderr_thread, stop_gst_stderr_thread
    global watchdog_thread, stop_watchdog_thread
    global stills_thread, stop_stills_thread, stills_dir
    global active_recipe_for_recording
    global usb_recording, recording_base_dir

    if not recording:
        return

    log_event("recording_stopping", "Stop requested")

    video_path = current_video_file
    ass_path = current_ass_file
    events_path = current_events_file

    stop_ass_thread = True
    if ass_thread and ass_thread.is_alive():
        ass_thread.join(timeout=2)

    stop_stills_thread = True
    if stills_thread and stills_thread.is_alive():
        stills_thread.join(timeout=5)

    stop_gst_stderr_thread = True
    if gst_stderr_thread and gst_stderr_thread.is_alive():
        gst_stderr_thread.join(timeout=2)

    stop_watchdog_thread = True
    if watchdog_thread and watchdog_thread.is_alive():
        watchdog_thread.join(timeout=2)

    if gst_process:
        logger.info("Stopping GStreamer gracefully...")
        gst_process.send_signal(signal.SIGINT)
        try:
            gst_process.wait(timeout=7)
            log_event("recording_stopped", "Graceful shutdown")
        except subprocess.TimeoutExpired:
            logger.warning("GStreamer did not stop, force killing")
            log_event("stop_timeout", "Force kill after 7s")
            gst_process.kill()
            gst_process.wait()

    was_usb = usb_recording
    usb_rec_dir = recording_base_dir

    recording = False
    start_time = None
    gst_process = None
    current_video_file = None
    current_ass_file = None
    current_events_file = None
    stills_dir = None
    active_recipe_for_recording = None
    usb_recording = False
    recording_base_dir = None

    if video_path and os.path.exists(video_path):
        time.sleep(2)
        video_path = _remux_to_mp4(video_path, was_usb=was_usb, usb_rec_dir=usb_rec_dir)
        dur, st = get_video_duration(video_path)
        if dur and ass_path and os.path.exists(ass_path):
            adjust_ass_timing(ass_path, dur)
        if events_path:
            try:
                evt = {
                    "ts": time.time(),
                    "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    "event": "remux_complete",
                    "detail": f"{os.path.basename(video_path)}, duration={dur:.1f}s" if dur else "remux failed",
                }
                with open(events_path, "a") as f:
                    f.write(json.dumps(evt) + "\n")
            except Exception:
                pass

    logger.info("Recording stopped")

# ── Flask Routes ─────────────────────────────────────────────────────────

@app.route("/")
def index():
    return app.send_static_file("index.html")


@app.route("/register_service")
def register_service():
    return jsonify({
        "name": "DropCam",
        "description": "Standalone drop camera recorder with servo and light control",
        "icon": "mdi-video",
        "company": "Blue Robotics",
        "version": "1.0",
        "webpage": "https://github.com/vshie/BlueOS_videorecorder",
        "api": "",
    })


@app.route("/start", methods=["GET"])
def route_start():
    if recording:
        return jsonify({"success": False, "message": "Already recording"}), 400
    try:
        ok = _start_recording_internal(mode="video", rotation=image_rotation)
        if ok:
            return jsonify({"success": True})
        return jsonify({"success": False, "message": "Failed to start recording"}), 500
    except Exception as e:
        logger.error(f"Start error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/stop", methods=["GET"])
def route_stop():
    try:
        scheduler.stop()
        hw.stop_sweep()
        hw.light_off()
        _stop_recording_internal()
        hw.led_idle()
        return jsonify({"success": True})
    except Exception as e:
        logger.error(f"Stop error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/status", methods=["GET"])
def route_status():
    global gst_process, recording, start_time
    try:
        if gst_process and gst_process.poll() is not None:
            logger.warning("GStreamer process died")
            gst_process = None
            if recording:
                recording = False
                start_time = None

        file_size_mb = 0.0
        if current_video_file and os.path.exists(current_video_file):
            file_size_mb = round(os.path.getsize(current_video_file) / (1024 * 1024), 1)

        sched = scheduler.get_state()

        if not recording:
            health = "idle"
        elif gst_error_count > 0 or (gst_process is None and current_video_file):
            health = "failed"
        elif gst_warning_count > 0 or file_stall_count > 0:
            health = "degraded"
        else:
            health = "healthy"

        resp = jsonify({
            "recording": recording,
            "start_time": start_time.isoformat() if start_time else None,
            "duration_seconds": round((datetime.now() - start_time).total_seconds(), 1) if start_time else 0,
            "file_size_mb": file_size_mb,
            "disk_free_mb": get_disk_free_mb(),
            "gst_errors": gst_error_count,
            "gst_warnings": gst_warning_count,
            "file_stalls": file_stall_count,
            "health": health,
            "stills_count": stills_count if stills_dir else 0,
            "mode": (active_recipe_for_recording or {}).get("mode", "video"),
            "scheduler": sched,
            "rotation_degrees": image_rotation,
            "recipe_name": active_recipe_for_recording["name"] if active_recipe_for_recording else None,
            "remux": {
                "active": remux_active,
                "filename": remux_filename,
                "progress": remux_progress,
                "stage": remux_stage,
            } if remux_active else None,
            "usb_storage": usb_storage.get_status(),
            "recording_to": "usb" if usb_recording else "local",
            "usb_failover_count": usb_failover_count,
        })
        resp.headers["Cache-Control"] = "no-store"
        return resp
    except Exception as e:
        logger.error(f"Status error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/list", methods=["GET"])
def list_videos():
    try:
        os.makedirs(VIDEO_DIR, exist_ok=True)
        videos = [f for f in os.listdir(VIDEO_DIR) if f.endswith((".ts", ".mp4"))]
        stills_dirs = [
            d for d in os.listdir(VIDEO_DIR)
            if d.startswith("stills_") and os.path.isdir(os.path.join(VIDEO_DIR, d))
        ]
        videos.sort(reverse=True)
        stills_dirs.sort(reverse=True)

        sessions = []
        for v in videos:
            base = os.path.splitext(v)[0]
            sidecars = []
            for ext in (".ass", "_events.ndjson"):
                s = base + ext
                if os.path.exists(os.path.join(VIDEO_DIR, s)):
                    sidecars.append(s)
            sessions.append({"video": v, "sidecars": sidecars, "location": "local"})

        # Scan USB DropCam subfolders
        usb_sessions = []
        usb_stills = []
        dropcam_dir = os.path.join(usb_storage.USB_MOUNT_POINT, usb_storage.DROPCAM_DIR)
        if usb_storage.is_mounted() and os.path.isdir(dropcam_dir):
            for folder in sorted(os.listdir(dropcam_dir), reverse=True):
                folder_path = os.path.join(dropcam_dir, folder)
                if not os.path.isdir(folder_path):
                    continue
                folder_files = os.listdir(folder_path)
                vids = [f for f in folder_files if f.endswith((".ts", ".mp4"))]
                has_session_json = "session.json" in folder_files
                if vids:
                    v = vids[0]
                    base = os.path.splitext(v)[0]
                    sidecars = []
                    for ext in (".ass", "_events.ndjson"):
                        if base + ext in folder_files:
                            sidecars.append(base + ext)
                    usb_sessions.append({
                        "video": v,
                        "sidecars": sidecars,
                        "location": "usb",
                        "usb_folder": folder,
                    })
                elif has_session_json:
                    usb_stills.append(folder)

        return jsonify({
            "videos": videos,
            "sessions": sessions,
            "stills_sessions": stills_dirs,
            "usb_sessions": usb_sessions,
            "usb_stills_sessions": usb_stills,
        })
    except Exception as e:
        logger.error(f"List error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/download/<filename>")
def download(filename):
    try:
        if filename.endswith(".zip"):
            return _download_zip(filename)
        return send_file(os.path.join(VIDEO_DIR, filename), as_attachment=True)
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


def _download_zip(zip_name):
    """Bundle a video and its sidecars (.ass, _events.ndjson) into a zip."""
    base = os.path.splitext(zip_name)[0]
    video_file = None
    for ext in (".ts", ".mp4"):
        if os.path.exists(os.path.join(VIDEO_DIR, base + ext)):
            video_file = base + ext
            break

    if not video_file:
        return jsonify({"success": False, "message": "Video not found"}), 404

    files_to_zip = [video_file]
    for ext in (".ass", "_events.ndjson"):
        sidecar = base + ext
        if os.path.exists(os.path.join(VIDEO_DIR, sidecar)):
            files_to_zip.append(sidecar)

    STORED_EXTS = {".ts", ".mp4", ".jpg", ".jpeg", ".png"}
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for fname in files_to_zip:
            ext = os.path.splitext(fname)[1].lower()
            method = zipfile.ZIP_STORED if ext in STORED_EXTS else zipfile.ZIP_DEFLATED
            zf.write(os.path.join(VIDEO_DIR, fname), fname, compress_type=method)
    data = buf.getvalue()
    return Response(
        data,
        mimetype="application/zip",
        headers={
            "Content-Disposition": f'attachment; filename="{zip_name}"',
            "Content-Length": str(len(data)),
        },
    )


@app.route("/download_stills/<dirname>")
def download_stills(dirname):
    """Bundle a stills session directory into a zip."""
    try:
        dir_path = os.path.join(VIDEO_DIR, dirname)
        if not os.path.isdir(dir_path):
            return jsonify({"success": False, "message": "Directory not found"}), 404

        IMG_EXTS = {".jpg", ".jpeg", ".png", ".gif", ".webp"}
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as zf:
            for root, _dirs, files in os.walk(dir_path):
                for f in files:
                    full = os.path.join(root, f)
                    arcname = os.path.join(dirname, os.path.relpath(full, dir_path))
                    ext = os.path.splitext(f)[1].lower()
                    method = zipfile.ZIP_STORED if ext in IMG_EXTS else zipfile.ZIP_DEFLATED
                    zf.write(full, arcname, compress_type=method)
        data = buf.getvalue()
        zip_name = dirname + ".zip"
        return Response(
            data,
            mimetype="application/zip",
            headers={
                "Content-Disposition": f'attachment; filename="{zip_name}"',
                "Content-Length": str(len(data)),
            },
        )
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/download_usb_zip/<folder>")
def download_usb_zip(folder):
    """Bundle all files in a USB DropCam session folder into a zip."""
    try:
        folder = os.path.basename(folder)
        dir_path = os.path.join(
            usb_storage.USB_MOUNT_POINT, usb_storage.DROPCAM_DIR, folder
        )
        if not os.path.isdir(dir_path):
            return jsonify({"success": False, "message": "USB folder not found"}), 404

        STORED_EXTS = {".ts", ".mp4", ".jpg", ".jpeg", ".png"}
        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w") as zf:
            for f in sorted(os.listdir(dir_path)):
                full = os.path.join(dir_path, f)
                if not os.path.isfile(full):
                    continue
                ext = os.path.splitext(f)[1].lower()
                method = zipfile.ZIP_STORED if ext in STORED_EXTS else zipfile.ZIP_DEFLATED
                zf.write(full, os.path.join(folder, f), compress_type=method)
        data = buf.getvalue()
        zip_name = folder + ".zip"
        return Response(
            data,
            mimetype="application/zip",
            headers={
                "Content-Disposition": f'attachment; filename="{zip_name}"',
                "Content-Length": str(len(data)),
            },
        )
    except Exception as e:
        logger.error(f"USB zip download error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/download_usb/<folder>/<filename>")
def download_usb_file(folder, filename):
    """Download a single file from a USB DropCam session folder."""
    try:
        folder = os.path.basename(folder)
        filename = os.path.basename(filename)
        file_path = os.path.join(
            usb_storage.USB_MOUNT_POINT, usb_storage.DROPCAM_DIR, folder, filename
        )
        if not os.path.isfile(file_path):
            return jsonify({"success": False, "message": "File not found on USB"}), 404
        return send_file(file_path, as_attachment=True)
    except Exception as e:
        logger.error(f"USB file download error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/filesize", methods=["GET"])
def get_filesize():
    try:
        path = None
        filename = None
        if recording and current_video_file and os.path.exists(current_video_file):
            path = current_video_file
            filename = os.path.basename(path)
        else:
            if os.path.exists(VIDEO_DIR):
                vids = [f for f in os.listdir(VIDEO_DIR) if f.endswith((".ts", ".mp4"))]
                vids.sort(reverse=True)
                if vids:
                    filename = vids[0]
                    path = os.path.join(VIDEO_DIR, filename)
        if path and os.path.exists(path):
            return jsonify({
                "success": True, "filename": filename,
                "size_bytes": os.path.getsize(path), "recording": recording,
            })
        return jsonify({"success": True, "filename": None, "size_bytes": 0, "recording": recording})
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/telemetry", methods=["GET"])
def route_telemetry():
    try:
        data = get_all_telemetry(
            servo_position=hw.get_servo_position(),
            light_brightness=hw.get_light_brightness(),
            recipe_name=active_recipe_for_recording["name"] if active_recipe_for_recording else None,
            recording_ok=file_stall_count == 0 if recording else None,
            usb_disk_free_mb=usb_storage.get_free_mb(),
        )
        data["success"] = True
        data["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        data["recording"] = recording
        data["light_on"] = hw.is_light_on()
        data["led_state"] = hw.get_led_state()
        return jsonify(data)
    except Exception as e:
        logger.error(f"Telemetry error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


# ── Snapshot ─────────────────────────────────────────────────────────────

@app.route("/snapshot", methods=["GET"])
def route_snapshot():
    """Capture and return a JPEG snapshot from the camera."""
    try:
        tmp = os.path.join(VIDEO_DIR, ".snapshot_tmp.jpg")
        ok = _capture_still(tmp, rotation=image_rotation)
        if ok and os.path.exists(tmp):
            return send_file(tmp, mimetype="image/jpeg")
        return jsonify({"success": False, "message": "Capture failed"}), 500
    except Exception as e:
        logger.error(f"Snapshot error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


@app.route("/rotate", methods=["POST"])
def route_rotate():
    """Cycle image rotation by 90 degrees."""
    global image_rotation
    data = request.get_json(silent=True) or {}
    if "degrees" in data:
        d = int(data["degrees"])
        if d in (0, 90, 180, 270):
            image_rotation = d
        else:
            return jsonify({"success": False, "message": "Must be 0, 90, 180, or 270"}), 400
    else:
        image_rotation = (image_rotation + 90) % 360
    cfg = load_config()
    cfg["rotation_degrees"] = image_rotation
    save_config(cfg)
    return jsonify({"success": True, "rotation_degrees": image_rotation})


# ── Servo / Light direct control ─────────────────────────────────────────

@app.route("/servo", methods=["POST"])
def route_servo():
    data = request.get_json(silent=True) or {}
    pos = data.get("position_us")
    if pos is None:
        return jsonify({"success": False, "message": "position_us required"}), 400
    hw.set_servo(int(pos))
    return jsonify({"success": True, "position_us": hw.get_servo_position()})


@app.route("/light", methods=["POST"])
def route_light():
    data = request.get_json(silent=True) or {}
    if "on" in data:
        if data["on"]:
            hw.light_on(data.get("brightness_pct"))
        else:
            hw.light_off()
    elif "brightness_pct" in data:
        hw.set_light(int(data["brightness_pct"]))
    return jsonify({
        "success": True,
        "light_on": hw.is_light_on(),
        "brightness_pct": hw.get_light_brightness(),
    })


@app.route("/led", methods=["POST"])
def route_led():
    data = request.get_json(silent=True) or {}
    r = max(0, min(255, int(data.get("r", 0))))
    g = max(0, min(255, int(data.get("g", 0))))
    b = max(0, min(255, int(data.get("b", 0))))
    mode = data.get("mode", "solid")
    if mode == "off":
        hw.led_off()
    elif mode == "breathe":
        hw.breathe_led(r, g, b, cycle_s=4.0)
    elif mode == "flash_slow":
        hw.flash_led(r, g, b, rate_hz=0.5)
    elif mode == "flash_fast":
        hw.flash_led(r, g, b, rate_hz=2.0)
    else:
        hw.set_led_color(r, g, b)
    return jsonify({"success": True, "led_state": hw.get_led_state()})


# ── Recipes API ──────────────────────────────────────────────────────────

@app.route("/recipes", methods=["GET"])
def route_recipes_list():
    return jsonify({"success": True, "recipes": list_recipes()})


@app.route("/recipes", methods=["POST"])
def route_recipes_save():
    data = request.get_json(silent=True) or {}
    recipe_id = data.pop("id", None)
    saved, errors = save_recipe(data, recipe_id)
    if errors:
        return jsonify({"success": False, "errors": errors}), 400
    return jsonify({"success": True, "recipe": saved})


@app.route("/recipes/<recipe_id>", methods=["GET"])
def route_recipe_get(recipe_id):
    r = get_recipe(recipe_id)
    if r:
        return jsonify({"success": True, "recipe": r})
    return jsonify({"success": False, "message": "Not found"}), 404


@app.route("/recipes/<recipe_id>", methods=["DELETE"])
def route_recipe_delete(recipe_id):
    ok = delete_recipe(recipe_id)
    return jsonify({"success": ok})


# ── Delete all recordings ─────────────────────────────────────────────────

@app.route("/recordings", methods=["DELETE"])
def route_delete_all_recordings():
    """Delete all video, stills, and associated sidecar files. Preserves recipes/ and config."""
    if recording:
        return jsonify({"success": False, "message": "Cannot delete while recording"}), 400
    deleted = 0
    errors = []
    try:
        for entry in os.listdir(VIDEO_DIR):
            path = os.path.join(VIDEO_DIR, entry)
            if entry == "recipes" or entry == ".snapshot_tmp.jpg":
                continue
            try:
                if os.path.isdir(path):
                    shutil.rmtree(path)
                else:
                    os.remove(path)
                deleted += 1
            except Exception as e:
                errors.append(f"{entry}: {e}")
    except Exception as e:
        return jsonify({"success": False, "message": str(e)}), 500
    return jsonify({"success": True, "deleted": deleted, "errors": errors})


# ── Active recipe / auto-start config ────────────────────────────────────

@app.route("/active_recipe", methods=["GET"])
def route_active_recipe_get():
    cfg = load_config()
    rid = cfg.get("active_recipe_id")
    recipe = get_recipe(rid) if rid else None
    return jsonify({"success": True, "active_recipe_id": rid, "recipe": recipe})


@app.route("/active_recipe", methods=["POST"])
def route_active_recipe_set():
    data = request.get_json(silent=True) or {}
    rid = data.get("recipe_id")
    cfg = load_config()
    cfg["active_recipe_id"] = rid
    save_config(cfg)
    return jsonify({"success": True, "active_recipe_id": rid})


# ── Schedule control ─────────────────────────────────────────────────────

@app.route("/schedule/start", methods=["POST"])
def route_schedule_start():
    """Start the scheduler with the active recipe or a specified recipe_id."""
    data = request.get_json(silent=True) or {}
    rid = data.get("recipe_id")
    if not rid:
        cfg = load_config()
        rid = cfg.get("active_recipe_id")
    if not rid:
        return jsonify({"success": False, "message": "No recipe selected"}), 400
    recipe = get_recipe(rid)
    if not recipe:
        return jsonify({"success": False, "message": "Recipe not found"}), 404
    scheduler.start(recipe)
    return jsonify({"success": True, "recipe": recipe["name"]})


@app.route("/schedule/stop", methods=["POST"])
def route_schedule_stop():
    scheduler.stop()
    _stop_recording_internal()
    hw.stop_sweep()
    hw.light_off()
    hw.led_idle()
    return jsonify({"success": True})


@app.route("/schedule/status", methods=["GET"])
def route_schedule_status():
    return jsonify({"success": True, **scheduler.get_state()})


# ── Startup ──────────────────────────────────────────────────────────────

CAMERA_BOOT_RETRIES = 10
CAMERA_RETRY_INTERVAL_S = 3


def _wait_for_camera():
    """Block until the camera device is available or retries are exhausted."""
    for attempt in range(1, CAMERA_BOOT_RETRIES + 1):
        if os.path.exists(VIDEO_DEVICE):
            logger.info(f"Camera {VIDEO_DEVICE} available (attempt {attempt})")
            return True
        logger.info(f"Waiting for camera {VIDEO_DEVICE} (attempt {attempt}/{CAMERA_BOOT_RETRIES})...")
        time.sleep(CAMERA_RETRY_INTERVAL_S)
    logger.warning(f"Camera {VIDEO_DEVICE} not found after {CAMERA_BOOT_RETRIES} attempts")
    return False


def _remux_orphaned_ts():
    """Remux any .ts files left over from previous runs (e.g. timeout or crash)."""
    try:
        os.makedirs(VIDEO_DIR, exist_ok=True)
        ts_files = [f for f in os.listdir(VIDEO_DIR) if f.endswith(".ts")]
        for fname in ts_files:
            mp4_name = os.path.splitext(fname)[0] + ".mp4"
            if os.path.exists(os.path.join(VIDEO_DIR, mp4_name)):
                continue
            ts_path = os.path.join(VIDEO_DIR, fname)
            logger.info(f"Found orphaned TS file, remuxing: {fname}")
            _remux_to_mp4(ts_path)
    except Exception as e:
        logger.error(f"Orphaned TS remux scan failed: {e}")


def _boot():
    """Initialize hardware, default recipes, USB storage, and auto-start if configured."""
    logger.info("=== DropCam boot sequence starting ===")
    hw.init()
    init_default_recipes()

    usb_storage.try_mount()
    usb_status = usb_storage.get_status()
    if usb_status["mounted"]:
        logger.info(f"USB storage detected: {usb_status['device']}, "
                    f"{usb_status['free_mb']:.0f} MB free, "
                    f"usable={usb_status['usable']}")
    else:
        logger.info("No USB storage detected at boot")
    usb_storage.start_probe()

    def _sweep_snapshot():
        """Capture a still during a sweep (for snapshot_only light mode)."""
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        path = os.path.join(VIDEO_DIR, f"sweep_snap_{ts}.jpg")
        _capture_still(path, rotation=image_rotation)

    scheduler.configure(
        start_fn=start_recording_with_recipe,
        stop_fn=_stop_recording_internal,
        disk_free_fn=get_disk_free_mb,
        hw=hw,
        capture_still_fn=_sweep_snapshot,
    )

    hw.led_idle()

    threading.Thread(target=_remux_orphaned_ts, daemon=True).start()

    cfg = load_config()
    rid = cfg.get("active_recipe_id")
    logger.info(f"Boot config: active_recipe_id={rid!r}, rotation={cfg.get('rotation_degrees', 0)}")

    if rid:
        recipe = get_recipe(rid)
        if recipe:
            logger.info(f"Auto-start recipe: {recipe['name']} "
                        f"(delay {recipe.get('auto_start_delay_minutes', 1)} min, "
                        f"mode={recipe.get('mode', 'video')})")
            camera_ok = _wait_for_camera()
            if not camera_ok:
                logger.warning("Proceeding with auto-start despite camera not yet detected — "
                               "scheduler delay may allow it time to appear")
            scheduler.start(recipe)
        else:
            logger.warning(f"Active recipe id '{rid}' not found on disk, skipping auto-start")
    else:
        logger.info("No active recipe configured (active_recipe_id is null), waiting for manual control")

    logger.info("=== DropCam boot sequence complete ===")


if __name__ == "__main__":
    _boot()
    app.run(host="0.0.0.0", port=5423)
