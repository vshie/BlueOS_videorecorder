"""
DropCam - Standalone BlueOS Video Recording Extension

Records H264 video from /dev/video2 into power-cut-safe .ts (MPEG-TS) files,
or captures stills at a configurable interval. Controls a camera tilt servo,
lumen light, and RGB status LED. Supports auto-start via saved recording recipes.
"""

from flask import Flask, jsonify, request, send_file
import io
import json
import os
import re
import subprocess
import shlex
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
    is_time_synced, get_disk_free_mb, get_all_telemetry,
)
from recipes import (
    init_default_recipes, list_recipes, get_recipe,
    save_recipe, delete_recipe, calculate_sweep_time,
)

# ── Constants ────────────────────────────────────────────────────────────
VIDEO_DIR = "/app/videorecordings"
CONFIG_FILE = os.path.join(VIDEO_DIR, "dropcam_config.json")
VIDEO_DEVICE = "/dev/video2"

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

active_recipe_for_recording = None
image_rotation = 0

# ── Config ───────────────────────────────────────────────────────────────

def load_config():
    defaults = {"active_recipe_id": None, "rotation_degrees": 0}
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, "r") as f:
                saved = json.load(f)
            defaults.update(saved)
    except Exception as e:
        logger.warning(f"Could not load config: {e}")
    return defaults


def save_config(cfg):
    try:
        os.makedirs(os.path.dirname(CONFIG_FILE), exist_ok=True)
        with open(CONFIG_FILE, "w") as f:
            json.dump(cfg, f, indent=2)
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
        f.write("Style: Telem,Arial,16,&H00FFFFFF,&H000000FF,&H00000000,"
                "&H00000000,0,0,0,0,100,100,0,0,1,2,1,7,10,10,10,1\n\n")
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
                text = " | ".join(parts)

                line = f"Dialogue: 0,{t0},{t1},Telem,,0,0,0,,{text}\n"
                with open(current_ass_file, "a") as f:
                    f.write(line)

            time.sleep(1 / rate)
        except Exception as e:
            logger.error(f"ASS subtitle error: {e}")
            time.sleep(1)


def adjust_ass_timing(ass_path, video_duration):
    """Scale ASS dialogue timestamps so they span exactly video_duration.
    Timestamps stay 0-based (players normalise the video timeline to start at 0)."""
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
        scale = video_duration / max_t
        if abs(scale - 1.0) < 0.005:
            return
        with open(ass_path, "w") as f:
            for line in header:
                f.write(line)
            for line in dialogues:
                parts = line.split(",", 9)
                if len(parts) >= 3:
                    parts[1] = format_ass_ts(parse_ass_ts(parts[1]) * scale)
                    parts[2] = format_ass_ts(parse_ass_ts(parts[2]) * scale)
                f.write(",".join(parts))
        logger.info(f"ASS timing scaled by {scale:.4f}")
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

def recording_health_watchdog():
    global file_stall_count
    last_size = 0
    file_stall_count = 0

    while not stop_watchdog_thread and recording:
        try:
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

            disk_free = get_disk_free_mb()
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
        fname = f"frame_{stills_count:06d}.jpg"
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


def _start_recording_internal(mode="video", still_interval_s=1.0, rotation=0):
    global gst_process, recording, start_time, current_video_file
    global current_ass_file, current_events_file
    global ass_thread, stop_ass_thread
    global gst_stderr_thread, stop_gst_stderr_thread, gst_error_count, gst_warning_count
    global watchdog_thread, stop_watchdog_thread, file_stall_count
    global stills_thread, stop_stills_thread, stills_dir, stills_count

    if recording:
        return False

    os.makedirs(VIDEO_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if mode == "video":
        if active_recipe_for_recording:
            safe_name = re.sub(r'[^a-zA-Z0-9_-]', '', active_recipe_for_recording.get("name", "recipe").replace(' ', '_'))
            filename = f"recipe_{safe_name}_{timestamp}.ts"
        else:
            filename = f"manual_{timestamp}.ts"
        filepath = os.path.join(VIDEO_DIR, filename)
        current_video_file = filepath

        pipeline = (
            f"v4l2src do-timestamp=true device={VIDEO_DEVICE} ! "
            "video/x-h264,width=1920,height=1080,framerate=30/1 ! "
            f"h264parse ! mpegtsmux ! filesink location={filepath}"
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
        if active_recipe_for_recording:
            safe_name = re.sub(r'[^a-zA-Z0-9_-]', '', active_recipe_for_recording.get("name", "recipe").replace(' ', '_'))
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

        stop_stills_thread = False
        stills_thread = threading.Thread(
            target=stills_capture_loop, args=(still_interval_s, rotation), daemon=True
        )
        stills_thread.start()
        start_time = datetime.now()
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
    log_event("recording_started", f"mode={mode}")
    logger.info(f"Recording started: mode={mode}")
    return True


def _stop_recording_internal():
    global gst_process, recording, start_time, current_video_file
    global current_ass_file, current_events_file
    global ass_thread, stop_ass_thread
    global gst_stderr_thread, stop_gst_stderr_thread
    global watchdog_thread, stop_watchdog_thread
    global stills_thread, stop_stills_thread, stills_dir
    global active_recipe_for_recording

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

    recording = False
    start_time = None
    gst_process = None
    current_video_file = None
    current_ass_file = None
    current_events_file = None
    stills_dir = None
    active_recipe_for_recording = None

    if video_path and ass_path and os.path.exists(video_path) and os.path.exists(ass_path):
        time.sleep(2)
        dur, st = get_video_duration(video_path)
        if dur:
            adjust_ass_timing(ass_path, dur)
            if st > 1.0:
                logger.warning(f"Video PTS offset: start_time={st:.2f}s (expected ~0)")
                if events_path:
                    try:
                        evt = {
                            "ts": time.time(),
                            "time": datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                            "event": "pts_warning",
                            "detail": f"Video start_time={st:.2f}s, expected ~0",
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
            sessions.append({"video": v, "sidecars": sidecars})

        return jsonify({"videos": videos, "sessions": sessions, "stills_sessions": stills_dirs})
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
    candidates = []
    for ext in (".ts", ".mp4"):
        vpath = os.path.join(VIDEO_DIR, base + ext)
        if os.path.exists(vpath):
            candidates.append(base + ext)
            break

    if not candidates:
        return jsonify({"success": False, "message": "Video not found"}), 404

    video_base = os.path.splitext(candidates[0])[0]
    files_to_zip = [candidates[0]]
    for ext in (".ass", "_events.ndjson"):
        sidecar = video_base + ext
        if os.path.exists(os.path.join(VIDEO_DIR, sidecar)):
            files_to_zip.append(sidecar)

    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
        for fname in files_to_zip:
            zf.write(os.path.join(VIDEO_DIR, fname), fname)
    buf.seek(0)
    return send_file(buf, mimetype="application/zip", as_attachment=True, download_name=zip_name)


@app.route("/download_stills/<dirname>")
def download_stills(dirname):
    """Bundle a stills session directory into a zip."""
    try:
        dir_path = os.path.join(VIDEO_DIR, dirname)
        if not os.path.isdir(dir_path):
            return jsonify({"success": False, "message": "Directory not found"}), 404

        buf = io.BytesIO()
        with zipfile.ZipFile(buf, "w", zipfile.ZIP_DEFLATED) as zf:
            for root, _dirs, files in os.walk(dir_path):
                for f in files:
                    full = os.path.join(root, f)
                    arcname = os.path.join(dirname, os.path.relpath(full, dir_path))
                    zf.write(full, arcname)
        buf.seek(0)
        return send_file(buf, mimetype="application/zip", as_attachment=True,
                         download_name=dirname + ".zip")
    except Exception as e:
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

def _boot():
    """Initialize hardware, default recipes, and auto-start if configured."""
    hw.init()
    init_default_recipes()

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

    cfg = load_config()
    rid = cfg.get("active_recipe_id")
    if rid:
        recipe = get_recipe(rid)
        if recipe:
            logger.info(f"Auto-start recipe: {recipe['name']} (delay {recipe.get('auto_start_delay_minutes', 1)} min)")
            scheduler.start(recipe)
        else:
            logger.info(f"Active recipe id '{rid}' not found, skipping auto-start")
    else:
        logger.info("No active recipe configured, waiting for manual control")


if __name__ == "__main__":
    _boot()
    app.run(host="0.0.0.0", port=5423)
