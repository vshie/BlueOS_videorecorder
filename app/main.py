"""
DropCam - Standalone BlueOS Video Recording Extension

Records H264 video from the BlueOS mavlink-camera-manager RTSP stream into
power-cut-safe fragmented .mp4 files (a self-contained moof/mdat fragment every
5 s, so a crash loses at most the final fragment and no TS→MP4 remux is needed),
or captures stills at a configurable interval.
BlueOS owns the USB camera; the extension consumes its published RTSP stream
rather than reading /dev/video* directly. Controls a camera tilt servo, lumen
light, and RGB status LED. Supports auto-start via saved recording recipes.
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
from battery import BatteryMonitor

# ── Constants ────────────────────────────────────────────────────────────
VIDEO_DIR = "/app/videorecordings"
CONFIG_FILE = os.path.join(VIDEO_DIR, "recipes", "dropcam_config.json")
_OLD_CONFIG_FILE = os.path.join(VIDEO_DIR, "dropcam_config.json")
VIDEO_DEVICE = "/dev/video2"
AUDIO_DEVICE = "hw:Camera,0"
RTSP_ENDPOINT = "rtsp://admin:blue@192.168.2.10:554/stream_0"
RADCAM_IP = "192.168.2.10"

# BlueOS mavlink-camera-manager.  The DropCam no longer reads the USB camera
# directly (BlueOS owns it); instead we consume the H264 RTSP stream that the
# camera manager publishes.  The extension runs with host networking, so the
# manager's REST API and RTSP server are reachable on localhost.
CAMERA_MANAGER_URL = "http://127.0.0.1:6020"
RTSP_HOST = "127.0.0.1"

# The H264 USB Camera plugged into the DropCam is always /dev/video2.  On
# first boot MCM publishes it as a UDP stream that the extension cannot
# consume; ensure_rtsp_stream_for_video2() swaps it to a localhost RTSP
# endpoint while preserving the user's resolution/encode/fps choices.
TARGET_CAMERA_DEVICE = "/dev/video2"
TARGET_RTSP_PATH = "video_2"   # rtsp://0.0.0.0:8554/video_2
TARGET_RTSP_NAME = "DropCam RTSP"

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
# Human-readable reason the most recent recording failed/aborted, surfaced to
# the UI.  Cleared when a new recording starts successfully.
recording_error = ""

# Watchdog cadence and how long the video file may go without growing (incl.
# stuck at 0 bytes from the start, e.g. camera never delivered a frame) before
# the recording is auto-aborted.
WATCHDOG_INTERVAL_S = 5
STALL_ABORT_INTERVALS = 6  # ~30s of no data written

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
# Rotation (degrees) captured at the moment recording started, applied to the
# finished video file as lossless display metadata so players/editors render
# it in the same orientation as the live preview.
recording_rotation = 0

usb_recording = False
usb_failover_count = 0
recording_base_dir = None
radcam_mode = False

# Last RTSP endpoint + codec discovered from the BlueOS camera manager, e.g.
# ("rtsp://127.0.0.1:8554/video_stream__dev_video3", "H264").  Refreshed on
# demand; cached so a transient API hiccup does not break an in-flight start.
blueos_rtsp_url = None
blueos_rtsp_encode = "H264"


def _rewrite_rtsp_host(url):
    """Force an RTSP endpoint to localhost.

    The camera manager advertises its LAN IP (e.g. 192.168.1.111), but the
    extension shares the host network namespace, so localhost is the most
    reliable address regardless of the unit's current IP.
    """
    try:
        from urllib.parse import urlsplit, urlunsplit
        parts = urlsplit(url)
        netloc = RTSP_HOST + (f":{parts.port}" if parts.port else "")
        return urlunsplit((parts.scheme, netloc, parts.path, parts.query, parts.fragment))
    except Exception:
        return url


def discover_blueos_stream(refresh=True):
    """Return (rtsp_url, encode) for a running BlueOS camera-manager video stream.

    Prefers an H264 stream (best for low-CPU remux), then any video stream.
    Caches the last good result in the module globals so a transient API
    hiccup mid-recording does not wipe out a known-good endpoint.  Returns
    ``(None, None)`` only when nothing has ever been discovered.
    """
    global blueos_rtsp_url, blueos_rtsp_encode
    if not refresh and blueos_rtsp_url:
        return blueos_rtsp_url, blueos_rtsp_encode
    try:
        import urllib.request
        with urllib.request.urlopen(f"{CAMERA_MANAGER_URL}/streams", timeout=5) as r:
            streams = json.loads(r.read().decode())
    except Exception as e:
        logger.warning(f"Could not query BlueOS camera manager: {e}")
        return blueos_rtsp_url, blueos_rtsp_encode

    candidates = []  # (priority, url, encode)
    for s in streams or []:
        vas = s.get("video_and_stream") or {}
        info = vas.get("stream_information") or {}
        cfg = info.get("configuration") or {}
        if cfg.get("type") != "video":
            continue
        encode = (cfg.get("encode") or "").upper()
        for ep in info.get("endpoints") or []:
            if not ep.startswith("rtsp://"):
                continue
            prio = 0 if encode == "H264" else (1 if encode == "H265" else 2)
            candidates.append((prio, _rewrite_rtsp_host(ep), encode or "H264"))

    if not candidates:
        logger.warning("BlueOS camera manager has no RTSP video stream available")
        return blueos_rtsp_url, blueos_rtsp_encode

    candidates.sort(key=lambda c: c[0])
    _, url, encode = candidates[0]
    blueos_rtsp_url, blueos_rtsp_encode = url, encode
    logger.info(f"BlueOS camera stream: {url} ({encode})")
    return url, encode


# ── MCM stream auto-configuration ────────────────────────────────────────
#
# On a fresh BlueOS install MCM picks up the USB H264 camera and exposes it
# as a UDP stream (defaults to udp://<host>:5600).  The DropCam extension
# consumes the camera over a *localhost RTSP* endpoint instead, so the user
# has historically had to manually delete the UDP stream and create an RTSP
# one via the BlueOS "Video Streams" page before the extension can record.
#
# ensure_rtsp_stream_for_video2() does that swap programmatically: it
# inspects MCM's running streams, and for any stream bound to the target
# device that does NOT already advertise an rtsp:// endpoint it removes
# the UDP-only stream and creates an RTSP one with the same encode /
# resolution / framerate / extended_configuration the user (or BlueOS
# default) had picked, just on a new endpoint URL.  Streams that already
# include an rtsp:// endpoint are left alone — even if they also have a UDP
# one — because the recorder is happy with either as long as RTSP works.
#
# The helper is safe to call repeatedly: it's a no-op once /dev/video2 is
# already RTSP, so it can run at boot, on recording-start failure, and from
# the Setup-tab "Auto-Configure" button without surprises.

_DEFAULT_EXTENDED_CONFIG = {
    "thermal": False,
    "disable_mavlink": False,
    "disable_zenoh": False,
    "disable_thumbnails": False,
    "disable_lazy": False,
}


def _mcm_get_streams():
    import urllib.request
    with urllib.request.urlopen(f"{CAMERA_MANAGER_URL}/streams", timeout=5) as r:
        return json.loads(r.read().decode())


def _mcm_delete_stream_by_name(name):
    import urllib.request, urllib.parse, urllib.error
    url = f"{CAMERA_MANAGER_URL}/delete_stream?name={urllib.parse.quote(name)}"
    req = urllib.request.Request(url, method="DELETE")
    with urllib.request.urlopen(req, timeout=8) as r:
        return r.status, r.read().decode()


def _mcm_post_stream(body):
    import urllib.request
    req = urllib.request.Request(
        f"{CAMERA_MANAGER_URL}/streams",
        data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=10) as r:
        return r.status, r.read().decode()


def _summarize_stream(s):
    """Compact representation used in /camera/ensure_rtsp responses + logs."""
    vas = s.get("video_and_stream") or {}
    src = (vas.get("video_source") or {}).get("Local") or {}
    return {
        "name": vas.get("name"),
        "device": src.get("device_path"),
        "endpoints": (vas.get("stream_information") or {}).get("endpoints", []),
        "state": s.get("state"),
    }


def ensure_rtsp_stream_for_video2():
    """Make sure MCM is publishing an RTSP stream for ``TARGET_CAMERA_DEVICE``.

    Returns a dict describing what happened so callers (boot logger, the
    /camera/ensure_rtsp route, the Setup-tab button) can surface a
    user-readable result:

      action ∈ {"already_rtsp", "swapped", "no_target_stream", "error"}
      before, after            -> list of {name, device, endpoints, state}
      message                  -> short human-readable summary
      error                    -> exception text when action == "error"
    """
    try:
        before = _mcm_get_streams()
    except Exception as e:
        logger.warning(f"ensure_rtsp: cannot reach MCM: {e}")
        return {"action": "error",
                "message": f"Camera manager not reachable: {e}",
                "error": str(e), "before": [], "after": []}

    targets = []
    for s in before or []:
        vas = s.get("video_and_stream") or {}
        src = (vas.get("video_source") or {}).get("Local") or {}
        if src.get("device_path") != TARGET_CAMERA_DEVICE:
            continue
        endpoints = (vas.get("stream_information") or {}).get("endpoints") or []
        targets.append((s, endpoints))

    if not targets:
        msg = (f"No MCM stream bound to {TARGET_CAMERA_DEVICE}; "
               "is the camera plugged in?")
        logger.info(f"ensure_rtsp: {msg}")
        return {"action": "no_target_stream", "message": msg,
                "before": [_summarize_stream(s) for s, _ in []],
                "after": [_summarize_stream(s) for s in (before or [])]}

    # If any stream for this device already advertises rtsp, we're done.
    if any(any(ep.startswith("rtsp://") for ep in eps) for _, eps in targets):
        msg = f"{TARGET_CAMERA_DEVICE} already has an RTSP stream — no change."
        logger.debug(f"ensure_rtsp: {msg}")
        return {"action": "already_rtsp", "message": msg,
                "before": [_summarize_stream(s) for s, _ in targets],
                "after":  [_summarize_stream(s) for s, _ in targets]}

    # Pick the first UDP-only stream as the template so we preserve the
    # user's encode/resolution/fps + extended_configuration if they changed
    # them.  Any remaining streams bound to the same device get deleted too
    # (MCM only allows one stream per source, so re-POSTing would 500
    # otherwise — exactly the collision we hit during probing).
    template_stream, _ = targets[0]
    template_vas = template_stream["video_and_stream"]
    template_si = template_vas["stream_information"]
    configuration = template_si.get("configuration") or {
        "type": "video", "encode": "H264",
        "height": 1080, "width": 1920,
        "frame_interval": {"numerator": 1, "denominator": 30},
    }
    extended_configuration = template_si.get("extended_configuration") or dict(_DEFAULT_EXTENDED_CONFIG)
    rtsp_endpoint = f"rtsp://0.0.0.0:8554/{TARGET_RTSP_PATH}"

    # Preserve any pre-existing rtsp endpoints if they had any (defensive — we
    # only get here when the loop above found none, but a future tweak might).
    existing_rtsp = [ep for _, eps in targets for ep in eps
                     if ep.startswith("rtsp://")]
    if rtsp_endpoint not in existing_rtsp:
        endpoints = [rtsp_endpoint] + existing_rtsp
    else:
        endpoints = existing_rtsp

    deleted = []
    try:
        for s, _ in targets:
            name = s["video_and_stream"]["name"]
            code, body = _mcm_delete_stream_by_name(name)
            logger.info(f"ensure_rtsp: deleted '{name}' (HTTP {code})")
            deleted.append(name)
        time.sleep(0.5)   # MCM frees the v4l source asynchronously
        post_body = {
            "name": TARGET_RTSP_NAME,
            "source": TARGET_CAMERA_DEVICE,
            "stream_information": {
                "endpoints": endpoints,
                "configuration": configuration,
                "extended_configuration": extended_configuration,
            },
        }
        code, body = _mcm_post_stream(post_body)
        logger.info(f"ensure_rtsp: created '{TARGET_RTSP_NAME}' "
                    f"endpoint={endpoints[0]} (HTTP {code})")
    except Exception as e:
        logger.error(f"ensure_rtsp: swap failed mid-flight: {e}", exc_info=True)
        try:
            after = _mcm_get_streams()
        except Exception:
            after = []
        return {"action": "error",
                "message": (f"Stream swap failed after deleting {deleted}: {e}. "
                            "Recreate the stream manually in BlueOS."),
                "error": str(e),
                "before": [_summarize_stream(s) for s, _ in targets],
                "after": [_summarize_stream(s) for s in (after or [])]}

    try:
        after = _mcm_get_streams()
    except Exception:
        after = []

    msg = (f"Swapped {TARGET_CAMERA_DEVICE}: removed {len(deleted)} UDP "
           f"stream(s), created '{TARGET_RTSP_NAME}' on {rtsp_endpoint}.")
    logger.info(f"ensure_rtsp: {msg}")
    # Force the next discover_blueos_stream() call to repopulate the cache.
    global blueos_rtsp_url
    blueos_rtsp_url = None
    return {"action": "swapped", "message": msg,
            "before": [_summarize_stream(s) for s, _ in targets],
            "after": [_summarize_stream(s) for s in (after or [])]}


# ── Config ───────────────────────────────────────────────────────────────

def load_config():
    defaults = {
        "active_recipe_id": None,
        "rotation_degrees": 0,
        "storage_preference": "usb",
        "radcam_focus_us": 900,
        "radcam_zoom_us": 900,
        "radcam_pan_us": 1500,
        "radcam_ext_servo_us": 1500,
        "battery": {
            "enabled": True,
            "serial_port": "auto",
            "board_number": 1,
            "baud_rate": 9600,
            "read_mode": "auto",
            "broadcast_window_s": 3.0,
            "poll_interval_s": 5.0,
            "low_voltage": 13.0,
            "clear_voltage": 13.2,
            "csv_logging_enabled": True,
            "full_charge_soc_percent": 98.0,
            "log_dir": "/app/videorecordings/battery_logs",
            "charge_state_path": "/app/videorecordings/battery_logs/charge_state.json",
        },
    }

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
storage_preference = _cfg.get("storage_preference", "usb")

# ── ASS subtitle generation (system telemetry) ──────────────────────────

def create_ass_file(video_path):
    base = os.path.splitext(video_path)[0]
    ass_path = base + ".ass"
    title = "RadCam Telemetry" if radcam_mode else "DropCam Telemetry"
    res_x = "3840" if radcam_mode else "1920"
    res_y = "2160" if radcam_mode else "1080"
    with open(ass_path, "w") as f:
        f.write("[Script Info]\n")
        f.write(f"Title: {title}\n")
        f.write("ScriptType: v4.00+\n")
        f.write("WrapStyle: 0\n")
        f.write("ScaledBorderAndShadow: yes\n")
        f.write("YCbCr Matrix: TV.601\n")
        f.write(f"PlayResX: {res_x}\n")
        f.write(f"PlayResY: {res_y}\n\n")
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
                if radcam_mode:
                    parts.append(f"Focus:{hw.get_aux_pwm('focus')}us")
                    parts.append(f"Zoom:{hw.get_aux_pwm('zoom')}us")
                try:
                    batt = battery_monitor.get_data_summary()
                except Exception:
                    batt = None
                if batt and batt.get("connected"):
                    v = batt.get("voltage_v")
                    if v is not None:
                        tag = f"Batt:{float(v):.1f}V"
                        soc = batt.get("soc_percent")
                        if soc is not None:
                            tag += f" SOC:{float(soc):.0f}%"
                        if batt.get("low_voltage_alarm"):
                            tag += " LOW"
                        parts.append(tag)
                try:
                    if hw.is_winch_active():
                        parts.append(
                            f"Winch:{hw.get_winch_state()}:{hw.get_winch_turns():+d}"
                        )
                except Exception:
                    pass
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

def _gst_startup_error(stderr_text):
    """Pick the most descriptive line from a failed gst-launch stderr blob."""
    lines = [ln.strip() for ln in (stderr_text or "").splitlines() if ln.strip()]
    for ln in lines:
        if "from element" in ln.lower() or "erroneous pipeline" in ln.lower():
            return _humanize_gst_error(ln)
    for ln in lines:
        if "error" in ln.lower():
            return _humanize_gst_error(ln)
    return "Recording failed to start (camera/encoder error)."


def _humanize_gst_error(msg):
    """Map a raw GStreamer error line to a short, operator-friendly hint."""
    low = msg.lower()
    if ("could not read from resource" in low
            or "failed to allocate a buffer" in low
            or "internal data stream error" in low):
        return ("Camera stopped delivering video. Check the USB camera "
                "connection/power (it may have disconnected).")
    if "device has been disconnected" in low or "no such device" in low:
        return ("Camera/audio device disconnected. Check the USB camera "
                "connection/power.")
    if "could not open device" in low or "no such file or directory" in low:
        return ("Camera device not found. Check the USB camera connection.")
    return msg


def gst_stderr_monitor(process):
    global gst_error_count, gst_warning_count, recording_error
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
            # Surface the first descriptive error to the UI; later lines are
            # usually generic ("streaming stopped") and less useful.
            if not recording_error and "from element" in decoded.lower():
                recording_error = _humanize_gst_error(decoded)
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


def _abort_recording(msg):
    """Abort an unhealthy recording from a background/watchdog thread.

    Records the reason (for the UI), flags the LED, and runs the stop in a
    *separate* thread.  ``_stop_recording_internal`` joins the watchdog thread,
    which would raise "cannot join current thread" if called inline from the
    watchdog itself, so it must run elsewhere.
    """
    global recording_error
    if not recording_error:
        recording_error = msg
    logger.error(msg)
    log_event("recording_aborted", msg)
    hw.led_warning()
    threading.Thread(target=_stop_recording_internal, daemon=True).start()


def recording_health_watchdog():
    global file_stall_count
    last_size = 0
    file_stall_count = 0

    while not stop_watchdog_thread and recording:
        try:
            if usb_recording and not usb_storage.is_healthy():
                _usb_failover()
                return

            # Video file growth check.  A file that never grows — including one
            # stuck at 0 bytes from the start (camera never delivered a frame) —
            # is treated as a stall and auto-aborted after STALL_ABORT_INTERVALS.
            if current_video_file:
                sz = os.path.getsize(current_video_file) if os.path.exists(current_video_file) else 0
                if sz > last_size:
                    if file_stall_count > 0 and recording:
                        hw.led_recording()
                    file_stall_count = 0
                else:
                    file_stall_count += 1
                    log_event("file_stall", f"No growth for {file_stall_count} intervals ({sz} bytes)")
                    hw.led_warning()
                    if file_stall_count >= STALL_ABORT_INTERVALS:
                        secs = file_stall_count * WATCHDOG_INTERVAL_S
                        _abort_recording(
                            f"Recording aborted: no video data written for ~{secs}s "
                            f"({sz} bytes). Check the camera/USB connection."
                        )
                        return
                last_size = sz

            if stills_dir and os.path.isdir(stills_dir):
                pass

            if gst_process and gst_process.poll() is not None:
                log_event("process_died", f"GStreamer exit code {gst_process.returncode}")
                _abort_recording(
                    f"Recording aborted: encoder process exited (code "
                    f"{gst_process.returncode}). Check the camera/USB connection."
                )
                return

            check_path = recording_base_dir or VIDEO_DIR
            disk_free = get_disk_free_mb(check_path)
            if disk_free is not None and disk_free < 1024:
                logger.warning(f"Disk space low: {disk_free} MB, stopping")
                log_event("disk_full", f"{disk_free} MB remaining")
                _abort_recording(f"Recording stopped: disk almost full ({disk_free} MB free).")
                return

            time.sleep(WATCHDOG_INTERVAL_S)
        except Exception as e:
            logger.error(f"Watchdog error: {e}")
            time.sleep(WATCHDOG_INTERVAL_S)

# ── Stills capture ───────────────────────────────────────────────────────

def _capture_still(output_path, rotation=0):
    """Capture a single JPEG frame from the active RTSP stream.

    Both RadCam and DropCam now pull from an RTSP source (DropCam via the
    BlueOS camera manager).  The manager's pipeline is lazy, so the first
    DESCRIBE after an idle period can return 503 while it warms up — we retry
    a few times before giving up.
    """
    tmp = output_path + ".tmp.jpg"
    if radcam_mode:
        url = RTSP_ENDPOINT
    else:
        url, _ = discover_blueos_stream(refresh=False)
        if not url:
            url, _ = discover_blueos_stream(refresh=True)
        if not url:
            logger.error("Still capture failed: no BlueOS camera stream available")
            return False

    cmd = [
        "ffmpeg", "-y", "-rtsp_transport", "tcp",
        "-i", url, "-frames:v", "1", "-q:v", "2", tmp,
    ]
    for attempt in range(3):
        try:
            r = subprocess.run(cmd, timeout=15, capture_output=True)
            if r.returncode == 0 and os.path.exists(tmp) and os.path.getsize(tmp) > 0:
                if rotation and rotation != 0:
                    _rotate_image(tmp, rotation)
                os.rename(tmp, output_path)
                return True
            logger.warning(
                f"Still capture attempt {attempt + 1} failed: "
                f"{r.stderr.decode(errors='replace').strip()[-200:]}"
            )
        except Exception as e:
            logger.error(f"Still capture attempt {attempt + 1} error: {e}")
        time.sleep(1)

    if os.path.exists(tmp):
        try:
            os.remove(tmp)
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
    """Public start hook for the recorder.

    Previously bracketed the inner body with preview-pipeline stop/restart so
    the server-side ffmpeg preview did not contend with the recorder for the
    camera.  The live preview is now a browser-side WebRTC consumer of the
    BlueOS camera manager (which supports multiple simultaneous consumers),
    so no server-side preview lifecycle has to be managed here.
    """
    if recording:
        return False
    return _start_recording_internal_body(
        mode=mode, still_interval_s=still_interval_s,
        rotation=rotation, force_local=force_local,
    )


def _start_recording_internal_body(mode="video", still_interval_s=1.0,
                                    rotation=0, force_local=False):
    """Inner body of _start_recording_internal — see wrapper above."""
    global gst_process, recording, start_time, current_video_file
    global current_ass_file, current_events_file
    global ass_thread, stop_ass_thread
    global gst_stderr_thread, stop_gst_stderr_thread, gst_error_count, gst_warning_count
    global watchdog_thread, stop_watchdog_thread, file_stall_count
    global stills_thread, stop_stills_thread, stills_dir, stills_count
    global usb_recording, recording_base_dir, recording_rotation
    global recording_error

    recording_error = ""
    recording_rotation = int(rotation) % 360
    os.makedirs(VIDEO_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    if active_recipe_for_recording:
        safe_name = re.sub(r'[^a-zA-Z0-9_-]', '', active_recipe_for_recording.get("name", "recipe").replace(' ', '_'))
    else:
        safe_name = None

    use_usb = (not force_local) and (storage_preference == "usb") and usb_storage.is_usable()
    if use_usb:
        subfolder = f"{safe_name}_{timestamp}" if safe_name else f"manual_{timestamp}"
        rec_dir = usb_storage.get_recording_dir(subfolder)
        usb_recording = True
        logger.info(f"Recording to USB: {rec_dir}")
    else:
        rec_dir = VIDEO_DIR
        usb_recording = False
        if storage_preference == "usb" and not force_local:
            logger.warning(
                f"USB preferred but not usable (mounted={usb_storage.is_mounted()}, "
                f"free_mb={usb_storage.get_free_mb()}); falling back to local "
                f"storage at {VIDEO_DIR}"
            )

    recording_base_dir = rec_dir

    if mode == "video":
        if use_usb:
            basename = f"{safe_name}_{timestamp}" if safe_name else f"manual_{timestamp}"
        else:
            basename = f"recipe_{safe_name}_{timestamp}" if safe_name else f"manual_{timestamp}"

        if radcam_mode:
            filename = basename + ".mp4"
            filepath = os.path.join(rec_dir, filename)
            current_video_file = filepath
            pipeline = (
                f"rtspsrc location={RTSP_ENDPOINT} "
                "protocols=tcp latency=500 retry=5 timeout=5000000 "
                "! rtph265depay ! h265parse ! "
                f"mp4mux fragment-duration=5000 ! filesink location={filepath}"
            )
        else:
            # DropCam: record the BlueOS camera-manager RTSP stream (BlueOS
            # owns the USB camera, so we no longer touch /dev/video* directly).
            url, encode = discover_blueos_stream(refresh=True)
            if not url:
                # No RTSP stream exposed.  The most common cause is that MCM
                # is publishing /dev/video2 as UDP only (default after a fresh
                # camera plug-in / BlueOS reset).  Try the auto-swap once and
                # re-discover before giving up; if MCM is unreachable or no
                # /dev/video2 stream exists the fixer is a quiet no-op.
                logger.info("No RTSP stream; attempting one-shot RTSP swap for "
                            f"{TARGET_CAMERA_DEVICE}")
                try:
                    fix = ensure_rtsp_stream_for_video2()
                    logger.info(f"Recovery RTSP swap: action={fix['action']} "
                                f"msg={fix.get('message','')}")
                    if fix["action"] == "swapped":
                        time.sleep(1.5)   # let MCM bring the RTSP pipeline up
                        url, encode = discover_blueos_stream(refresh=True)
                except Exception as e:
                    logger.warning(f"Recovery RTSP swap raised: {e}")
            if not url:
                recording_error = ("No BlueOS camera stream available. Check that "
                                   "the camera is connected and streaming in BlueOS.")
                logger.error(recording_error)
                return False
            depay = "rtph265depay ! h265parse" if encode == "H265" else "rtph264depay ! h264parse"
            filename = basename + ".mp4"
            filepath = os.path.join(rec_dir, filename)
            current_video_file = filepath
            # Record straight to a fragmented MP4 (a moof/mdat fragment every 5 s)
            # instead of MPEG-TS.  fragment-duration keeps the file power-cut-safe:
            # the moov header is written up front and every fragment is
            # self-contained, so a crash loses at most the final ~5 s fragment.
            # This yields a ready-to-play .mp4 with no slow TS→MP4 remux on stop
            # (the remux was pure USB I/O at ~9.5 MB/s, ~18 min for a 10 GiB file).
            pipeline = (
                f"rtspsrc location={url} protocols=tcp latency=200 "
                "retry=10 timeout=5000000 ! "
                f"{depay} ! queue ! mp4mux fragment-duration=5000 ! "
                f"filesink location={filepath}"
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
                err_text = err.decode(errors="replace")
                logger.error(f"GStreamer failed: {err_text}")
                recording_error = _gst_startup_error(err_text)
                gst_process = None
                start_time = None
                return False
            time.sleep(2)
            if gst_process.poll() is not None:
                out, err = gst_process.communicate()
                err_text = err.decode(errors="replace")
                logger.error(f"GStreamer died during startup: {err_text}")
                recording_error = _gst_startup_error(err_text)
                gst_process = None
                start_time = None
                return False
        except Exception as e:
            logger.error(f"Failed to start GStreamer: {e}")
            recording_error = f"Failed to start recording: {e}"
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


def _rotation_metadata_args(rotation):
    """ffmpeg args that stamp display-rotation metadata onto the video stream.

    This is lossless (used alongside ``-c copy``): no pixels are touched, only
    the MP4 display matrix is set so players/editors auto-rotate on playback,
    matching the live preview orientation.  The container's ffmpeg (4.x) writes
    the rotation via the legacy ``rotate`` stream tag, which all common players
    honour.  Returns ``[]`` for a zero rotation so nothing is written.

    The ``rotate`` tag's direction is the inverse of the preview's ffmpeg
    ``transpose`` filter: a player auto-rotating a ``rotate=90`` clip turns it
    counter-clockwise, whereas the preview's 90° uses ``transpose=1``
    (clockwise).  So we store ``(360 - deg)`` to make the recording match the
    preview.  Verified on ffmpeg 4.2 via PSNR: preview transpose=1 == rotate=270,
    hflip+vflip == rotate=180, transpose=2 == rotate=90.
    """
    deg = int(rotation) % 360
    if deg == 0:
        return []
    meta_deg = (360 - deg) % 360
    return ["-metadata:s:v:0", f"rotate={meta_deg}"]


def _run_ffmpeg_remux(ts_path, mp4_path, ts_size, rotation=0):
    """Run the ffmpeg copy-remux, tracking progress.  Returns True on success."""
    global remux_progress
    size_gib = ts_size / (1024 ** 3)
    timeout_s = int(180 + size_gib * 180)

    cmd = ["ffmpeg", "-y", "-i", ts_path, "-c", "copy"]
    cmd += _rotation_metadata_args(rotation)
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


def _apply_rotation_metadata_mp4(mp4_path, rotation):
    """Stamp display-rotation metadata onto a finished .mp4 in place (lossless).

    Used for the RadCam path, where GStreamer writes the .mp4 directly and it
    never passes through the TS→MP4 remux.  Rewrites the container with
    ``-c copy`` (no re-encode) into a temp file, then atomically replaces the
    original.  Returns the path on success, or the original path on failure /
    when no rotation is needed.
    """
    global remux_active, remux_filename, remux_progress, remux_stage
    deg = int(rotation) % 360
    if deg == 0 or not mp4_path or not os.path.exists(mp4_path):
        return mp4_path

    tmp_path = os.path.splitext(mp4_path)[0] + ".rot.mp4"
    cmd = ["ffmpeg", "-y", "-i", mp4_path, "-map", "0", "-c", "copy"]
    cmd += _rotation_metadata_args(deg)
    cmd.append(tmp_path)

    try:
        src_size = os.path.getsize(mp4_path)
    except OSError:
        src_size = 0
    size_gib = src_size / (1024 ** 3)
    timeout_s = int(180 + size_gib * 180)

    show_led = not recording
    if show_led:
        hw.led_processing()
    remux_filename = os.path.basename(mp4_path)
    remux_stage = f"Applying {deg}° rotation metadata"
    remux_progress = 0
    remux_active = True

    logger.info(f"Stamping {deg}° rotation metadata onto {os.path.basename(mp4_path)}")
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        deadline = time.monotonic() + timeout_s
        while proc.poll() is None:
            if time.monotonic() > deadline:
                proc.kill()
                proc.wait()
                raise subprocess.TimeoutExpired(cmd, timeout_s)
            if src_size > 0:
                try:
                    remux_progress = min(99, int(os.path.getsize(tmp_path) * 100 / src_size))
                except OSError:
                    pass
            time.sleep(2)
        if proc.returncode == 0 and os.path.exists(tmp_path):
            os.replace(tmp_path, mp4_path)
            remux_progress = 100
            logger.info(f"Rotation metadata applied: {os.path.basename(mp4_path)}")
            return mp4_path
        logger.error(f"Rotation metadata pass failed (rc={proc.returncode})")
    except Exception as e:
        logger.error(f"Rotation metadata pass error: {e}")
    finally:
        remux_active = False
        remux_filename = ""
        remux_progress = 0
        remux_stage = ""
        if show_led:
            hw.led_idle()
    if os.path.exists(tmp_path):
        try:
            os.remove(tmp_path)
        except OSError:
            pass
    return mp4_path


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


def _remux_to_mp4(ts_path, was_usb=False, usb_rec_dir=None, rotation=0):
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
        hw.led_processing()

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
            return _remux_usb(ts_path, ts_size, usb_rec_dir, show_led, rotation)
        else:
            return _remux_local(ts_path, ts_size, show_led, rotation)
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


def _remux_local(ts_path, ts_size, show_led, rotation=0):
    """Standard in-place remux for local SD recordings."""
    global remux_progress, remux_stage
    mp4_path = os.path.splitext(ts_path)[0] + ".mp4"
    remux_stage = "Remuxing TS→MP4"
    if _run_ffmpeg_remux(ts_path, mp4_path, ts_size, rotation):
        os.remove(ts_path)
        logger.info(f"Remuxed to MP4: {os.path.basename(mp4_path)}")
        return mp4_path
    return ts_path


def _remux_usb(ts_path, ts_size, usb_rec_dir, show_led, rotation=0):
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
        if _run_ffmpeg_remux(ts_path, mp4_on_usb, ts_size, rotation):
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
        if not _run_ffmpeg_remux(ts_path, local_tmp_mp4, ts_size, rotation):
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
    saved_rotation = recording_rotation

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
    saved_stills_dir = stills_dir

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
        if video_path.endswith(".ts"):
            # Legacy path: only hit by .ts files from older builds still finishing
            # up.  Rotation is stamped during the lossless TS→MP4 remux.
            video_path = _remux_to_mp4(video_path, was_usb=was_usb,
                                       usb_rec_dir=usb_rec_dir, rotation=saved_rotation)
        elif video_path.endswith(".mp4") and saved_rotation:
            # DropCam and RadCam both write the .mp4 directly, so stamp rotation now.
            video_path = _apply_rotation_metadata_mp4(video_path, saved_rotation)
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
    name = "RadCam" if radcam_mode else "DropCam"
    desc = ("H265 4K RTSP recorder with servo, focus, and zoom control"
            if radcam_mode
            else "Standalone drop camera recorder with servo and light control")
    return jsonify({
        "name": name,
        "description": desc,
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
            "recording_error": recording_error or None,
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
            "storage_preference": storage_preference,
            "radcam_mode": radcam_mode,
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
                    all_files = []
                    for f in sorted(folder_files):
                        fp = os.path.join(folder_path, f)
                        if os.path.isfile(fp):
                            all_files.append({"name": f, "size": os.path.getsize(fp)})
                    usb_sessions.append({
                        "video": v,
                        "sidecars": sidecars,
                        "location": "usb",
                        "usb_folder": folder,
                        "files": all_files,
                    })
                elif has_session_json:
                    stills_files = []
                    for f in sorted(folder_files):
                        fp = os.path.join(folder_path, f)
                        if os.path.isfile(fp):
                            stills_files.append({"name": f, "size": os.path.getsize(fp)})
                    usb_stills.append({"folder": folder, "files": stills_files})

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
        try:
            data["gpio_backends"] = hw.get_backend_info()
        except Exception:
            pass
        data["release_position_us"] = hw.get_release_position()
        data["release_direction"] = hw.get_release_direction()
        data["release_running"] = hw.is_release_running()
        try:
            data["rotation_sensor_available"] = hw.is_rotation_sensor_available()
            data["rotation_count"] = hw.get_rotation_count()
            data["rotation_rpm"] = round(hw.get_rotation_rpm(), 1)
        except Exception:
            pass
        try:
            from hardware import WINCH_UNWIND_RPM, WINCH_WIND_RPM
            data["winch_active"] = hw.is_winch_active()
            data["winch_state"] = hw.get_winch_state()
            data["winch_turns"] = hw.get_winch_turns()
            data["winch_unwind_rpm"] = WINCH_UNWIND_RPM
            data["winch_wind_rpm"] = WINCH_WIND_RPM
        except Exception:
            pass
        data["radcam_mode"] = radcam_mode
        if radcam_mode:
            data["aux_pwm"] = hw.get_all_aux_pwm()
        try:
            data["battery"] = battery_monitor.get_data_summary()
        except Exception as e:
            logger.debug(f"Battery summary unavailable: {e}")
            data["battery"] = {"connected": False, "last_error": str(e)}
        return jsonify(data)
    except Exception as e:
        logger.error(f"Telemetry error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


# ── Battery ──────────────────────────────────────────────────────────────

@app.route("/battery", methods=["GET"])
def route_battery():
    """Full Daly BMS snapshot (voltage, current, SOC, per-cell, errors, etc.)
    plus connection state and the rolling CSV log path."""
    try:
        return jsonify({"success": True, **battery_monitor.get_data()})
    except Exception as e:
        logger.error(f"Battery telemetry error: {e}")
        return jsonify({"success": False, "message": str(e)}), 500


# ── Camera streams (for browser WebRTC preview) ──────────────────────────


def _battery_config_getter():
    """Read the latest battery section from disk so config edits take effect
    on the next poll without an extension restart."""
    try:
        return load_config().get("battery") or {}
    except Exception as e:
        logger.warning(f"Could not load battery config: {e}")
        return {}


battery_monitor = BatteryMonitor(
    hw=hw,
    config_getter=_battery_config_getter,
    time_synced_fn=is_time_synced,
)


@app.route("/streams", methods=["GET"])
def route_streams():
    """Return the BlueOS camera-manager video streams the browser can play.

    The frontend WebRTC consumer needs the producer name / id / RTSP URL to
    match a stream in MCM's signalling channel.  This is a thin pass-through
    over ``GET {CAMERA_MANAGER_URL}/streams`` (queried via urllib — no extra
    dependency) that normalises just the fields the browser helper uses.
    """
    try:
        import urllib.request
        with urllib.request.urlopen(f"{CAMERA_MANAGER_URL}/streams", timeout=5) as r:
            raw = json.loads(r.read().decode())
    except Exception as e:
        logger.warning(f"Could not query BlueOS camera manager streams: {e}")
        return jsonify({"success": False, "message": str(e), "streams": []}), 502

    out = []
    for s in raw or []:
        try:
            sid = s.get("id")
            vas = s.get("video_and_stream") or {}
            name = vas.get("name") or "stream"
            info = vas.get("stream_information") or {}
            cfg = info.get("configuration") or {}
            if cfg.get("type") != "video":
                continue
            rtsp = None
            for ep in info.get("endpoints") or []:
                if isinstance(ep, str) and ep.lower().startswith("rtsp://"):
                    rtsp = _rewrite_rtsp_host(ep)
                    break
            if not (sid and rtsp):
                continue
            out.append({
                "stream_id": str(sid),
                "name": name,
                "rtsp_url": rtsp,
                "encode": (cfg.get("encode") or "").upper(),
                "running": bool(s.get("running")),
            })
        except Exception as parse_err:
            logger.debug(f"Skipping malformed /streams entry: {parse_err}")
            continue
    return jsonify({"success": True, "streams": out})


@app.route("/camera/ensure_rtsp", methods=["POST"])
def route_camera_ensure_rtsp():
    """One-shot fix-up: make sure MCM exposes ``TARGET_CAMERA_DEVICE`` as RTSP.

    Returns the same dict ``ensure_rtsp_stream_for_video2()`` produces.
    Always returns HTTP 200 (even for ``action == "error"``) so the UI can
    render the structured result; non-200 is reserved for actual extension
    crashes.  ``success`` is True for the no-op + happy paths and False
    when MCM is unreachable / mid-flight swap blew up.
    """
    result = ensure_rtsp_stream_for_video2()
    result["success"] = result["action"] in ("swapped", "already_rtsp")
    return jsonify(result), 200


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


@app.route("/release", methods=["GET"])
def route_release_get():
    return jsonify({
        "success": True,
        "position_us": hw.get_release_position(),
        "direction": hw.get_release_direction(),
        "running": hw.is_release_running(),
    })


@app.route("/release", methods=["POST"])
def route_release():
    """Control the release servo.

    The release uses a continuous-rotation drive: 1500 us = stop,
    1000 us = wind one direction, 2000 us = unwind the other direction.

    Body: ``{"action": "stop" | "wind" | "unwind" | "test" | "rotate"}`` or
    ``{"position_us": <int>}``.

    - ``wind`` / ``unwind`` set a sustained pulse; the caller is responsible
      for sending ``stop`` (typical pattern: press-and-hold UI button).
    - ``test`` runs a fast unwind (2000 us) until ``RELEASE_ROTATION_CAP``
      (52) sensor rotations have been counted, or ``RELEASE_MAX_DURATION_S``
      (60 s) elapses as a safety cap, then auto-returns to stop.  Same
      closed-loop behaviour the recipe trigger uses.  Calling ``test``
      again, or ``stop``, while a test is in flight will cancel it.
    - ``rotate`` runs a slow rotation-counted jog: body must include
      ``rotations`` (int >= 1) and ``direction`` ("unwind" or "wind").
      Uses the winch-calibration PWMs (WINCH_UNWIND_US / WINCH_WIND_US,
      ~52 RPM) and the same closed-loop stop logic.

    Any call here cancels a running scheduled recipe.
    """
    from hardware import (
        RELEASE_WIND_US, RELEASE_UNWIND_US, RELEASE_STOP_US,
        RELEASE_ROTATION_CAP, RELEASE_MAX_DURATION_S,
        WINCH_UNWIND_US, WINCH_WIND_US,
    )

    data = request.get_json(silent=True) or {}
    action = (data.get("action") or "").lower()

    scheduler_was_running = scheduler.is_running()
    if scheduler_was_running:
        logger.info("Release control invoked while recipe running — cancelling schedule")
        scheduler.stop()
        try:
            _stop_recording_internal()
        except Exception as e:
            logger.error(f"Stop recording during release control failed: {e}")
        hw.stop_sweep()
        hw.light_off()
        hw.led_idle()

    if action == "stop" or action == "off" or action == "reset":
        hw.release_stop()
    elif action == "wind":
        hw.release_wind()
    elif action == "unwind":
        hw.release_unwind()
    elif action == "test":
        # Calling /release with action=test starts a fresh closed-loop
        # 52-rotation fast unwind (cancels any prior in-flight run).  Use
        # action=stop to abort.  Same path the recipe-finish release uses.
        hw.release_run_for_rotations(
            RELEASE_UNWIND_US, RELEASE_ROTATION_CAP, RELEASE_MAX_DURATION_S,
        )
    elif action == "rotate":
        # Slow rotation-counted jog from the main page.  Closed-loop on
        # the rotation sensor; safety-capped at RELEASE_MAX_DURATION_S so
        # a stalled or missing sensor cannot hold the servo indefinitely.
        try:
            rotations = int(data.get("rotations", 0))
        except (TypeError, ValueError):
            return jsonify({"success": False,
                            "message": "rotations must be an integer"}), 400
        direction = (data.get("direction") or "").lower()
        if rotations <= 0:
            return jsonify({"success": False,
                            "message": "rotations must be >= 1"}), 400
        if direction == "unwind":
            jog_pwm = WINCH_UNWIND_US
        elif direction == "wind":
            jog_pwm = WINCH_WIND_US
        else:
            return jsonify({"success": False,
                            "message": "direction must be 'unwind' or 'wind'"}), 400
        hw.release_run_for_rotations(
            jog_pwm, rotations, RELEASE_MAX_DURATION_S,
        )
    elif "position_us" in data:
        # Direct low-level set (cancels any timed run).
        try:
            hw.release_stop()  # cancel timed run if any, then set explicit value
        except Exception:
            pass
        hw.set_release(int(data["position_us"]))
    else:
        return jsonify({
            "success": False,
            "message": "Provide action=stop|wind|unwind|test or position_us",
        }), 400

    return jsonify({
        "success": True,
        "position_us": hw.get_release_position(),
        "direction": hw.get_release_direction(),
        "running": hw.is_release_running(),
        "schedule_cancelled": scheduler_was_running,
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


# ── Auxiliary PWM control ─────────────────────────────────────────────

@app.route("/aux_pwm", methods=["GET"])
def route_aux_pwm_get():
    return jsonify({"success": True, "aux_pwm": hw.get_all_aux_pwm()})


@app.route("/aux_pwm", methods=["POST"])
def route_aux_pwm_set():
    data = request.get_json(silent=True) or {}
    channel = data.get("channel")
    position_us = data.get("position_us")
    if not channel or position_us is None:
        return jsonify({"success": False, "message": "channel and position_us required"}), 400
    try:
        hw.set_aux_pwm(channel, int(position_us))
    except ValueError as e:
        return jsonify({"success": False, "message": str(e)}), 400
    cfg = load_config()
    config_key = f"radcam_{channel}_us"
    cfg[config_key] = int(position_us)
    save_config(cfg)
    return jsonify({"success": True, "channel": channel, "position_us": hw.get_aux_pwm(channel)})


# ── RadCam detection (manual) ────────────────────────────────────────────

@app.route("/detect_radcam", methods=["POST"])
def route_detect_radcam():
    global radcam_mode
    if radcam_mode:
        return jsonify({"success": True, "message": "Already in RadCam mode"})
    if not _ping_radcam():
        return jsonify({"success": False, "message": f"RadCam not reachable at {RADCAM_IP}"}), 404
    radcam_mode = True
    logger.info(f"Manual RadCam detection succeeded — switching to RadCam mode")
    cfg = load_config()
    hw.set_aux_pwm("focus", cfg.get("radcam_focus_us", 900))
    hw.set_aux_pwm("zoom", cfg.get("radcam_zoom_us", 900))
    hw.set_aux_pwm("pan", cfg.get("radcam_pan_us", 1500))
    hw.set_aux_pwm("ext_servo", cfg.get("radcam_ext_servo_us", 1500))
    init_default_recipes(radcam=True)
    register_service()
    return jsonify({"success": True, "message": "RadCam detected, mode switched"})


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


# ── Delete recordings ─────────────────────────────────────────────────────

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


@app.route("/recordings/selected", methods=["DELETE"])
def route_delete_selected_recordings():
    """Delete a specific set of recordings identified by location and id."""
    if recording:
        return jsonify({"success": False, "message": "Cannot delete while recording"}), 400
    data = request.get_json(silent=True) or {}
    items = data.get("items", [])
    deleted = 0
    errors = []
    dropcam_dir = os.path.join(usb_storage.USB_MOUNT_POINT, usb_storage.DROPCAM_DIR)
    for item in items:
        location = item.get("location")
        item_id = item.get("id", "")
        try:
            if location == "usb":
                path = os.path.join(dropcam_dir, item_id)
                if os.path.isdir(path):
                    shutil.rmtree(path)
                    deleted += 1
                else:
                    errors.append(f"USB folder not found: {item_id}")
            elif location == "local":
                stills_path = os.path.join(VIDEO_DIR, item_id)
                if os.path.isdir(stills_path):
                    shutil.rmtree(stills_path)
                    deleted += 1
                else:
                    found = False
                    for ext in (".ts", ".mp4"):
                        vpath = os.path.join(VIDEO_DIR, item_id + ext)
                        if os.path.exists(vpath):
                            os.remove(vpath)
                            found = True
                    for ext in (".ass", "_events.ndjson"):
                        spath = os.path.join(VIDEO_DIR, item_id + ext)
                        if os.path.exists(spath):
                            os.remove(spath)
                    if found:
                        deleted += 1
                    else:
                        errors.append(f"Not found: {item_id}")
            else:
                errors.append(f"Unknown location for item: {item_id}")
        except Exception as e:
            errors.append(f"{item_id}: {e}")
    return jsonify({"success": True, "deleted": deleted, "errors": errors})


# ── Process (remux) recordings ───────────────────────────────────────────

@app.route("/recordings/process", methods=["POST"])
def route_process_selected():
    """Remux selected .ts recordings to .mp4 in a background thread."""
    if recording:
        return jsonify({"success": False, "message": "Cannot process while recording"}), 400
    if remux_active:
        return jsonify({"success": False, "message": "Processing already in progress"}), 400

    data = request.get_json(silent=True) or {}
    items = data.get("items", [])
    ts_jobs = []
    dropcam_dir = os.path.join(usb_storage.USB_MOUNT_POINT, usb_storage.DROPCAM_DIR)

    for item in items:
        location = item.get("location")
        item_id = item.get("id", "")
        if location == "local":
            ts_path = os.path.join(VIDEO_DIR, item_id + ".ts")
            if os.path.exists(ts_path):
                ts_jobs.append({"path": ts_path, "was_usb": False, "usb_rec_dir": None})
        elif location == "usb":
            folder_path = os.path.join(dropcam_dir, item_id)
            if os.path.isdir(folder_path):
                for f in os.listdir(folder_path):
                    if f.endswith(".ts"):
                        ts_jobs.append({
                            "path": os.path.join(folder_path, f),
                            "was_usb": True,
                            "usb_rec_dir": folder_path,
                        })

    if not ts_jobs:
        return jsonify({"success": False, "message": "No unprocessed .ts files found"}), 404

    def _process_batch():
        for job in ts_jobs:
            mp4_path = _remux_to_mp4(job["path"], was_usb=job["was_usb"],
                                     usb_rec_dir=job["usb_rec_dir"])
            ass_path = os.path.splitext(job["path"])[0] + ".ass"
            if os.path.exists(ass_path):
                dur, _ = get_video_duration(mp4_path)
                if dur:
                    adjust_ass_timing(ass_path, dur)

    threading.Thread(target=_process_batch, daemon=True).start()
    return jsonify({"success": True, "count": len(ts_jobs)})


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


@app.route("/storage_preference", methods=["POST"])
def route_storage_preference_set():
    global storage_preference
    data = request.get_json(silent=True) or {}
    pref = data.get("preference")
    if pref not in ("usb", "local"):
        return jsonify({"success": False, "message": "preference must be 'usb' or 'local'"}), 400
    storage_preference = pref
    cfg = load_config()
    cfg["storage_preference"] = pref
    save_config(cfg)
    logger.info(f"Storage preference set to: {pref}")
    return jsonify({"success": True, "storage_preference": pref})


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


def _ping_radcam():
    """Try to reach the RadCam at 192.168.2.10. Returns True if reachable."""
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "2", RADCAM_IP],
            capture_output=True, timeout=5,
        )
        return result.returncode == 0
    except Exception as e:
        logger.debug(f"RadCam ping failed: {e}")
        return False


def _wait_for_camera():
    """Block until a camera source is available.

    Pings the RadCam first (fast); otherwise waits for the BlueOS camera
    manager to publish an RTSP video stream for the USB camera.
    """
    global radcam_mode

    logger.info(f"Checking for RadCam at {RADCAM_IP}...")
    if _ping_radcam():
        radcam_mode = True
        logger.info(f"RadCam detected at {RADCAM_IP} — entering RadCam mode (H265 4K RTSP)")
        return True

    radcam_mode = False
    # Auto-fix the common first-boot case: MCM picked up the USB H264 camera
    # and exposed it as UDP, which the extension cannot consume.  This is
    # idempotent — once /dev/video2 is RTSP it short-circuits to a no-op,
    # so it costs ~1 HTTP roundtrip on every subsequent boot.
    try:
        result = ensure_rtsp_stream_for_video2()
        if result["action"] == "swapped":
            logger.info(f"Boot RTSP swap: {result['message']}")
        elif result["action"] == "error":
            logger.warning(f"Boot RTSP swap could not complete: {result['message']}")
    except Exception as e:
        logger.warning(f"Boot RTSP swap raised: {e}", exc_info=True)

    for attempt in range(1, CAMERA_BOOT_RETRIES + 1):
        url, encode = discover_blueos_stream(refresh=True)
        if url:
            logger.info(f"BlueOS camera stream available: {url} ({encode}) (attempt {attempt})")
            return True
        logger.info(f"Waiting for BlueOS camera stream (attempt {attempt}/{CAMERA_BOOT_RETRIES})...")
        time.sleep(CAMERA_RETRY_INTERVAL_S)

    logger.warning("No camera source found (BlueOS RTSP or RadCam)")
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
    try:
        battery_monitor.start()
    except Exception as e:
        logger.warning(f"Battery monitor failed to start: {e}")
    init_default_recipes()

    # Mount USB in the background only.  A failing/unresponsive USB stick can
    # leave mount.ntfs wedged in uninterruptible (D) sleep, which would hang a
    # synchronous mount here forever and prevent the web server from ever
    # starting.  The probe thread owns the (re)mount; until it succeeds,
    # recordings transparently fall back to local storage.
    try:
        usb_storage.start_probe()
        logger.info("USB mount deferred to background probe thread; "
                    "recordings fall back to local storage until USB is ready")
    except Exception as e:
        logger.warning(f"Could not start USB probe thread: {e}")

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
        log_event_fn=log_event,
    )

    hw.led_idle()

    threading.Thread(target=_remux_orphaned_ts, daemon=True).start()

    cfg = load_config()
    global storage_preference
    storage_preference = cfg.get("storage_preference", "usb")

    rid = cfg.get("active_recipe_id")
    logger.info(f"Boot config: active_recipe_id={rid!r}, rotation={cfg.get('rotation_degrees', 0)}, "
                f"storage_preference={storage_preference!r}")

    camera_ok = _wait_for_camera()

    if radcam_mode:
        logger.info("Applying saved RadCam PWM settings from config")
        hw.set_aux_pwm("focus", cfg.get("radcam_focus_us", 900))
        hw.set_aux_pwm("zoom", cfg.get("radcam_zoom_us", 900))
        hw.set_aux_pwm("pan", cfg.get("radcam_pan_us", 1500))
        hw.set_aux_pwm("ext_servo", cfg.get("radcam_ext_servo_us", 1500))
        init_default_recipes(radcam=True)

    # Release-shaft rotation sensor lives on GPIO 26 (shared with the RadCam
    # zoom aux output), so only set it up in DropCam mode.  It's optional —
    # if pigpiod / the sensor isn't available, init returns False and the
    # rest of the system keeps running with rotation_count=0.
    try:
        if not radcam_mode:
            hw.init_rotation_sensor(enable=True)
    except Exception as e:
        logger.warning(f"Rotation sensor init failed: {e}")

    # The live preview is served by the BlueOS camera manager directly to the
    # browser over WebRTC (signalling on :6021); nothing to start here.  If
    # the camera is not yet available the WebRTC client will simply retry
    # once MCM publishes the producer.
    if not camera_ok:
        logger.info("Camera not yet available — WebRTC preview will appear once MCM exposes the stream")

    if rid:
        recipe = get_recipe(rid)
        if recipe:
            logger.info(f"Auto-start recipe: {recipe['name']} "
                        f"(delay {recipe.get('auto_start_delay_minutes', 1)} min, "
                        f"mode={recipe.get('mode', 'video')})")
            if not camera_ok:
                logger.warning("Proceeding with auto-start despite camera not yet detected — "
                               "scheduler delay may allow it time to appear")
            scheduler.start(recipe)
        else:
            logger.warning(f"Active recipe id '{rid}' not found on disk, skipping auto-start")
    else:
        logger.info("No active recipe configured (active_recipe_id is null), waiting for manual control")

    mode_label = "RadCam" if radcam_mode else "DropCam"
    logger.info(f"=== {mode_label} boot sequence complete ===")

    import atexit
    atexit.register(_shutdown_safely)


def _shutdown_safely():
    try:
        battery_monitor.stop()
    except Exception:
        pass


if __name__ == "__main__":
    _boot()
    app.run(host="0.0.0.0", port=5423)
