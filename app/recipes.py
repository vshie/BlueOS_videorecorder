"""
Recipe system for DropCam recording plans.

Recipes are JSON files stored in the recordings directory under a recipes/ subfolder.
Each recipe defines capture mode, duration, servo movement, and light settings.
"""

import json
import logging
import os
import re
import uuid
from datetime import datetime, timedelta

from dateutil import tz

logger = logging.getLogger(__name__)

RECIPES_DIR = "/app/videorecordings/recipes"

# Max recipe recording length (minutes). Raised from 8h while battery/runtime limits are TBD.
RECIPE_MAX_DURATION_MINUTES = 32 * 60


def parse_duration_to_minutes(value):
    """Interpret user/API duration as minutes.

    Accepts int/float (minutes), or strings such as:
    ``120``, ``90 min``, ``4.65 hours``, ``2h``, ``30m``.
    """
    if isinstance(value, bool):
        raise ValueError("invalid duration")
    if isinstance(value, (int, float)):
        return float(value)
    s = str(value).strip().lower()
    if not s:
        raise ValueError("empty duration")
    m = re.match(r"^(\d+(?:\.\d+)?)\s*(?:hours?|hrs?|h)\s*$", s)
    if m:
        return float(m.group(1)) * 60
    m = re.match(r"^(\d+(?:\.\d+)?)\s*(?:minutes?|mins?)\s*$", s)
    if m:
        return float(m.group(1))
    m = re.match(r"^(\d+(?:\.\d+)?)\s*m\s*$", s)
    if m:
        return float(m.group(1))
    m = re.match(r"^(\d+(?:\.\d+)?)\s*$", s)
    if m:
        return float(m.group(1))
    raise ValueError("unrecognized duration format")


RECIPE_SCHEMA_DEFAULTS = {
    "name": "Untitled",
    "mode": "video",
    "still_interval_s": 1.0,
    "duration_minutes": 30.0,
    "auto_start_delay_minutes": 1,
    "rotation_degrees": 0,
    "servo_start_us": 1500,
    "servo_end_us": 1500,
    "servo_pause_points": 0,
    "servo_loiter_time_s": 0,
    "servo_oscillations": 1,
    "servo_fixed": True,
    "light_brightness_pct": 0,
    "light_mode": "off",
    "led_color": "red",
    "led_blink": "slow",
    "still_prefix": "",
    "cam4k_focus_finder": False,
    "focus_sweep_start_us": 870,
    "focus_sweep_end_us": 2130,
    "focus_finder_zoom_us": 935,
    "cam4k_focus_us": 900,
    "cam4k_zoom_us": 935,
    # One-push AWB via 4k Cam setImageAdjustmentEx (same path as
    # 4k Cam Manager / towfish). Interval 0 with enable=False is a no-op.
    "cam4k_awb_enable": True,
    "cam4k_awb_interval_s": 120,
    "release_enable": False,
    "release_offset_s": 0,
    # Winch vertical profiling (oscillates the release servo during the
    # recording window).  See app/hardware.py WINCH_* constants for the
    # PWMs and target RPM.  winch_rotations is per-leg; total motion budget
    # is winch_profiles * 2 * leg_time.
    "winch_enable": False,
    "winch_rotations": 10,
    "winch_profiles": 1,
    "winch_start_delay_minutes": 0,
    # Record / idle duty cycle.  When enabled, duration_minutes is the total
    # deployment length and the recorder repeats "record for
    # cycle_record_minutes, then sit idle for cycle_idle_minutes" until it
    # runs out.  The Lumen light (light_mode "always") is on only while
    # recording.
    "cycle_enable": False,
    "cycle_record_minutes": 10.0,
    "cycle_idle_minutes": 50.0,
    # Open-ended deployment: ignore duration_minutes and keep going until
    # Stop, disk full, or the battery dies (e.g. a whale-fall timelapse).
    "run_until_stopped": False,
    # Night schedule.  Between night_start and night_end (local time in
    # time_zone; may wrap midnight) the recorder repeats "record
    # night_record_minutes with the light at night_light_brightness_pct,
    # then night_idle_minutes off".  Outside it, the day settings above
    # apply (continuous, or the record/idle cycle).
    "night_enable": False,
    "night_start": "19:00",
    "night_end": "06:00",
    "night_record_minutes": 5.0,
    "night_idle_minutes": 10.0,
    "night_light_brightness_pct": 80,
    # With one-push AWB on: at night fire it once, shortly after the light
    # first comes on, then not again until the day phase resumes the interval.
    "night_awb_once": False,
    "time_zone": "UTC",
}

# Shortest record or idle window a cycle may use (minutes).
CYCLE_MIN_MINUTES = 0.5
# Light modes that make sense across idle gaps; the others follow sweep
# pause points, which a duty cycle doesn't line up with.
CYCLE_LIGHT_MODES = ("off", "always")

# Continuous video, and record windows longer than this, are split into
# clips at the top of each local hour so a long run isn't one huge file.
# Recordings planned for this long or less are never split.
HOURLY_SPLIT_MINUTES = 60.0
# A recording that starts this close to the top of the hour runs its first
# clip on to the following hour instead of making a clip of a few seconds.
FIRST_CLIP_MIN_S = 120.0
# A record segment shorter than this (cut by a phase boundary) is skipped
# rather than starting a clip that would stop again immediately.
MIN_CLIP_S = 5.0

_HHMM_RE = re.compile(r"^([01]?\d|2[0-3]):([0-5]\d)$")

# Allowed window for release-trigger offset relative to recording finish.
# Negative = fire before the recording ends, positive = fire after.
RELEASE_OFFSET_MIN_S = -3600
RELEASE_OFFSET_MAX_S = 3600


def calculate_sweep_time(duration_minutes, pause_points, loiter_time_s, oscillations):
    """Auto-calculate sweep_time_s per leg so that oscillations fit the duration.

    Returns dict with sweep_time_s, time_per_oscillation_s, and valid flag.
    Each leg has pause_points intermediate loiters + 1 extent destination loiter.
    One oscillation = 2 legs.
    """
    duration_s = duration_minutes * 60
    oscillations = max(oscillations, 1)
    loiters_per_oscillation = 2 * (pause_points + 1)
    total_loiter_s = oscillations * loiters_per_oscillation * loiter_time_s
    remaining_transit_s = duration_s - total_loiter_s
    if remaining_transit_s <= 0:
        return {
            "sweep_time_s": 0,
            "time_per_oscillation_s": total_loiter_s / oscillations,
            "valid": False,
        }
    sweep_time_s = remaining_transit_s / (2 * oscillations)
    time_per_osc = 2 * sweep_time_s + loiters_per_oscillation * loiter_time_s
    return {
        "sweep_time_s": round(sweep_time_s, 1),
        "time_per_oscillation_s": round(time_per_osc, 1),
        "valid": True,
    }

def calculate_cycle_plan(duration_minutes, record_minutes, idle_minutes):
    """Lay out record windows across a deployment.

    Windows start every record + idle minutes until duration_minutes runs
    out; the last window is cut short if the deadline lands inside it.
    Returns dict with windows, record_total_minutes and last_window_minutes.
    """
    duration = max(float(duration_minutes), 0.0)
    # Match the scheduler loop exactly; the floor only guards against a
    # zero-length period (validation enforces CYCLE_MIN_MINUTES).
    record = max(float(record_minutes), 0.001)
    period = record + max(float(idle_minutes), 0.001)
    windows = 0
    record_total = 0.0
    last = 0.0
    start = 0.0
    while start < duration:
        last = min(record, duration - start)
        windows += 1
        record_total += last
        start += period
    return {
        "windows": windows,
        "record_total_minutes": round(record_total, 2),
        "last_window_minutes": round(last, 2),
    }


def uses_schedule(recipe):
    """True when the scheduler should run the wall-clock schedule loop."""
    return bool(recipe.get("night_enable") or recipe.get("run_until_stopped"))


def recipe_zone(recipe):
    return tz.gettz(str(recipe.get("time_zone") or "UTC")) or tz.UTC


def _hhmm(value):
    m = _HHMM_RE.match(str(value).strip())
    if not m:
        raise ValueError(value)
    return int(m.group(1)), int(m.group(2))


def night_window_at(now_utc, recipe):
    """Return (is_night, next_boundary_utc) for an aware UTC datetime.

    The night runs from night_start to night_end in the recipe's time
    zone, wrapping past midnight when night_end <= night_start.
    """
    zone = recipe_zone(recipe)
    sh, sm = _hhmm(recipe.get("night_start", "19:00"))
    eh, em = _hhmm(recipe.get("night_end", "06:00"))
    wraps = (eh, em) <= (sh, sm)
    today = now_utc.astimezone(zone).date()
    nights = []
    for offset in (-1, 0, 1, 2):
        d = today + timedelta(days=offset)
        start = datetime(d.year, d.month, d.day, sh, sm, tzinfo=zone)
        e = d + timedelta(days=1) if wraps else d
        end = datetime(e.year, e.month, e.day, eh, em, tzinfo=zone)
        nights.append((start, end))
    for start, end in nights:
        if start <= now_utc < end:
            return True, end.astimezone(tz.UTC)
    nxt = min(start for start, _ in nights if start > now_utc)
    return False, nxt.astimezone(tz.UTC)


def schedule_owns_day_light(recipe):
    """False when the day light belongs to someone else: the operator
    (manual sessions) or a tilt sweep, which switches it at pause points."""
    return not (recipe.get("_manual")
                or not recipe.get("servo_fixed", True)
                or recipe.get("light_mode", "off") not in CYCLE_LIGHT_MODES)


def phase_settings(recipe, phase):
    """(record_minutes, idle_minutes, light_pct) for 'day' or 'night'.

    record_minutes is None for continuous recording (idle 0).  light_pct
    is None when the schedule should leave the light alone.
    """
    if phase == "night":
        rec = float(recipe.get("night_record_minutes", 5))
        idle = float(recipe.get("night_idle_minutes", 10))
        light = int(recipe.get("night_light_brightness_pct", 80))
    else:
        if not schedule_owns_day_light(recipe):
            light = None
        elif recipe.get("light_mode", "off") == "always":
            light = int(recipe.get("light_brightness_pct", 0))
        else:
            light = 0
        if recipe.get("cycle_enable"):
            rec = float(recipe.get("cycle_record_minutes", 10))
            idle = float(recipe.get("cycle_idle_minutes", 50))
        else:
            rec, idle = None, 0.0
    if idle <= 0:
        rec, idle = None, 0.0
    return rec, idle, light


def next_local_hour(now_utc, zone):
    """The next top of the hour in zone, as an aware UTC datetime.

    Counted in UTC from the seconds already past the local hour, so DST
    changes and half-hour zones come out right; wall-clock arithmetic on
    the local time would not.
    """
    local = now_utc.astimezone(zone)
    past = local.minute * 60 + local.second + local.microsecond / 1e6
    return now_utc + timedelta(seconds=3600 - past)


class PhaseTracker:
    """Remembers when the current day/night phase began.  Record/idle
    windows are timed from that moment, so entering night starts a
    recording window straight away.  Also remembers when the recording
    started, for the first clip's FIRST_CLIP_MIN_S rule."""

    def __init__(self):
        self.phase = None
        self.anchor = None
        self.started = None


def next_segment(now_utc, recipe, tracker, end_utc=None, split_zone=None):
    """Work out the segment running at now_utc.

    With split_zone set, continuous recording and record windows longer
    than HOURLY_SPLIT_MINUTES also end at the top of each hour in that zone.

    Returns a dict: kind ('record'|'idle'), phase ('day'|'night'),
    continuous (bool), light_pct (None = leave the light alone), end (aware
    UTC, capped at the hour, the day/night boundary and end_utc),
    next_change (that boundary, or None) and phase_changed.
    """
    if recipe.get("night_enable"):
        is_night, boundary = night_window_at(now_utc, recipe)
        phase = "night" if is_night else "day"
    else:
        phase, boundary = "day", None
    changed = phase != tracker.phase
    # A clock that jumped backwards would put the anchor in the future.
    if changed or tracker.anchor is None or tracker.anchor > now_utc:
        tracker.phase, tracker.anchor = phase, now_utc
    if tracker.started is None or tracker.started > now_utc:
        tracker.started = now_utc
    rec, idle, light = phase_settings(recipe, phase)
    if rec is None:
        kind, end = "record", None
    else:
        period_s = (rec + idle) * 60.0
        elapsed = (now_utc - tracker.anchor).total_seconds()
        k = int(elapsed // period_s)
        window_start = tracker.anchor + timedelta(seconds=k * period_s)
        if elapsed - k * period_s < rec * 60.0:
            kind, end = "record", window_start + timedelta(minutes=rec)
        else:
            kind, end = "idle", window_start + timedelta(seconds=period_s)
    hour = None
    if kind == "record" and split_zone is not None and (rec is None or rec > HOURLY_SPLIT_MINUTES):
        hour = next_local_hour(now_utc, split_zone)
        if (hour - tracker.started).total_seconds() < FIRST_CLIP_MIN_S:
            hour = next_local_hour(hour, split_zone)
    for cap in (hour, boundary, end_utc):
        if cap is not None and (end is None or cap < end):
            end = cap
    if end is None:
        # Continuous, unsplit and open-ended: replan once a day.
        end = now_utc + timedelta(days=1)
    return {
        "kind": kind,
        "phase": phase,
        "continuous": idle <= 0,
        "light_pct": light if kind == "record" or light is None else 0,
        "end": end,
        "next_change": boundary,
        "phase_changed": changed,
    }


def simulate_schedule(start_utc, end_utc, recipe, split_zone=None):
    """List the segments a schedule produces between two UTC times."""
    tracker = PhaseTracker()
    out, now = [], start_utc
    while now < end_utc:
        seg = next_segment(now, recipe, tracker, end_utc, split_zone)
        seg["start"] = now
        out.append(seg)
        now = seg["end"]
    return out


def safe_label(name):
    """A recipe name made safe for file and folder names."""
    return re.sub(r"[^a-zA-Z0-9_-]", "", str(name or "").replace(" ", "_")) or "Recording"


def deployment_folder(name, start_local):
    """Folder for one recording session, e.g. Whalefall_20261116_0902."""
    return f"{safe_label(name)}_{start_local:%Y%m%d_%H%M}"


def clip_basename(name, index, start_local, phase=None):
    """Clip file name without extension, clip number first, e.g.
    Whalefall_c013_20261116_1900_night."""
    suffix = "_night" if phase == "night" else ""
    return f"{safe_label(name)}_c{int(index):03d}_{start_local:%Y%m%d_%H%M}{suffix}"


DEFAULT_RECIPES = [
    {
        "id": "quick-survey-30",
        "name": "Quick Survey 30min",
        "mode": "video",
        "still_interval_s": 1.0,
        "duration_minutes": 30,
        "auto_start_delay_minutes": 1,
        "rotation_degrees": 0,
        "servo_start_us": 1500,
        "servo_end_us": 1500,
        "servo_pause_points": 0,
        "servo_loiter_time_s": 0,
        "servo_oscillations": 1,
        "servo_fixed": True,
        "light_brightness_pct": 80,
        "light_mode": "always",
        "led_color": "red",
        "led_blink": "slow",
        "still_prefix": "",
    },
    {
        "id": "deep-drop-2hr",
        "name": "Deep Drop 2hr",
        "mode": "video",
        "still_interval_s": 1.0,
        "duration_minutes": 120,
        "auto_start_delay_minutes": 5,
        "rotation_degrees": 0,
        "servo_start_us": 1200,
        "servo_end_us": 1800,
        "servo_pause_points": 3,
        "servo_loiter_time_s": 5,
        "servo_oscillations": 10,
        "servo_fixed": False,
        "light_brightness_pct": 100,
        "light_mode": "always",
        "led_color": "red",
        "led_blink": "slow",
        "still_prefix": "",
    },
    {
        "id": "time-lapse-4hr",
        "name": "Time Lapse 4hr",
        "mode": "stills",
        "still_interval_s": 5.0,
        "duration_minutes": 240,
        "auto_start_delay_minutes": 2,
        "rotation_degrees": 0,
        "servo_start_us": 1500,
        "servo_end_us": 1500,
        "servo_pause_points": 0,
        "servo_loiter_time_s": 0,
        "servo_oscillations": 1,
        "servo_fixed": True,
        "light_brightness_pct": 50,
        "light_mode": "always",
        "led_color": "green",
        "led_blink": "slow",
        "still_prefix": "",
    },
]

# Added to every unit that doesn't have them yet (DEFAULT_RECIPES only seed
# an empty recipes folder).
SCHEDULE_DEFAULT_RECIPES = [
    {
        "id": "whale-fall-timelapse",
        "name": "Whale Fall Timelapse",
        "mode": "video",
        "duration_minutes": 60,
        "auto_start_delay_minutes": 1,
        "servo_fixed": True,
        "light_brightness_pct": 0,
        "light_mode": "off",
        "led_color": "blue",
        "led_blink": "slow",
        "run_until_stopped": True,
        "night_enable": True,
        "night_start": "19:00",
        "night_end": "06:00",
        "night_record_minutes": 5,
        "night_idle_minutes": 10,
        "night_light_brightness_pct": 80,
        "night_awb_once": True,
    },
]

CAM4K_DEFAULT_RECIPES = [
    {
        "id": "cam4k-focus-finder",
        "name": "4k Cam Focus Finder",
        "mode": "video",
        "still_interval_s": 1.0,
        "duration_minutes": 2,
        "auto_start_delay_minutes": 0,
        "rotation_degrees": 0,
        "servo_start_us": 1500,
        "servo_end_us": 1500,
        "servo_pause_points": 0,
        "servo_loiter_time_s": 0,
        "servo_oscillations": 1,
        "servo_fixed": True,
        "light_brightness_pct": 0,
        "light_mode": "off",
        "led_color": "cyan",
        "led_blink": "fast",
        "still_prefix": "",
        "cam4k_focus_finder": True,
        "focus_sweep_start_us": 870,
        "focus_sweep_end_us": 2130,
        "focus_finder_zoom_us": 935,
    },
]


def _ensure_dir():
    os.makedirs(RECIPES_DIR, exist_ok=True)


def _recipe_path(recipe_id):
    return os.path.join(RECIPES_DIR, f"{recipe_id}.json")


VALID_LED_COLORS = ("red", "green", "blue", "yellow", "cyan", "magenta", "white")
VALID_LED_BLINKS = ("solid", "slow", "fast")
VALID_LIGHT_MODES = ("off", "always", "pause_only", "snapshot_only")


def validate_recipe(data):
    """Validate and fill defaults for a recipe dict. Returns (clean_dict, errors)."""
    errors = []
    clean = dict(RECIPE_SCHEMA_DEFAULTS)

    # Backward compatibility: light_on bool -> light_mode
    if "light_on" in data and "light_mode" not in data:
        clean["light_mode"] = "always" if data["light_on"] else "off"

    if "name" in data:
        clean["name"] = str(data["name"]).strip()[:80]
    if not clean["name"]:
        errors.append("name is required")

    if "mode" in data:
        if data["mode"] not in ("video", "stills"):
            errors.append("mode must be 'video' or 'stills'")
        else:
            clean["mode"] = data["mode"]

    if "duration_minutes" in data:
        try:
            dm = parse_duration_to_minutes(data["duration_minutes"])
        except ValueError:
            errors.append(
                "duration_minutes: use minutes (e.g. 90) or hours (e.g. 4.65 hours)"
            )
        else:
            if dm < 1 or dm > RECIPE_MAX_DURATION_MINUTES:
                errors.append(
                    f"duration must be between 1 minute and {RECIPE_MAX_DURATION_MINUTES} minutes (32 hours)"
                )
            else:
                clean["duration_minutes"] = round(dm, 2)

    for fld, lo, hi in [
        ("still_interval_s", 0.1, 3600),
        ("auto_start_delay_minutes", 0, 60),
        # 4k Cam tilt travels 865-2250 µs; DropCam stays in the 1000-2000 subset.
        ("servo_start_us", 865, 2250),
        ("servo_end_us", 865, 2250),
        ("servo_pause_points", 0, 50),
        ("servo_loiter_time_s", 0, 3600),
        ("servo_oscillations", 0, 999),
        ("light_brightness_pct", 0, 100),
    ]:
        if fld in data:
            try:
                val = float(data[fld])
                if val < lo or val > hi:
                    errors.append(f"{fld} must be between {lo} and {hi}")
                else:
                    clean[fld] = int(val) if isinstance(RECIPE_SCHEMA_DEFAULTS[fld], int) else round(val, 1)
            except (ValueError, TypeError):
                errors.append(f"{fld} must be a number")

    if "rotation_degrees" in data:
        rd = int(data["rotation_degrees"])
        if rd not in (0, 90, 180, 270):
            errors.append("rotation_degrees must be 0, 90, 180, or 270")
        else:
            clean["rotation_degrees"] = rd

    if "servo_fixed" in data:
        clean["servo_fixed"] = bool(data["servo_fixed"])

    if "light_mode" in data:
        if data["light_mode"] not in VALID_LIGHT_MODES:
            errors.append(f"light_mode must be one of {VALID_LIGHT_MODES}")
        else:
            clean["light_mode"] = data["light_mode"]

    if "led_color" in data:
        if data["led_color"] not in VALID_LED_COLORS:
            errors.append(f"led_color must be one of {VALID_LED_COLORS}")
        else:
            clean["led_color"] = data["led_color"]

    if "led_blink" in data:
        if data["led_blink"] not in VALID_LED_BLINKS:
            errors.append(f"led_blink must be one of {VALID_LED_BLINKS}")
        else:
            clean["led_blink"] = data["led_blink"]

    if "still_prefix" in data:
        pfx = re.sub(r'[^a-zA-Z0-9_-]', '', str(data["still_prefix"]).strip())[:32]
        clean["still_prefix"] = pfx

    if "release_enable" in data:
        clean["release_enable"] = bool(data["release_enable"])

    if "release_offset_s" in data:
        try:
            ros = int(float(data["release_offset_s"]))
        except (ValueError, TypeError):
            errors.append("release_offset_s must be a number (seconds)")
        else:
            if ros < RELEASE_OFFSET_MIN_S or ros > RELEASE_OFFSET_MAX_S:
                errors.append(
                    f"release_offset_s must be between {RELEASE_OFFSET_MIN_S} "
                    f"and {RELEASE_OFFSET_MAX_S} seconds"
                )
            else:
                clean["release_offset_s"] = ros

    if "winch_enable" in data:
        clean["winch_enable"] = bool(data["winch_enable"])

    if "winch_rotations" in data:
        try:
            wr = int(float(data["winch_rotations"]))
        except (ValueError, TypeError):
            errors.append("winch_rotations must be an integer")
        else:
            from hardware import WINCH_ROTATIONS_MAX
            if wr < 1 or wr > WINCH_ROTATIONS_MAX:
                errors.append(
                    f"winch_rotations must be between 1 and {WINCH_ROTATIONS_MAX}"
                )
            else:
                clean["winch_rotations"] = wr

    if "winch_profiles" in data:
        try:
            wp = int(float(data["winch_profiles"]))
        except (ValueError, TypeError):
            errors.append("winch_profiles must be an integer")
        else:
            if wp < 1 or wp > 10000:
                errors.append("winch_profiles must be >= 1")
            else:
                clean["winch_profiles"] = wp

    if "winch_start_delay_minutes" in data:
        try:
            wsd = float(data["winch_start_delay_minutes"])
        except (ValueError, TypeError):
            errors.append("winch_start_delay_minutes must be a number")
        else:
            if wsd < 0 or wsd > RECIPE_MAX_DURATION_MINUTES:
                errors.append(
                    f"winch_start_delay_minutes must be between 0 "
                    f"and {RECIPE_MAX_DURATION_MINUTES} minutes"
                )
            else:
                clean["winch_start_delay_minutes"] = wsd

    if "cycle_enable" in data:
        clean["cycle_enable"] = bool(data["cycle_enable"])

    for fld in ("cycle_record_minutes", "cycle_idle_minutes"):
        if fld in data:
            try:
                val = parse_duration_to_minutes(data[fld])
            except ValueError:
                errors.append(f"{fld} must be a number of minutes")
                continue
            if val < CYCLE_MIN_MINUTES or val > RECIPE_MAX_DURATION_MINUTES:
                errors.append(
                    f"{fld} must be between {CYCLE_MIN_MINUTES} and "
                    f"{RECIPE_MAX_DURATION_MINUTES} minutes"
                )
            else:
                clean[fld] = round(val, 2)

    if clean["cycle_enable"] and clean["light_mode"] not in CYCLE_LIGHT_MODES:
        errors.append(
            "light_mode must be 'off' or 'always' when the record/idle cycle is enabled"
        )

    for fld in ("run_until_stopped", "night_enable", "night_awb_once"):
        if fld in data:
            clean[fld] = bool(data[fld])

    for fld in ("night_start", "night_end"):
        if fld in data:
            try:
                h, m = _hhmm(data[fld])
            except ValueError:
                errors.append(f"{fld} must be a time like 19:00")
            else:
                clean[fld] = f"{h:02d}:{m:02d}"
    if clean["night_enable"] and clean["night_start"] == clean["night_end"]:
        errors.append("night start and end times must differ")

    for fld, lo in (("night_record_minutes", CYCLE_MIN_MINUTES), ("night_idle_minutes", 0.0)):
        if fld in data:
            try:
                val = parse_duration_to_minutes(data[fld])
            except ValueError:
                errors.append(f"{fld} must be a number of minutes")
                continue
            if val < lo or val > 24 * 60:
                errors.append(f"{fld} must be between {lo} and {24 * 60} minutes")
            else:
                clean[fld] = round(val, 2)

    if "night_light_brightness_pct" in data:
        try:
            nl = float(data["night_light_brightness_pct"])
        except (ValueError, TypeError):
            errors.append("night_light_brightness_pct must be a number")
        else:
            if nl < 0 or nl > 100:
                errors.append("night_light_brightness_pct must be between 0 and 100")
            else:
                clean["night_light_brightness_pct"] = int(nl)

    if "time_zone" in data:
        name = str(data["time_zone"] or "").strip()
        if not name or tz.gettz(name) is None:
            errors.append(f"unknown time zone '{name}' (use a name like America/Los_Angeles)")
        else:
            clean["time_zone"] = name

    if clean["night_enable"] and clean["light_mode"] not in CYCLE_LIGHT_MODES:
        errors.append(
            "daytime light_mode must be 'off' or 'always' when the night schedule is on"
        )

    if "cam4k_focus_finder" in data:
        clean["cam4k_focus_finder"] = bool(data["cam4k_focus_finder"])

    if "cam4k_awb_enable" in data:
        clean["cam4k_awb_enable"] = bool(data["cam4k_awb_enable"])

    if "cam4k_awb_interval_s" in data:
        try:
            awb_iv = float(data["cam4k_awb_interval_s"])
        except (ValueError, TypeError):
            errors.append("cam4k_awb_interval_s must be a number")
        else:
            if awb_iv < 10 or awb_iv > 3600:
                errors.append("cam4k_awb_interval_s must be between 10 and 3600")
            else:
                clean["cam4k_awb_interval_s"] = int(round(awb_iv))

    for fld, lo, hi in [
        ("focus_sweep_start_us", 870, 2130),
        ("focus_sweep_end_us", 870, 2130),
        ("focus_finder_zoom_us", 935, 1850),
        ("cam4k_focus_us", 870, 2130),
        ("cam4k_zoom_us", 935, 1850),
    ]:
        if fld in data:
            try:
                val = int(float(data[fld]))
                if val < lo or val > hi:
                    errors.append(f"{fld} must be between {lo} and {hi}")
                else:
                    clean[fld] = val
            except (ValueError, TypeError):
                errors.append(f"{fld} must be a number")

    if clean["run_until_stopped"]:
        # These are all timed from the total duration, which an open-ended
        # run doesn't have.
        for fld, what in (("release_enable", "release"),
                          ("winch_enable", "winch profiling"),
                          ("cam4k_focus_finder", "focus finder")):
            if clean[fld]:
                errors.append(f"{what} can't be used with 'run until stopped'")
        if not clean["servo_fixed"]:
            errors.append("a tilt sweep can't be used with 'run until stopped'")

    if "id" in data:
        clean["id"] = str(data["id"])

    return clean, errors


# Saved before the RadCam -> 4k Cam rename: settings keys were ``radcam_*``
# and the focus-finder recipe was ``radcam-focus-finder``.
_LEGACY_PREFIX, _NEW_PREFIX = "radcam_", "cam4k_"
LEGACY_RECIPE_IDS = {"radcam-focus-finder": "cam4k-focus-finder"}


def rename_legacy_keys(d):
    """Rename ``radcam_*`` keys to ``cam4k_*`` in place. Returns True if any changed."""
    old_keys = [k for k in d if k.startswith(_LEGACY_PREFIX)]
    for k in old_keys:
        d.setdefault(_NEW_PREFIX + k[len(_LEGACY_PREFIX):], d[k])
        del d[k]
    return bool(old_keys)


def migrate_legacy_recipes():
    """Rewrite recipe files saved with the pre-rename names."""
    _ensure_dir()
    for fname in sorted(os.listdir(RECIPES_DIR)):
        if not fname.endswith(".json") or fname == "dropcam_config.json":
            continue
        old_id = fname[:-5]
        new_id = LEGACY_RECIPE_IDS.get(old_id, old_id)
        path = os.path.join(RECIPES_DIR, fname)
        try:
            with open(path, "r") as f:
                r = json.load(f)
        except Exception:
            continue
        if not rename_legacy_keys(r) and new_id == old_id:
            continue
        r["id"] = new_id
        new_path = _recipe_path(new_id)
        tmp = new_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(r, f, indent=2)
        os.replace(tmp, new_path)
        if new_path != path:
            os.remove(path)
        logger.info(f"Migrated recipe {old_id} to 4k Cam naming")


def init_default_recipes(cam4k=False):
    """Create default recipe files if the recipes dir is empty.

    When cam4k=True, also ensure the 4k Cam-specific recipes exist.
    """
    _ensure_dir()
    migrate_legacy_recipes()
    existing = list_recipes()
    if not existing:
        for recipe in DEFAULT_RECIPES:
            save_recipe(recipe, recipe["id"])
        logger.info(f"Initialized {len(DEFAULT_RECIPES)} default recipes")

    existing_ids = {r["id"] for r in list_recipes()}
    for recipe in SCHEDULE_DEFAULT_RECIPES:
        if recipe["id"] not in existing_ids:
            save_recipe(recipe, recipe["id"])
            logger.info(f"Added default recipe '{recipe['name']}'")

    if cam4k:
        existing_ids = {r["id"] for r in list_recipes()}
        added = 0
        for recipe in CAM4K_DEFAULT_RECIPES:
            if recipe["id"] not in existing_ids:
                save_recipe(recipe, recipe["id"])
                added += 1
        if added:
            logger.info(f"Initialized {added} 4k Cam default recipe(s)")


def list_recipes():
    """Return list of all saved recipes (dicts with id + name)."""
    _ensure_dir()
    recipes = []
    for fname in sorted(os.listdir(RECIPES_DIR)):
        if not fname.endswith(".json") or fname == "dropcam_config.json":
            continue
        try:
            with open(os.path.join(RECIPES_DIR, fname), "r") as f:
                r = json.load(f)
            r["id"] = fname[:-5]
            recipes.append(r)
        except Exception as e:
            logger.warning(f"Bad recipe file {fname}: {e}")
    return recipes


def get_recipe(recipe_id):
    """Load a single recipe by id. Returns dict or None."""
    path = _recipe_path(recipe_id)
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r") as f:
            r = json.load(f)
        r["id"] = recipe_id
        return r
    except Exception as e:
        logger.error(f"Failed to load recipe {recipe_id}: {e}")
    return None


def save_recipe(data, recipe_id=None):
    """Validate and save a recipe atomically. Returns (recipe_dict, errors)."""
    _ensure_dir()
    clean, errors = validate_recipe(data)
    if errors:
        return None, errors
    if recipe_id is None:
        recipe_id = clean.get("id", uuid.uuid4().hex[:12])
    clean["id"] = recipe_id
    path = _recipe_path(recipe_id)
    try:
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(clean, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
        return clean, []
    except Exception as e:
        return None, [str(e)]


def delete_recipe(recipe_id):
    """Delete a recipe file. Returns True on success."""
    path = _recipe_path(recipe_id)
    if os.path.exists(path):
        os.remove(path)
        return True
    return False
