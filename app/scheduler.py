"""
Scheduler for DropCam auto-start recording, duration tracking,
servo sweep orchestration, and disk space guard.

The scheduler runs in a background thread. On startup it reads the
active recipe from config and, after the configured delay, calls back
into the main app to start recording and hardware control.
"""

import json
import logging
import re
import threading
import time
from datetime import datetime, timedelta, timezone
import urllib.error
import urllib.request

from dateutil import tz

logger = logging.getLogger(__name__)

DISK_FREE_MINIMUM_MB = 1024  # 1 GB
RECORDING_START_RETRIES = 5
RECORDING_RETRY_INTERVAL_S = 5
# Wait before restarting a clip the recorder dropped mid-segment; doubles for
# each clip in a row that died within CLIP_DROP_QUICK_S, up to the max.
CLIP_RESTART_DELAY_S = 5
CLIP_RESTART_DELAY_MAX_S = 300
CLIP_DROP_QUICK_S = 120
# Night-once AWB: wait this long after the light comes on (exposure settles),
# and retry this often if the camera doesn't answer.
NIGHT_AWB_SETTLE_S = 10
NIGHT_AWB_RETRY_S = 30

# Same POSTs 4k Cam Manager / towfish use for WB scene + one-push.
# cgi_action GET path returns "error user/pwd" on current firmware.
CAM4K_IMAGE_URL = "http://192.168.2.10/action/setImageAdjustment"
CAM4K_AWB_URL = "http://192.168.2.10/action/setImageAdjustmentEx"
CAM4K_AWB_BODY = {"onceAWB": 1}

# Water-environment AWB scene (4k Cam Manager Green/Blue buttons).
# Maps to camera awb_auto_mode; scene only applies with auto_awb=0 (Auto).
AWB_SCENE_MODE = {"green": 0, "blue": 1}
DEFAULT_AWB_SCENE = "blue"


def normalize_awb_scene(scene) -> str:
    s = str(scene or DEFAULT_AWB_SCENE).strip().lower()
    return s if s in AWB_SCENE_MODE else DEFAULT_AWB_SCENE


def apply_awb_scene(scene) -> bool:
    """Set 4k Cam Auto WB + water scene (green/blue). Never raises."""
    scene = normalize_awb_scene(scene)
    body = {
        "auto_awb": 0,  # Auto (Manual=1); scene is ignored in Manual
        "awb_auto_mode": AWB_SCENE_MODE[scene],
    }
    try:
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            CAM4K_IMAGE_URL,
            data=data,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=3) as resp:
            body_text = (resp.read() or b"").decode("utf-8", errors="replace").strip()
            code = None
            try:
                code = (json.loads(body_text) or {}).get("code")
            except Exception:
                pass
            if code is None or code == 0:
                logger.info(
                    "4k Cam AWB scene set: %s (auto_awb=0, awb_auto_mode=%d)",
                    scene, AWB_SCENE_MODE[scene],
                )
                return True
            logger.warning("4k Cam AWB scene failed: code %s: %s", code, body_text[:120])
    except urllib.error.HTTPError as e:
        logger.warning("4k Cam AWB scene failed: HTTP %s", e.code)
    except Exception as e:
        logger.warning("4k Cam AWB scene failed: %s", e)
    return False

LED_COLOR_MAP = {
    "red": (20, 0, 0),
    "green": (0, 20, 0),
    "blue": (0, 0, 20),
    "yellow": (20, 20, 0),
    "cyan": (0, 20, 20),
    "magenta": (20, 0, 20),
    "white": (20, 20, 20),
}

LED_BLINK_RATE = {
    "solid": 0,
    "slow": 0.5,
    "fast": 2.0,
}


class Scheduler:
    def __init__(self):
        self._thread = None
        self._stop = threading.Event()
        self._active_recipe = None
        self._start_recording_fn = None
        self._stop_recording_fn = None
        self._get_disk_free_fn = None
        self._is_recording_fn = None
        self._default_tz_fn = None
        self._capture_still_fn = None
        self._log_event_fn = None
        self._hw = None
        self._state = "idle"
        self._lock = threading.Lock()
        self._remaining_s = 0
        self._focus_sweep_thread = None
        self._release_thread = None
        self._winch_thread = None
        self._awb_thread = None
        self._cam4k_mode = False
        self._awb_scene = DEFAULT_AWB_SCENE
        # Record / idle cycle progress (0 when the recipe doesn't cycle).
        self._cycle_window = 0
        self._cycle_windows_total = 0
        # Clip progress for any session that splits into clips.
        self._clip = 0
        self._session_started = None  # epoch seconds of the first clip
        self._manual = False
        self._night_schedule = False
        self._deadline = None  # monotonic end of the whole deployment
        # Wall-clock schedule (night window / run until stopped) progress.
        self._phase = None
        self._light_pct = 0
        self._light_on_since = None  # monotonic time the schedule lit the light
        self._light_touched = False  # the clip loop has set the light
        self._next_change_utc = None
        self._zone = None
        self._open_ended = False
        self._now = lambda: datetime.now(timezone.utc)

    def configure(self, *, start_fn, stop_fn, disk_free_fn, hw,
                  is_recording_fn=None, default_tz_fn=None, capture_still_fn=None,
                  log_event_fn=None, cam4k_mode=False, awb_scene=None):
        self._start_recording_fn = start_fn
        self._stop_recording_fn = stop_fn
        self._get_disk_free_fn = disk_free_fn
        self._is_recording_fn = is_recording_fn
        # Time zone for clip names when the recipe has no night schedule.
        self._default_tz_fn = default_tz_fn
        self._capture_still_fn = capture_still_fn
        # Inject main's log_event so scheduler-thread events land in the active
        # recording's events.ndjson.  Re-importing main here would load a second
        # module copy (main.py runs as __main__), whose current_events_file is
        # always None, silently dropping every winch/release event.
        self._log_event_fn = log_event_fn
        self._hw = hw
        self._cam4k_mode = bool(cam4k_mode)
        if awb_scene is not None:
            self._awb_scene = normalize_awb_scene(awb_scene)

    def set_cam4k_mode(self, enabled):
        """Update tilt-nod extents after a late 4k Cam detect."""
        self._cam4k_mode = bool(enabled)

    def set_awb_scene(self, scene):
        """Update the water-environment AWB scene used before each one-push."""
        self._awb_scene = normalize_awb_scene(scene)

    def get_awb_scene(self):
        return normalize_awb_scene(self._awb_scene)

    def _log(self, event, detail=""):
        """Best-effort event log via the injected main.log_event."""
        if not self._log_event_fn:
            return
        try:
            self._log_event_fn(event, detail)
        except Exception as e:
            logger.debug(f"scheduler log_event failed: {e}")

    def start(self, recipe):
        """Begin the auto-start sequence for the given recipe dict."""
        self.stop()
        self._stop.clear()
        # Park the release servo at its stop position (1500 us) at the very
        # start of every recipe, cancelling any prior in-flight test or
        # manual wind/unwind so the new run begins from a known state.
        if self._hw:
            try:
                self._hw.release_stop()
            except Exception as e:
                logger.warning(f"Could not reset release servo: {e}")
        self._active_recipe = recipe
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(f"Scheduler started for recipe '{recipe.get('name')}'")

    def manual_session(self):
        """Clip 1's recipe context for a manual recording.

        The caller starts clip 1 with it, then hands it to start_manual so
        later clips split at the top of each hour.  Manual sessions leave
        the light, tilt, LED, AWB and rotation to the operator.
        """
        from recipes import deployment_folder
        zone_name = self._label_tz_name({})
        zone = tz.gettz(zone_name) or tz.UTC
        recipe = {
            "name": "Manual", "mode": "video", "run_until_stopped": True,
            "time_zone": zone_name, "_manual": True,
        }
        return self._clip_recipe(
            recipe, deployment_folder("Manual", self._now().astimezone(zone)),
            zone_name, 1, "day")

    def start_manual(self, recipe):
        """Take over a manual recording whose clip 1 is already running."""
        self.stop()
        self._stop.clear()
        self._active_recipe = recipe
        self._thread = threading.Thread(target=self._run_manual, daemon=True)
        self._thread.start()
        logger.info("Scheduler took over manual recording %s", recipe.get("_deployment_id"))

    def is_manual(self):
        return self.is_running() and self._manual

    def _label_tz_name(self, recipe):
        """Time zone for clip names: a night-schedule recipe's own zone,
        otherwise the unit's setting.  (Every saved recipe has a time_zone,
        but it only means anything for the night schedule.)"""
        if recipe.get("night_enable") and recipe.get("time_zone"):
            return recipe["time_zone"]
        name = None
        if self._default_tz_fn:
            try:
                name = self._default_tz_fn()
            except Exception as e:
                logger.debug(f"default time zone lookup failed: {e}")
        return name if name and tz.gettz(name) else "UTC"

    @staticmethod
    def _clip_recipe(recipe, deployment_id, zone_name, index, phase):
        return dict(recipe, _deployment_id=deployment_id, _window_index=index,
                    _clip_phase=phase, _label_tz=zone_name)

    def stop(self):
        """Cancel any running schedule."""
        self._stop.set()
        if self._focus_sweep_thread and self._focus_sweep_thread.is_alive():
            self._focus_sweep_thread.join(timeout=5)
        if self._awb_thread and self._awb_thread.is_alive():
            self._awb_thread.join(timeout=5)
        if self._release_thread and self._release_thread.is_alive():
            self._release_thread.join(timeout=5)
        if self._winch_thread and self._winch_thread.is_alive():
            self._winch_thread.join(timeout=10)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=10)
        with self._lock:
            self._state = "idle"
            self._remaining_s = 0
            self._cycle_window = 0
            self._cycle_windows_total = 0
            self._clip = 0
            self._session_started = None
            self._manual = False
            self._night_schedule = False
            self._deadline = None
            self._phase = None
            self._light_pct = 0
            self._light_on_since = None
            self._next_change_utc = None
            self._open_ended = False
        self._active_recipe = None

    def get_state(self):
        with self._lock:
            total_remaining = None
            if self._deadline is not None:
                total_remaining = round(max(0.0, self._deadline - time.monotonic()), 1)
            next_change = None
            if self._next_change_utc is not None and self._zone is not None:
                next_change = self._next_change_utc.astimezone(self._zone).strftime("%H:%M")
            return {
                "state": self._state,
                "remaining_s": round(self._remaining_s, 1),
                "recipe_name": self._active_recipe["name"] if self._active_recipe else None,
                "cycle_window": self._cycle_window,
                "cycle_windows_total": self._cycle_windows_total,
                "clip": self._clip,
                "session_started": self._session_started,
                "is_manual": self._manual,
                "night_schedule": self._night_schedule,
                "total_remaining_s": total_remaining,
                # Wall-clock schedule only (None / False otherwise).
                "phase": self._phase,
                "light_pct": self._light_pct,
                "next_change_local": next_change,
                "open_ended": self._open_ended,
            }

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    def _set_state(self, state, remaining=0):
        with self._lock:
            self._state = state
            self._remaining_s = remaining

    def _apply_recipe_led(self, recipe):
        """Set LED color/blink from recipe settings."""
        if not self._hw:
            return
        color_name = recipe.get("led_color", "red")
        blink_name = recipe.get("led_blink", "slow")
        r, g, b = LED_COLOR_MAP.get(color_name, (255, 0, 0))
        rate = LED_BLINK_RATE.get(blink_name, 0.5)
        if rate == 0:
            self._hw.set_led_color(r, g, b)
        else:
            self._hw.flash_led(r, g, b, rate_hz=rate)

    def _run(self):
        recipe = self._active_recipe
        if not recipe:
            return

        try:
            delay_s = recipe.get("auto_start_delay_minutes", 1) * 60
            self._countdown("delay", delay_s)
            if self._stop.is_set():
                return

            # Visual "I'm about to start" cue before the recorder arms.
            if self._hw:
                try:
                    start_us = int(recipe.get("servo_start_us", 1500))
                    self._set_state("nodding")
                    self._hw.nod_gesture(
                        cam4k=self._cam4k_mode,
                        return_us=start_us,
                        stop_event=self._stop,
                    )
                except Exception as e:
                    logger.warning("Pre-recipe tilt nod failed: %s", e)
            if self._stop.is_set():
                return

            from recipes import (calculate_cycle_plan, deployment_folder,
                                 night_window_at, schedule_owns_day_light, uses_schedule)
            video = recipe.get("mode", "video") == "video"
            # Every video recipe runs the wall-clock clip loop, which splits
            # long recordings at the top of each hour; stills only when they
            # need the schedule.
            scheduled = video or uses_schedule(recipe)
            open_ended = bool(recipe.get("run_until_stopped"))
            duration_s = float("inf") if open_ended else recipe.get("duration_minutes", 30) * 60
            cycling = bool(recipe.get("cycle_enable")) and not scheduled
            if scheduled:
                zone_name = self._label_tz_name(recipe)
                deployment_id = deployment_folder(
                    recipe.get("name"), self._now().astimezone(tz.gettz(zone_name) or tz.UTC))
                first_phase = "day"
                if recipe.get("night_enable") and night_window_at(self._now(), recipe)[0]:
                    first_phase = "night"
                windows_total = 0
                if recipe.get("cycle_enable") and not recipe.get("night_enable") and not open_ended:
                    windows_total = calculate_cycle_plan(
                        recipe.get("duration_minutes", 30),
                        recipe.get("cycle_record_minutes", 10),
                        recipe.get("cycle_idle_minutes", 50),
                    )["windows"]
                with self._lock:
                    self._cycle_windows_total = windows_total
                    self._cycle_window = 1
                    self._clip = 1
                    self._open_ended = open_ended
                    self._night_schedule = bool(recipe.get("night_enable"))
                    self._zone = tz.gettz(zone_name) or tz.UTC
                    self._phase = first_phase
                logger.info(
                    "Scheduled deployment %s: night=%s %s-%s, names in %s, %s",
                    deployment_id, bool(recipe.get("night_enable")),
                    recipe.get("night_start"), recipe.get("night_end"), zone_name,
                    "until stopped" if open_ended else f"{recipe.get('duration_minutes')} min",
                )
            elif cycling:
                plan = calculate_cycle_plan(
                    recipe.get("duration_minutes", 30),
                    recipe.get("cycle_record_minutes", 10),
                    recipe.get("cycle_idle_minutes", 50),
                )
                safe_name = re.sub(r"[^a-zA-Z0-9_-]", "",
                                   str(recipe.get("name", "recipe")).replace(" ", "_"))
                deployment_id = f"{safe_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                with self._lock:
                    self._cycle_windows_total = plan["windows"]
                    self._cycle_window = 1
                logger.info(
                    "Record/idle cycle: %s min record / %s min idle over %s min "
                    "-> %d window(s), deployment %s",
                    recipe.get("cycle_record_minutes"), recipe.get("cycle_idle_minutes"),
                    recipe.get("duration_minutes"), plan["windows"], deployment_id,
                )

            def window_recipe(index, phase="day"):
                if scheduled:
                    return self._clip_recipe(recipe, deployment_id, zone_name, index, phase)
                if cycling:
                    return dict(recipe, _deployment_id=deployment_id, _window_index=index)
                return recipe

            if not self._start_recording_with_retries(
                    window_recipe(1, first_phase if scheduled else "day")):
                return
            with self._lock:
                self._deadline = None if open_ended else time.monotonic() + duration_s
                self._session_started = time.time()
            if cycling:
                self._log("cycle_window_started", f"window=1/{self._cycle_windows_total}")

            self._apply_recipe_led(recipe)

            if self._hw and recipe.get("release_enable"):
                offset_s = int(recipe.get("release_offset_s", 0))
                self._schedule_release(duration_s, offset_s, recipe)

            if self._hw:
                # NB: ``zoom`` shares GPIO 26 with the DropCam release-shaft
                # rotation sensor.  is_aux_pwm_available() returns False for
                # ``zoom`` when the sensor is live so we skip the recipe push
                # entirely and don't clobber the input — otherwise pigpiod
                # starts driving the pin as a servo output, the sensor
                # callback goes silent, and closed-loop winch legs run away
                # to the safety cap (~9 rev instead of 3).
                if "cam4k_focus_us" in recipe and self._hw.is_aux_pwm_available("focus"):
                    self._hw.set_aux_pwm("focus", recipe["cam4k_focus_us"])
                    logger.info(f"Recipe applied focus={recipe['cam4k_focus_us']} us")
                if "cam4k_zoom_us" in recipe and self._hw.is_aux_pwm_available("zoom"):
                    self._hw.set_aux_pwm("zoom", recipe["cam4k_zoom_us"])
                    logger.info(f"Recipe applied zoom={recipe['cam4k_zoom_us']} us")

            if recipe.get("cam4k_focus_finder") and self._hw:
                zoom_us = recipe.get("focus_finder_zoom_us", 935)
                self._hw.set_aux_pwm("zoom", zoom_us)
                start_us = recipe.get("focus_sweep_start_us", 870)
                end_us = recipe.get("focus_sweep_end_us", 2130)
                duration_s = recipe.get("duration_minutes", 2) * 60
                self._focus_sweep_thread = threading.Thread(
                    target=self._focus_sweep_loop,
                    args=(start_us, end_us, duration_s),
                    daemon=True,
                )
                self._focus_sweep_thread.start()
                logger.info(f"Focus finder: sweeping {start_us}-{end_us} us over "
                            f"{duration_s}s at zoom {zoom_us} us")

            # Default on (towfish behaviour) when the recipe omits the key.
            if recipe.get("cam4k_awb_enable", True):
                interval_s = float(recipe.get("cam4k_awb_interval_s", 120) or 120)
                interval_s = max(10.0, min(3600.0, interval_s))
                night_once = bool(recipe.get("night_enable") and recipe.get("night_awb_once"))
                self._awb_thread = threading.Thread(
                    target=self._awb_loop,
                    args=(interval_s, duration_s, night_once),
                    name="cam4k-awb-loop",
                    daemon=True,
                )
                self._awb_thread.start()
                logger.info(
                    "4k Cam AWB loop started (onceAWB=1 every %.0fs for %.0fs%s)",
                    interval_s, duration_s, ", once per night" if night_once else "",
                )

            if self._hw:
                light_mode = recipe.get("light_mode", "off")
                # Backward compat
                if "light_on" in recipe and "light_mode" not in recipe:
                    light_mode = "always" if recipe["light_on"] else "off"
                light_brightness = recipe.get("light_brightness_pct", 100)

                if not recipe.get("servo_fixed", True):
                    from recipes import calculate_sweep_time
                    sweep_info = calculate_sweep_time(
                        recipe.get("duration_minutes", 30),
                        recipe.get("servo_pause_points", 0),
                        recipe.get("servo_loiter_time_s", 0),
                        recipe.get("servo_oscillations", 1),
                    )
                    sweep_time_s = sweep_info["sweep_time_s"] if sweep_info["valid"] else 30
                    self._hw.sweep_servo(
                        recipe.get("servo_start_us", 1500),
                        recipe.get("servo_end_us", 1500),
                        sweep_time_s,
                        recipe.get("servo_pause_points", 0),
                        recipe.get("servo_loiter_time_s", 0),
                        recipe.get("servo_oscillations", 1),
                        light_mode=light_mode,
                        light_brightness_pct=light_brightness,
                        capture_still_fn=self._capture_still_fn,
                    )
                else:
                    self._hw.set_servo(recipe.get("servo_start_us", 1500))

                # The clip loop sets the light itself when it owns it.
                if not (scheduled and schedule_owns_day_light(recipe)):
                    if light_mode == "always":
                        self._hw.light_on(light_brightness)
                    elif recipe.get("servo_fixed", True) and light_mode == "off":
                        self._hw.light_off()

            if self._hw and recipe.get("winch_enable"):
                from hardware import WINCH_ROTATIONS_MAX
                rot = max(1, min(int(recipe.get("winch_rotations", 10) or 0),
                                 WINCH_ROTATIONS_MAX))
                prof = max(1, int(recipe.get("winch_profiles", 1) or 1))
                delay_s = max(0.0, float(recipe.get("winch_start_delay_minutes", 0)) * 60.0)
                self._winch_thread = threading.Thread(
                    target=self._winch_loop,
                    args=(rot, prof, duration_s, delay_s),
                    daemon=True,
                )
                self._winch_thread.start()
                logger.info(
                    f"Winch profiling started: {prof} profiles of {rot} rev each, "
                    f"delay {delay_s:.0f}s, recording {duration_s:.0f}s"
                )

            if scheduled:
                self._run_schedule(recipe, window_recipe)
            elif cycling:
                if not self._run_cycle(recipe, window_recipe):
                    return  # a later window failed to start; error state is set
            else:
                self._countdown("recording", duration_s, check_disk=True)

            if not self._stop.is_set():
                logger.info("Scheduler: duration reached, stopping recording")
                if not (cycling or scheduled):
                    self._stop_recording_fn()
                self._set_state("complete")
                if self._hw:
                    self._hw.stop_sweep()
                    self._hw.light_off()
                    self._hw.led_complete()
            elif self._state != "disk_full":
                self._set_state("stopped")

        except Exception as e:
            logger.error(f"Scheduler error: {e}", exc_info=True)
            self._set_state("error")
            if self._hw:
                self._hw.led_warning()

    def _start_recording_with_retries(self, recipe, attempts=RECORDING_START_RETRIES):
        """Start recording, retrying a few times. Returns True on success.

        On final failure sets the error state and warning LED.  A clip that
        comes up after Stop was requested is stopped again straight away, so
        a slow start can't outlive the session.
        """
        self._set_state("starting")
        for attempt in range(1, attempts + 1):
            if self._stop.is_set():
                return False
            if self._start_recording_fn(recipe):
                if self._stop.is_set():
                    self._stop_recording_fn()
                    return False
                return True
            logger.warning(f"Scheduler: recording start attempt {attempt}/{attempts} failed")
            if attempt < attempts:
                self._stop.wait(RECORDING_RETRY_INTERVAL_S)
        if self._stop.is_set():
            return False
        logger.error("Scheduler: recording failed to start after all retries")
        self._set_state("error")
        if self._hw:
            self._hw.led_warning()
        return False

    def _run_cycle(self, recipe, window_recipe):
        """Alternate record and idle windows until the deployment deadline.

        Window 1 is already recording when this is called. Each record
        window ends with the recording stopped, so the caller must not stop
        it again. The light (light_mode "always") is on only while recording.
        Returns False if a later window failed to start.
        """
        record_s = float(recipe.get("cycle_record_minutes", 10)) * 60
        idle_s = float(recipe.get("cycle_idle_minutes", 50)) * 60
        light_on_while_recording = recipe.get("light_mode", "off") == "always"
        brightness = recipe.get("light_brightness_pct", 100)
        window = 1

        def until_deadline():
            return max(0.0, self._deadline - time.monotonic())

        while True:
            self._countdown("recording", min(record_s, until_deadline()), check_disk=True)
            if self._stop.is_set():
                return True
            self._stop_recording_fn()
            if self._hw and light_on_while_recording:
                self._hw.light_off()
            if until_deadline() <= 0:
                return True

            self._log("cycle_idle_started", f"after_window={window}")
            self._countdown("cycle_idle", min(idle_s, until_deadline()))
            if self._stop.is_set() or until_deadline() <= 0:
                return True

            window += 1
            with self._lock:
                self._cycle_window = window
            if not self._start_recording_with_retries(window_recipe(window)):
                return False
            self._log("cycle_window_started", f"window={window}/{self._cycle_windows_total}")
            if self._hw:
                if light_on_while_recording:
                    self._hw.light_on(brightness)
                self._apply_recipe_led(recipe)

    def _run_manual(self):
        """Split a manual recording (clip 1 already running) into clips at
        the top of each hour until Stop or a full disk."""
        recipe = self._active_recipe
        zone_name = recipe["_label_tz"]
        deployment_id = recipe["_deployment_id"]
        with self._lock:
            self._manual = True
            self._open_ended = True
            self._clip = 1
            self._cycle_window = 1
            self._zone = tz.gettz(zone_name) or tz.UTC
            self._phase = "day"
            self._session_started = time.time()

        def window_recipe(index, phase="day"):
            return self._clip_recipe(recipe, deployment_id, zone_name, index, phase)

        try:
            self._run_schedule(recipe, window_recipe)
            if self._state != "disk_full":
                self._set_state("stopped")
        except Exception as e:
            logger.error(f"Manual session error: {e}", exc_info=True)
            self._set_state("error")

    def _run_schedule(self, recipe, window_recipe):
        """Follow the wall-clock schedule: day settings, the night window,
        and long recordings split into clips at the top of each hour
        (recipes.next_segment).

        Clip 1 is already recording when this is called. Every later clip
        goes in the same deployment folder; the light follows each segment
        when the schedule owns it. Runs until the deadline, or forever when
        open-ended (Stop / disk full / power loss end it).
        """
        self._light_touched = False
        try:
            self._schedule_loop(recipe, window_recipe)
        finally:
            # Disk full or an error can end the loop during a night clip;
            # don't leave the light burning.
            if self._light_touched:
                self._set_light(0)

    def _schedule_loop(self, recipe, window_recipe):
        from recipes import HOURLY_SPLIT_MINUTES, MIN_CLIP_S, PhaseTracker, next_segment

        # Video planned for an hour or less stays in one clip; stills never
        # split at the hour.
        long_run = (recipe.get("run_until_stopped")
                    or float(recipe.get("duration_minutes", 0)) > HOURLY_SPLIT_MINUTES)
        video = recipe.get("mode", "video") == "video"
        split_zone = self._zone if video and long_run else None
        tracker = PhaseTracker()
        recording = True          # clip 1 is already recording
        clip = 1
        window = 1
        in_window = True
        with self._lock:
            clip_phase = self._phase or "day"
        clip_end = None           # planned end of the clip that is recording
        clip_started = time.monotonic()
        quick_drops = 0
        while not self._stop.is_set():
            now = self._now()
            end_utc = None
            if self._deadline is not None:
                left = self._deadline - time.monotonic()
                if left <= 0:
                    break
                end_utc = now + timedelta(seconds=left)
            seg = next_segment(now, recipe, tracker, end_utc, split_zone)
            seg_s = (seg["end"] - now).total_seconds()
            with self._lock:
                self._phase = seg["phase"]
                self._next_change_utc = seg["next_change"]
            if seg["phase_changed"]:
                self._log("phase_changed", f"phase={seg['phase']}")
                logger.info("Schedule: %s phase", seg["phase"])

            if seg["kind"] == "idle":
                if recording:
                    self._stop_recording_fn()
                    recording = False
                    self._log("cycle_idle_started", f"after_window={window} phase={seg['phase']}")
                in_window = False
                self._follow_light(seg["light_pct"])
                self._wait_until("cycle_idle", seg["end"])
                continue

            if recording and clip_end is None:
                # Clip 1, started before the loop.
                clip_end, clip_phase = seg["end"], seg["phase"]
            elif not recording or now >= clip_end or seg["phase"] != clip_phase:
                if seg_s < MIN_CLIP_S:
                    # A few seconds before the next boundary: keep the clip
                    # that's recording rather than start one this short.
                    if recording:
                        clip_end = seg["end"]
                        self._wait_until("recording", seg["end"], check_disk=True)
                    else:
                        self._wait_until("starting", seg["end"])
                    continue
                if recording:
                    self._stop_recording_fn()
                    recording = False
                next_window = window if in_window else window + 1
                if not self._start_recording_with_retries(window_recipe(clip + 1, seg["phase"])):
                    if self._stop.is_set():
                        break
                    # Keep the session alive: a camera hiccup at the top of
                    # the hour shouldn't end a multi-day deployment.
                    quick_drops += 1
                    delay_s = self._restart_delay(quick_drops)
                    logger.warning("Schedule: clip %d failed to start; retrying in %ds",
                                   clip + 1, delay_s)
                    self._stop.wait(delay_s)
                    continue
                clip, window, in_window = clip + 1, next_window, True
                recording = True
                clip_started = time.monotonic()
                clip_end, clip_phase = seg["end"], seg["phase"]
                with self._lock:
                    self._clip = clip
                    self._cycle_window = window
                self._log("cycle_window_started", f"window={clip} phase={seg['phase']}")
                if not recipe.get("_manual"):
                    self._apply_recipe_led(recipe)
            # else: a clock step woke us early; the clip carries on.

            self._follow_light(seg["light_pct"])
            self._wait_until("recording", seg["end"], check_disk=True,
                             watch_recorder=True)
            ran_s = time.monotonic() - clip_started
            if self._recorder_dropped():
                # The watchdog stopped the clip (encoder exit, stall, file
                # size limit).  Start a new one rather than sit out the
                # rest of the segment; the recorder picks USB or SD again.
                quick_drops = quick_drops + 1 if ran_s < CLIP_DROP_QUICK_S else 0
                delay_s = self._restart_delay(quick_drops)
                logger.warning("Schedule: clip %d stopped after %.0fs; "
                               "starting a new clip in %ds", clip, ran_s, delay_s)
                self._log("clip_dropped", f"window={clip} ran_s={ran_s:.0f}")
                recording = False
                self._stop.wait(delay_s)
            elif ran_s >= CLIP_DROP_QUICK_S:
                quick_drops = 0

        if recording and not self._stop.is_set():
            self._stop_recording_fn()

    @staticmethod
    def _restart_delay(quick_drops):
        return min(CLIP_RESTART_DELAY_MAX_S,
                   CLIP_RESTART_DELAY_S * 2 ** max(0, quick_drops - 1))

    def _follow_light(self, pct):
        """Set the light for a segment, unless the schedule doesn't own it."""
        if pct is None:
            return
        self._light_touched = True
        self._set_light(pct)

    def _set_light(self, pct):
        with self._lock:
            if pct <= 0:
                self._light_on_since = None
            elif self._light_pct <= 0:
                self._light_on_since = time.monotonic()
            self._light_pct = pct
        if not self._hw:
            return
        if pct > 0:
            self._hw.light_on(pct)
        else:
            self._hw.light_off()

    def _disk_low(self):
        """Stop everything when free space drops below the minimum."""
        if not self._get_disk_free_fn:
            return False
        free = self._get_disk_free_fn()
        if free is None or free >= DISK_FREE_MINIMUM_MB:
            return False
        logger.warning(f"Disk space low ({free} MB), stopping recording")
        self._stop_recording_fn()
        self._set_state("disk_full")
        if self._hw:
            self._hw.led_warning()
        self._stop.set()
        return True

    def _recorder_dropped(self):
        """True when the recorder stopped without the scheduler asking."""
        return (self._is_recording_fn is not None
                and not self._stop.is_set()
                and not self._is_recording_fn())

    def _wait_until(self, state, end_utc, check_disk=False, watch_recorder=False):
        """Like _countdown, but against the wall clock, so a late RTC sync
        or other clock step moves the boundary with it. A backwards step
        that would stretch the wait returns early so the caller replans.
        With watch_recorder, also returns early if the recorder drops out."""
        planned_s = (end_utc - self._now()).total_seconds()
        while not self._stop.is_set():
            remaining = (end_utc - self._now()).total_seconds()
            self._set_state(state, max(0.0, remaining))
            if remaining <= 0 or remaining > planned_s + 5:
                break
            if check_disk and self._disk_low():
                return
            if watch_recorder and self._recorder_dropped():
                return
            self._stop.wait(min(1.0, remaining))

    def _schedule_release(self, duration_s, offset_s, recipe):
        """Schedule the closed-loop release (fast unwind at 2000 us until
        RELEASE_ROTATION_CAP rotations have been counted, or
        RELEASE_MAX_DURATION_S elapses as a safety cap) to fire at
        ``duration_s + offset_s`` seconds from now.

        - ``offset_s`` < 0  → starts that many seconds *before* the recording
          duration finishes.
        - ``offset_s`` == 0 → starts right when the recording duration ends.
        - ``offset_s`` > 0  → starts that many seconds *after* the recording
          stops.

        ``recipe`` is accepted for API symmetry but no per-recipe release
        tuning is honoured: the rotation cap and safety duration are
        bench-calibrated constants, deliberately not user-configurable.

        The timer is cancellable via ``self._stop``.
        """
        del recipe  # currently unused; retained for future per-recipe knobs
        delay_s = max(0.0, float(duration_s) + float(offset_s))
        from hardware import RELEASE_ROTATION_CAP, RELEASE_MAX_DURATION_S
        logger.info(
            f"Release scheduled: {RELEASE_ROTATION_CAP} rotations "
            f"(cap {RELEASE_MAX_DURATION_S}s) in {delay_s:.0f}s "
            f"(duration={duration_s:.0f}s, offset={offset_s:+d}s)"
        )
        self._release_thread = threading.Thread(
            target=self._release_loop,
            args=(delay_s,),
            daemon=True,
        )
        self._release_thread.start()

    def _release_loop(self, delay_s):
        if self._stop.wait(delay_s):
            return
        try:
            from hardware import (
                RELEASE_UNWIND_US, RELEASE_ROTATION_CAP, RELEASE_MAX_DURATION_S,
            )

            def _on_done(result):
                detail = (
                    f"outcome={result['outcome']} "
                    f"delivered={result['delivered']}/{result['target']} "
                    f"elapsed={result['elapsed_s']}s "
                    f"sensor_ok={result['sensor_available']}"
                )
                self._log("release_by_rotations_done", detail)

            self._log(
                "release_by_rotations_started",
                f"target={RELEASE_ROTATION_CAP} cap={RELEASE_MAX_DURATION_S}s",
            )
            self._hw.release_run_for_rotations(
                RELEASE_UNWIND_US, RELEASE_ROTATION_CAP, RELEASE_MAX_DURATION_S,
                on_complete=_on_done,
            )
        except Exception as e:
            logger.error(f"Release run failed: {e}")

    def _winch_loop(self, rotations, profiles, duration_s, start_delay_s):
        """Oscillate the release servo (unwind -> pause -> wind -> pause)
        for ``profiles`` cycles during the recording window.

        Pause windows are evenly distributed across whatever time remains
        after ``start_delay_s`` and after the deterministic motion budget
        (``profiles * 2 * leg_s``).  We honour ``start_delay_s`` first so
        the user can stage the unit, then run the profiles back-to-back.

        Always cancellable via ``self._stop``; ``hw.winch_end()`` runs in
        the finally so the pin returns to 1500 us and the rotation
        callback stops attributing edges to the winch.
        """
        from hardware import (
            WINCH_UNWIND_US, WINCH_WIND_US, WINCH_UNWIND_RPM,
            RELEASE_MAX_DURATION_S,
        )

        # Per-leg time budget.  Used both to compute pause_s and as the
        # max_duration safety cap passed to winch_run_leg — we add a 50%
        # slop so a slightly slow servo still completes its rotation
        # target before the safety cap trips.  RELEASE_MAX_DURATION_S
        # (60 s) is the absolute upper bound either way.
        rpm = max(1, WINCH_UNWIND_RPM)
        leg_s = (rotations / rpm) * 60.0
        leg_cap_s = min(RELEASE_MAX_DURATION_S, max(leg_s * 1.5, leg_s + 5.0))
        motion_s = profiles * 2 * leg_s
        usable_s = max(0.0, duration_s - start_delay_s)
        stationary_s = max(0.0, usable_s - motion_s)
        pauses = max(1, 2 * profiles)
        pause_s = stationary_s / pauses

        self._log(
            "winch_started",
            f"profiles={profiles} rotations={rotations} "
            f"leg_s={leg_s:.1f} pause_s={pause_s:.1f} "
            f"start_delay_s={start_delay_s:.0f}",
        )

        self._hw.winch_begin()
        try:
            # Stage 1: start delay (also a "pause" — sensor edges here
            # latch the error flag, same as inter-leg pauses).
            if start_delay_s > 0:
                self._hw.winch_set_pause()
                if self._stop.wait(start_delay_s):
                    return

            for i in range(profiles):
                if self._stop.is_set():
                    return
                # Unwind leg
                self._hw.winch_set_leg(+1)
                self._log("winch_unwind",
                          f"profile={i+1}/{profiles} target={rotations}")
                self._hw.winch_run_leg(WINCH_UNWIND_US, rotations,
                                       leg_cap_s, stop_event=self._stop)
                if self._stop.is_set():
                    return

                # Pause
                self._hw.winch_set_pause()
                self._log("winch_pause",
                          f"profile={i+1} after=unwind "
                          f"pause_s={pause_s:.1f}")
                if self._stop.wait(pause_s):
                    return
                if self._hw.is_winch_active() and self._hw.get_winch_state() == "error":
                    self._log(
                        "winch_pause_rotation_error",
                        f"profile={i+1} after=unwind "
                        f"turns={self._hw.get_winch_turns()}",
                    )

                if self._stop.is_set():
                    return
                # Wind leg
                self._hw.winch_set_leg(-1)
                self._log("winch_wind",
                          f"profile={i+1}/{profiles} target={rotations}")
                self._hw.winch_run_leg(WINCH_WIND_US, rotations,
                                       leg_cap_s, stop_event=self._stop)
                if self._stop.is_set():
                    return

                # Trailing pause
                self._hw.winch_set_pause()
                self._log("winch_pause",
                          f"profile={i+1} after=wind "
                          f"pause_s={pause_s:.1f}")
                if self._stop.wait(pause_s):
                    return
                if self._hw.is_winch_active() and self._hw.get_winch_state() == "error":
                    self._log(
                        "winch_pause_rotation_error",
                        f"profile={i+1} after=wind "
                        f"turns={self._hw.get_winch_turns()}",
                    )
        except Exception as e:
            logger.error(f"Winch loop failed: {e}", exc_info=True)
        finally:
            try:
                turns = self._hw.get_winch_turns()
                state = self._hw.get_winch_state()
                self._log("winch_finished",
                          f"turns={turns} final_state={state}")
            except Exception:
                pass
            try:
                self._hw.winch_end()
            except Exception:
                pass

    def _trigger_awb_once(self) -> bool:
        """Apply water WB scene, then POST onceAWB=1. Never raises."""
        scene = self.get_awb_scene()
        if not apply_awb_scene(scene):
            self._log("cam4k_awb_scene_failed", scene)
            # Still attempt one-push; scene may already be correct on camera.
        try:
            data = json.dumps(CAM4K_AWB_BODY).encode("utf-8")
            req = urllib.request.Request(
                CAM4K_AWB_URL,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=3) as resp:
                body_text = (resp.read() or b"").decode("utf-8", errors="replace").strip()
                code = None
                try:
                    code = (json.loads(body_text) or {}).get("code")
                except Exception:
                    pass
                if code is None or code == 0:
                    logger.info("4k Cam AWB (onceAWB=1) OK scene=%s", scene)
                    self._log("cam4k_awb_ok", f"onceAWB=1 scene={scene}")
                    return True
                err = f"code {code}: {body_text[:120]}"
        except urllib.error.HTTPError as e:
            err = f"HTTP {e.code}"
        except Exception as e:
            err = str(e)
        logger.warning("4k Cam AWB failed: %s", err)
        self._log("cam4k_awb_failed", err)
        return False

    def _awb_loop(self, interval_s, duration_s, night_once=False):
        """Fire AWB once now, then every interval_s for the recording window.

        With night_once the night phase gets a single one-push instead,
        NIGHT_AWB_SETTLE_S after the light first comes on; none follow until
        the day phase resumes the interval, firing straight away.

        Bounded by duration_s so a delayed release (which keeps ``_stop``
        clear after the recording ends) does not keep retriggering AWB.
        """
        logger.info("AWB loop started (interval %.0fs)", interval_s)
        deadline = time.monotonic() + float(duration_s)
        next_due = time.monotonic()
        night_done = False
        night_next_try = 0.0
        while not self._stop.is_set():
            now = time.monotonic()
            if now >= deadline:
                break
            with self._lock:
                phase, lit_since = self._phase, self._light_on_since
            if night_once and phase is None:
                pass  # schedule not running yet; don't guess day or night
            elif night_once and phase == "night":
                next_due = now
                if (not night_done and lit_since is not None
                        and now - lit_since >= NIGHT_AWB_SETTLE_S
                        and now >= night_next_try):
                    night_done = self._trigger_awb_once()
                    night_next_try = now + NIGHT_AWB_RETRY_S
                    if night_done:
                        self._log("cam4k_awb_night_hold", "no more one-push until day")
            else:
                night_done = False
                if now >= next_due:
                    self._trigger_awb_once()
                    next_due = now + interval_s
            if self._stop.wait(min(1.0, max(0.0, deadline - now))):
                break
        logger.info("AWB loop stopped")

    def _focus_sweep_loop(self, start_us, end_us, duration_s):
        """Linearly increment focus PWM from start to end over duration."""
        step_interval = 0.5
        total_steps = max(int(duration_s / step_interval), 1)
        for step in range(total_steps + 1):
            if self._stop.is_set():
                return
            t = step / total_steps
            pos = int(start_us + (end_us - start_us) * t)
            try:
                self._hw.set_aux_pwm("focus", pos)
            except Exception as e:
                logger.error(f"Focus sweep error: {e}")
            if self._stop.wait(step_interval):
                return
        logger.info("Focus sweep complete")

    def _countdown(self, state, total_s, check_disk=False):
        """Wait for total_s seconds, updating remaining time. Checks disk if requested."""
        start = time.monotonic()
        while not self._stop.is_set():
            elapsed = time.monotonic() - start
            remaining = max(0, total_s - elapsed)
            self._set_state(state, remaining)
            if remaining <= 0:
                break

            if check_disk and self._disk_low():
                return

            self._stop.wait(min(1.0, remaining))


scheduler = Scheduler()
