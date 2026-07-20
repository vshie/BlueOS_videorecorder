"""
Scheduler for DropCam auto-start recording, duration tracking,
servo sweep orchestration, and disk space guard.

The scheduler runs in a background thread. On startup it reads the
active recipe from config and, after the configured delay, calls back
into the main app to start recording and hardware control.
"""

import json
import logging
import threading
import time
import urllib.error
import urllib.request

logger = logging.getLogger(__name__)

DISK_FREE_MINIMUM_MB = 1024  # 1 GB
RECORDING_START_RETRIES = 5
RECORDING_RETRY_INTERVAL_S = 5

# Same POSTs radcam-manager / towfish use for WB scene + one-push.
# cgi_action GET path returns "error user/pwd" on current firmware.
RADCAM_IMAGE_URL = "http://192.168.2.10/action/setImageAdjustment"
RADCAM_AWB_URL = "http://192.168.2.10/action/setImageAdjustmentEx"
RADCAM_AWB_BODY = {"onceAWB": 1}

# Water-environment AWB scene (radcam-manager Green/Blue buttons).
# Maps to camera awb_auto_mode; scene only applies with auto_awb=0 (Auto).
AWB_SCENE_MODE = {"green": 0, "blue": 1}
DEFAULT_AWB_SCENE = "blue"


def normalize_awb_scene(scene) -> str:
    s = str(scene or DEFAULT_AWB_SCENE).strip().lower()
    return s if s in AWB_SCENE_MODE else DEFAULT_AWB_SCENE


def apply_awb_scene(scene) -> bool:
    """Set RadCam Auto WB + water scene (green/blue). Never raises."""
    scene = normalize_awb_scene(scene)
    body = {
        "auto_awb": 0,  # Auto (Manual=1); scene is ignored in Manual
        "awb_auto_mode": AWB_SCENE_MODE[scene],
    }
    try:
        data = json.dumps(body).encode("utf-8")
        req = urllib.request.Request(
            RADCAM_IMAGE_URL,
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
                    "RadCam AWB scene set: %s (auto_awb=0, awb_auto_mode=%d)",
                    scene, AWB_SCENE_MODE[scene],
                )
                return True
            logger.warning("RadCam AWB scene failed: code %s: %s", code, body_text[:120])
    except urllib.error.HTTPError as e:
        logger.warning("RadCam AWB scene failed: HTTP %s", e.code)
    except Exception as e:
        logger.warning("RadCam AWB scene failed: %s", e)
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
        self._radcam_mode = False
        self._awb_scene = DEFAULT_AWB_SCENE

    def configure(self, *, start_fn, stop_fn, disk_free_fn, hw,
                  capture_still_fn=None, log_event_fn=None, radcam_mode=False,
                  awb_scene=None):
        self._start_recording_fn = start_fn
        self._stop_recording_fn = stop_fn
        self._get_disk_free_fn = disk_free_fn
        self._capture_still_fn = capture_still_fn
        # Inject main's log_event so scheduler-thread events land in the active
        # recording's events.ndjson.  Re-importing main here would load a second
        # module copy (main.py runs as __main__), whose current_events_file is
        # always None, silently dropping every winch/release event.
        self._log_event_fn = log_event_fn
        self._hw = hw
        self._radcam_mode = bool(radcam_mode)
        if awb_scene is not None:
            self._awb_scene = normalize_awb_scene(awb_scene)

    def set_radcam_mode(self, enabled):
        """Update tilt-nod extents after a late RadCam detect."""
        self._radcam_mode = bool(enabled)

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
        self._active_recipe = None

    def get_state(self):
        with self._lock:
            return {
                "state": self._state,
                "remaining_s": round(self._remaining_s, 1),
                "recipe_name": self._active_recipe["name"] if self._active_recipe else None,
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
                        radcam=self._radcam_mode,
                        return_us=start_us,
                        stop_event=self._stop,
                    )
                except Exception as e:
                    logger.warning("Pre-recipe tilt nod failed: %s", e)
            if self._stop.is_set():
                return

            self._set_state("starting")
            ok = False
            for attempt in range(1, RECORDING_START_RETRIES + 1):
                if self._stop.is_set():
                    return
                ok = self._start_recording_fn(recipe)
                if ok:
                    break
                logger.warning(f"Scheduler: recording start attempt {attempt}/{RECORDING_START_RETRIES} failed")
                if attempt < RECORDING_START_RETRIES:
                    self._stop.wait(RECORDING_RETRY_INTERVAL_S)

            if not ok:
                logger.error("Scheduler: recording failed to start after all retries")
                self._set_state("error")
                if self._hw:
                    self._hw.led_warning()
                return

            self._apply_recipe_led(recipe)

            duration_s = recipe.get("duration_minutes", 30) * 60
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
                if "radcam_focus_us" in recipe and self._hw.is_aux_pwm_available("focus"):
                    self._hw.set_aux_pwm("focus", recipe["radcam_focus_us"])
                    logger.info(f"Recipe applied focus={recipe['radcam_focus_us']} us")
                if "radcam_zoom_us" in recipe and self._hw.is_aux_pwm_available("zoom"):
                    self._hw.set_aux_pwm("zoom", recipe["radcam_zoom_us"])
                    logger.info(f"Recipe applied zoom={recipe['radcam_zoom_us']} us")

            if recipe.get("radcam_focus_finder") and self._hw:
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
            if recipe.get("radcam_awb_enable", True):
                interval_s = float(recipe.get("radcam_awb_interval_s", 120) or 120)
                interval_s = max(10.0, min(3600.0, interval_s))
                self._awb_thread = threading.Thread(
                    target=self._awb_loop,
                    args=(interval_s, duration_s),
                    name="radcam-awb-loop",
                    daemon=True,
                )
                self._awb_thread.start()
                logger.info(
                    "RadCam AWB loop started (onceAWB=1 every %.0fs for %.0fs)",
                    interval_s, duration_s,
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

            self._countdown("recording", duration_s, check_disk=True)

            if not self._stop.is_set():
                logger.info("Scheduler: duration reached, stopping recording")
                self._stop_recording_fn()
                self._set_state("complete")
                if self._hw:
                    self._hw.stop_sweep()
                    self._hw.light_off()
                    self._hw.led_complete()
            else:
                self._set_state("stopped")

        except Exception as e:
            logger.error(f"Scheduler error: {e}", exc_info=True)
            self._set_state("error")
            if self._hw:
                self._hw.led_warning()

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
            self._log("radcam_awb_scene_failed", scene)
            # Still attempt one-push; scene may already be correct on camera.
        try:
            data = json.dumps(RADCAM_AWB_BODY).encode("utf-8")
            req = urllib.request.Request(
                RADCAM_AWB_URL,
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
                    logger.info("RadCam AWB (onceAWB=1) OK scene=%s", scene)
                    self._log("radcam_awb_ok", f"onceAWB=1 scene={scene}")
                    return True
                err = f"code {code}: {body_text[:120]}"
        except urllib.error.HTTPError as e:
            err = f"HTTP {e.code}"
        except Exception as e:
            err = str(e)
        logger.warning("RadCam AWB failed: %s", err)
        self._log("radcam_awb_failed", err)
        return False

    def _awb_loop(self, interval_s, duration_s):
        """Fire AWB once now, then every interval_s for the recording window.

        Bounded by duration_s so a delayed release (which keeps ``_stop``
        clear after the recording ends) does not keep retriggering AWB.
        """
        logger.info("AWB loop started (interval %.0fs)", interval_s)
        self._trigger_awb_once()
        deadline = time.monotonic() + float(duration_s)
        while not self._stop.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            if self._stop.wait(min(interval_s, remaining)):
                break
            if time.monotonic() >= deadline:
                break
            self._trigger_awb_once()
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

            if check_disk and self._get_disk_free_fn:
                free = self._get_disk_free_fn()
                if free is not None and free < DISK_FREE_MINIMUM_MB:
                    logger.warning(f"Disk space low ({free} MB), stopping recording")
                    self._stop_recording_fn()
                    self._set_state("disk_full")
                    if self._hw:
                        self._hw.led_warning()
                    self._stop.set()
                    return

            self._stop.wait(min(1.0, remaining))


scheduler = Scheduler()
