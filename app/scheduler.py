"""
Scheduler for DropCam auto-start recording, duration tracking,
servo sweep orchestration, and disk space guard.

The scheduler runs in a background thread. On startup it reads the
active recipe from config and, after the configured delay, calls back
into the main app to start recording and hardware control.
"""

import logging
import threading
import time

logger = logging.getLogger(__name__)

DISK_FREE_MINIMUM_MB = 1024  # 1 GB
RECORDING_START_RETRIES = 5
RECORDING_RETRY_INTERVAL_S = 5

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
        self._hw = None
        self._state = "idle"
        self._lock = threading.Lock()
        self._remaining_s = 0
        self._focus_sweep_thread = None

    def configure(self, *, start_fn, stop_fn, disk_free_fn, hw, capture_still_fn=None):
        self._start_recording_fn = start_fn
        self._stop_recording_fn = stop_fn
        self._get_disk_free_fn = disk_free_fn
        self._capture_still_fn = capture_still_fn
        self._hw = hw

    def start(self, recipe):
        """Begin the auto-start sequence for the given recipe dict."""
        self.stop()
        self._stop.clear()
        self._active_recipe = recipe
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        logger.info(f"Scheduler started for recipe '{recipe.get('name')}'")

    def stop(self):
        """Cancel any running schedule."""
        self._stop.set()
        if self._focus_sweep_thread and self._focus_sweep_thread.is_alive():
            self._focus_sweep_thread.join(timeout=5)
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

            if recipe.get("radcam_focus_finder") and self._hw:
                zoom_us = recipe.get("focus_finder_zoom_us", 900)
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

            duration_s = recipe.get("duration_minutes", 30) * 60
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
