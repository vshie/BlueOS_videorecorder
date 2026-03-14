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


class Scheduler:
    def __init__(self):
        self._thread = None
        self._stop = threading.Event()
        self._active_recipe = None
        self._start_recording_fn = None
        self._stop_recording_fn = None
        self._get_disk_free_fn = None
        self._hw = None
        self._state = "idle"
        self._lock = threading.Lock()
        self._remaining_s = 0

    def configure(self, *, start_fn, stop_fn, disk_free_fn, hw):
        self._start_recording_fn = start_fn
        self._stop_recording_fn = stop_fn
        self._get_disk_free_fn = disk_free_fn
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
            ok = self._start_recording_fn(recipe)
            if not ok:
                logger.error("Scheduler: recording failed to start")
                self._set_state("error")
                if self._hw:
                    self._hw.led_warning()
                return

            if self._hw:
                if not recipe.get("servo_fixed", True):
                    self._hw.sweep_servo(
                        recipe.get("servo_start_us", 1500),
                        recipe.get("servo_end_us", 1500),
                        recipe.get("servo_sweep_time_s", 30),
                        recipe.get("servo_pause_points", 0),
                        recipe.get("servo_loiter_time_s", 0),
                        recipe.get("servo_oscillations", 1),
                    )
                else:
                    self._hw.set_servo(recipe.get("servo_start_us", 1500))

                if recipe.get("light_on", False):
                    self._hw.light_on(recipe.get("light_brightness_pct", 100))
                else:
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
            logger.error(f"Scheduler error: {e}")
            self._set_state("error")

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
