"""
Hardware control for DropCam: RGB LED (WS2812), camera servo, lumen light,
release servo, and auxiliary servo-style PWM outputs.

GPIO assignments:
  - GPIO 10 (Pin 19, SPI MOSI): WS2812/NeoPixel RGB status LED
  - GPIO 18 (Pin 12): Camera tilt servo (1000-2000 us PWM). On the Pi 5 this is
        driven by the RP1 hardware-PWM peripheral (jitter-free); requires
        `dtoverlay=pwm-2chan` in config.txt. (Was GPIO 21 on the Pi 4 build;
        moved to a hardware-PWM-capable pin to eliminate servo jitter.)
  - GPIO 13 (Pin 33): Lumen light (1000-2000 us servo-style PWM; 1000 us = off,
        2000 us = full brightness. Note: a disabled/floating signal turns the
        light ON at full brightness, so we always hold 1000 us to keep it off.)
  - GPIO 12 (Pin 32): Release servo on a continuous-rotation drive.
        1500 us = stop (idle), 1000 us = wind one direction, 2000 us =
        unwind the opposite direction.  The release mechanism is a string
        wound around a post: running unwind for ~60 s lets the string
        spool off and frees the unit to float to the surface.  Held at
        1500 us from boot.
  - GPIO 20 (Pin 38): Camera focus (1000-2000 us servo-style PWM)
  - GPIO 26 (Pin 37): Zoom (1000-2000 us servo-style PWM)
  - GPIO 16 (Pin 36): Pan (1000-2000 us servo-style PWM)
  - GPIO 19 (Pin 35): External servo (1000-2000 us servo-style PWM)
"""

import math
import threading
import time
import logging
import atexit
from collections import deque

from gpio_backend import make_servo_backend, make_led_backend

logger = logging.getLogger(__name__)

# GPIO pin assignments
LED_GPIO = 10
# Camera tilt servo. GPIO 18 is hardware-PWM capable on both Pi 4 and Pi 5
# (RP1), so the Pi 5 can drive it jitter-free via /sys/class/pwm. Moved here
# from GPIO 21 (which has no PWM peripheral on the RP1).
SERVO_GPIO = 18
LIGHT_GPIO = 13
RELEASE_GPIO = 12
FOCUS_GPIO = 20
ZOOM_GPIO = 26
PAN_GPIO = 16
EXT_SERVO_GPIO = 19

# Release-servo shaft rotation sensor (DropCam only).  The sensor produces an
# analog 0 V -> 3.3 V ramp once per shaft rotation, snapping back to 0 V at
# the end of each ramp.  Read as a Schmitt-triggered digital input with a
# software glitch filter so each ramp resolves to a single rising edge.
# Shares the GPIO with ZOOM_GPIO above; only one of the two roles is wired
# on a given board (DropCam = sensor, RadCam = zoom output), so the rotation
# sensor only initialises when the caller explicitly enables it.
ROTATION_SENSOR_GPIO = 26
# 100 ms glitch filter.  We trigger on the FAST snap-back (falling edge)
# from ~3.3 V to ~0 V, not the slow ramp up through the input threshold.
# Both rest levels are stable for the entire rotation period so a large
# filter is fine — it just kills threshold-band noise while the shaft is
# stationary.  Fastest real signal is the 112 RPM fast release (535 ms
# per rev), so 100 ms leaves ~50 ms of margin before we start clipping
# real edges.
ROTATION_GLITCH_FILTER_US = 100000
# Software min-inter-edge debounce, belt-and-suspenders on top of the
# hardware filter.  At the max real RPM (~120) two rotations can arrive
# no closer than ~500 ms apart, so 250 ms rejects any duplicate/noise
# edge pair without ever discarding a real rotation.
ROTATION_MIN_INTER_EDGE_S = 0.25
ROTATION_RPM_WINDOW = 5               # smooth RPM over the last N rotations
ROTATION_RPM_STALE_S = 3.0            # no rotation in this long -> RPM = 0

AUX_PWM_GPIOS = {
    "focus": FOCUS_GPIO,
    "zoom": ZOOM_GPIO,
    "pan": PAN_GPIO,
    "ext_servo": EXT_SERVO_GPIO,
}

# Servo PWM range (microseconds)
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_MID_US = 1500

# Release servo positions (microseconds).  The release uses a continuous-
# rotation drive: 1500 us holds the shaft still, 1000/2000 us spin it in
# opposite directions.  Recipe-triggered "release" runs unwind for
# RELEASE_DEFAULT_RUN_S seconds, then returns to stop.
RELEASE_STOP_US = SERVO_MID_US      # 1500 us — shaft stationary
RELEASE_WIND_US = SERVO_MIN_US      # 1000 us — winds string onto the post
RELEASE_UNWIND_US = SERVO_MAX_US    # 2000 us — releases the unit to surface
RELEASE_DEFAULT_RUN_S = 60          # how long the recipe holds unwind for

# Fast release (recipe-trigger + manual Test Release) stops after this many
# shaft rotations, with RELEASE_MAX_DURATION_S as the safety cap.  At
# RELEASE_UNWIND_US (2000 us, ~112 RPM measured), 52 rotations completes in
# ~28 s; the 60 s cap protects against sensor failure / spool jam.
RELEASE_ROTATION_CAP = 52
RELEASE_MAX_DURATION_S = 60

# PWM values for manual rotation-counted jogs + recipe winch oscillation.
# Re-calibrated 2026-06 because the original 1516/1453 pair (~24 RPM) was
# too close to the deadband edge to deliver useful torque under spool load
# — the wind direction stalled against tension.  Sweep at this PWM range
# now shows a clean step at 1555/1410 (~45 RPM) and a higher plateau at
# 1560/1400 (~52 RPM each).  We pick the 1560/1400 pair: well past the
# torque-starved deadband, well matched both directions (drift minimised),
# and roughly double the previous speed.  WINCH_*_RPM is shown in the UI
# and used by the scheduler's stationary-time preview; future closed-loop
# calibration may refine these without rewriting callers.
WINCH_UNWIND_US = 1560
WINCH_WIND_US = 1400
WINCH_UNWIND_RPM = 52
WINCH_WIND_RPM = 52

# Recipe cap on user-selected revolutions per profile leg.
WINCH_ROTATIONS_MAX = 30

# Backwards-compat alias used by older callers — interpreted as "stop".
RELEASE_OFF_US = RELEASE_STOP_US

# NeoPixel config
LED_COUNT = 1
LED_BRIGHTNESS = 255


class HardwareController:
    def __init__(self):
        # Hardware backends are selected at init() time based on the board:
        #   _servo -> lgpio (Pi 5) or pigpio (Pi 4); _led -> rpi5-ws2812 (Pi 5)
        #   or rpi_ws281x (Pi 4). Both degrade to no-op sim backends off-Pi.
        self._servo = None
        self._led = None
        self._led_thread = None
        self._led_stop = threading.Event()
        self._sweep_thread = None
        self._sweep_stop = threading.Event()
        self._servo_position = SERVO_MID_US
        self._light_brightness = 0
        self._light_on = False
        self._release_position = RELEASE_STOP_US
        self._release_run_thread = None
        self._release_run_cancel = threading.Event()
        self._release_run_active = False
        self._aux_positions = {name: SERVO_MID_US for name in AUX_PWM_GPIOS}
        self._lock = threading.Lock()
        self._initialized = False
        self._led_color = (0, 0, 0)
        self._led_mode = "off"
        # Battery-low alarm overrides every other LED state. Public LED
        # setters (led_idle, led_recording, ...) still record their desired
        # state in self._desired_led, but only drive the pixel when
        # self._battery_alarm is False. set_battery_alarm() flips the flag
        # and either takes over the LED or replays the deferred desired
        # state when cleared.
        self._battery_alarm = False
        self._desired_led = None  # tuple: (kind, r, g, b, mode, rate_hz, cycle_s)

        # Release-servo rotation sensor (initialised lazily by
        # init_rotation_sensor() once the caller knows DropCam mode).
        # _rotation_count is incremented from a pigpio callback thread; reads
        # under _rotation_lock for atomicity with reset_rotation_count().
        self._rotation_pi = None
        self._rotation_cb = None
        self._rotation_count = 0
        self._rotation_lock = threading.Lock()
        self._rotation_last_tick = None     # pigpio tick (us, 32-bit)
        self._rotation_last_wall_s = 0.0
        self._rotation_intervals_us = deque(maxlen=ROTATION_RPM_WINDOW)
        self._rotation_available = False

        # Recipe winch state.  All reads/writes share _rotation_lock so the
        # pigpio edge callback can update _winch_turns / _winch_state without
        # racing with /status readers or the scheduler's _winch_loop.
        #   _winch_active   -> True while a winch profile sequence is running
        #   _winch_direction -> +1 unwind, -1 wind, 0 pause (callback uses this
        #                       to decide how to update _winch_turns)
        #   _winch_turns    -> signed cumulative shaft rotations during this
        #                       recipe's winch run (positive = unwound /
        #                       descended, negative = wound back)
        #   _winch_state    -> 'idle' | 'unwind' | 'wind' | 'pause' | 'error'
        #   _winch_error    -> latched true after any pause-state edge; the
        #                       scheduler clears it on the next leg.
        self._winch_active = False
        self._winch_direction = 0
        self._winch_turns = 0
        self._winch_state = "idle"
        self._winch_error = False

    def init(self):
        if self._initialized:
            return
        self._init_servo()
        self._init_led()
        self._initialized = True
        atexit.register(self.cleanup)
        logger.info("Hardware controller initialized")

    def _init_servo(self):
        self._servo = make_servo_backend()
        if not self._servo.available:
            return
        # Drive the Lumen light to its "off" pulse (1000 us) immediately,
        # because a floating/undriven signal on this pin makes the light
        # come on at full brightness.
        try:
            self._servo.set_pulse(LIGHT_GPIO, SERVO_MIN_US)
        except Exception as e:
            logger.warning(f"Could not preset light off: {e}")
        # Park the release servo at its stop pulse (1500 us) on boot so
        # the continuous-rotation drive is stationary while idle.
        try:
            self._servo.set_pulse(RELEASE_GPIO, RELEASE_STOP_US)
        except Exception as e:
            logger.warning(f"Could not preset release servo stop: {e}")
        # Center the camera tilt servo on boot. Without this, the GPIO
        # produces no pulses until something calls set_servo(), so the
        # shaft is uncommanded and may sit at an arbitrary angle even
        # though telemetry reports the cached default of 1500 us.
        try:
            self._servo.set_pulse(SERVO_GPIO, SERVO_MID_US)
        except Exception as e:
            logger.warning(f"Could not preset tilt servo center: {e}")

    def _init_led(self):
        self._led = make_led_backend(LED_GPIO, LED_COUNT, LED_BRIGHTNESS)

    # ── RGB LED ──────────────────────────────────────────────────────────

    def _set_pixel(self, r, g, b):
        if self._led and self._led.available:
            self._led.set_color(r, g, b)
        else:
            logger.debug(f"LED sim: ({r},{g},{b})")

    def _stop_led_thread(self):
        self._led_stop.set()
        if self._led_thread and self._led_thread.is_alive():
            self._led_thread.join(timeout=2)

    def _drive_led(self, kind, r, g, b, mode, rate_hz=None, cycle_s=None):
        """Actually drive the pixel.  Bypasses the alarm guard."""
        self._stop_led_thread()
        with self._lock:
            self._led_color = (r, g, b)
            self._led_mode = mode
        if kind == "solid":
            self._set_pixel(r, g, b)
        elif kind == "off":
            self._set_pixel(0, 0, 0)
        elif kind == "flash":
            self._led_stop.clear()
            self._led_thread = threading.Thread(
                target=self._flash_loop, args=(r, g, b, rate_hz or 1.0), daemon=True
            )
            self._led_thread.start()
        elif kind == "breathe":
            self._led_stop.clear()
            self._led_thread = threading.Thread(
                target=self._breathe_loop, args=(r, g, b, cycle_s or 4.0), daemon=True
            )
            self._led_thread.start()

    def _request_led(self, kind, r, g, b, mode, rate_hz=None, cycle_s=None):
        """Record desired LED state, then drive it only if the battery alarm
        is not currently overriding the LED."""
        with self._lock:
            self._desired_led = (kind, r, g, b, mode, rate_hz, cycle_s)
            alarmed = self._battery_alarm
        if not alarmed:
            self._drive_led(kind, r, g, b, mode, rate_hz=rate_hz, cycle_s=cycle_s)

    def set_led_color(self, r, g, b):
        self._request_led("solid", r, g, b, "solid")

    def flash_led(self, r, g, b, rate_hz=1.0):
        mode = "flash_slow" if rate_hz <= 1.0 else "flash_fast"
        self._request_led("flash", r, g, b, mode, rate_hz=rate_hz)

    def _flash_loop(self, r, g, b, rate_hz):
        period = 1.0 / max(rate_hz, 0.1)
        half = period / 2.0
        while not self._led_stop.is_set():
            self._set_pixel(r, g, b)
            if self._led_stop.wait(half):
                break
            self._set_pixel(0, 0, 0)
            if self._led_stop.wait(half):
                break

    def breathe_led(self, r, g, b, cycle_s=4.0):
        """Smooth breathing effect: fades from off to the given color and back."""
        self._request_led("breathe", r, g, b, "breathe", cycle_s=cycle_s)

    def _breathe_loop(self, r, g, b, cycle_s):
        step_s = 0.03
        lo, hi = 10.0 / 255.0, 200.0 / 255.0
        while not self._led_stop.is_set():
            t = time.monotonic()
            phase = (t % cycle_s) / cycle_s
            unit = (math.sin(phase * 2.0 * math.pi - math.pi / 2.0) + 1.0) / 2.0
            brightness = lo + (hi - lo) * unit
            cr = int(r * brightness)
            cg = int(g * brightness)
            cb = int(b * brightness)
            self._set_pixel(cr, cg, cb)
            if self._led_stop.wait(step_s):
                break

    def led_off(self):
        self._request_led("off", 0, 0, 0, "off")

    def led_idle(self):
        """Breathing blue when idle — fades 0 to 50% over ~4 s cycle."""
        self.breathe_led(0, 0, 128, cycle_s=4.0)

    def led_recording(self):
        self.flash_led(20, 0, 0, rate_hz=0.5)

    def led_warning(self):
        self.flash_led(255, 180, 0, rate_hz=2.0)

    def led_complete(self):
        self.set_led_color(0, 0, 127)

    def led_battery_low(self):
        """6 Hz red flash for a low-battery alarm (full brightness)."""
        self._drive_led("flash", 255, 0, 0, "battery_low", rate_hz=6.0)

    def set_battery_alarm(self, active):
        """Highest-priority LED state. When True the LED flashes rapid red
        regardless of what other code requests (those requests are stored
        and replayed when the alarm clears)."""
        active = bool(active)
        with self._lock:
            if active == self._battery_alarm:
                return
            self._battery_alarm = active
            desired = self._desired_led
        if active:
            self.led_battery_low()
        else:
            if desired is None:
                # No prior request: fall back to idle so we don't leave the
                # LED on the red flash after clearing.
                self.led_idle()
            else:
                kind, r, g, b, mode, rate_hz, cycle_s = desired
                self._drive_led(kind, r, g, b, mode, rate_hz=rate_hz, cycle_s=cycle_s)

    def is_battery_alarm_active(self):
        with self._lock:
            return self._battery_alarm

    def get_led_state(self):
        """Return current LED state for telemetry."""
        with self._lock:
            return {
                "color": list(self._led_color),
                "mode": self._led_mode,
                "battery_alarm": self._battery_alarm,
            }

    def get_backend_info(self):
        """Return the active servo/LED backend names (for diagnostics)."""
        servo = getattr(self._servo, "name", None) if self._servo else None
        led = getattr(self._led, "name", None) if self._led else None
        return {
            "servo": servo,
            "servo_available": bool(self._servo and self._servo.available),
            "led": led,
            "led_available": bool(self._led and self._led.available),
        }

    # ── Camera Servo ─────────────────────────────────────────────────────

    def set_servo(self, position_us):
        position_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(position_us)))
        with self._lock:
            self._servo_position = position_us
        if self._servo and self._servo.available:
            self._servo.set_pulse(SERVO_GPIO, position_us)
        else:
            logger.debug(f"Servo sim: {position_us} us")

    def get_servo_position(self):
        with self._lock:
            return self._servo_position

    def stop_sweep(self):
        self._sweep_stop.set()
        if self._sweep_thread and self._sweep_thread.is_alive():
            self._sweep_thread.join(timeout=5)

    def sweep_servo(self, start_us, end_us, sweep_time_s, pause_points=0,
                    loiter_time_s=0, oscillations=1, light_mode="off",
                    light_brightness_pct=100, capture_still_fn=None):
        self.stop_sweep()
        self._sweep_stop.clear()
        self._sweep_thread = threading.Thread(
            target=self._sweep_loop,
            args=(start_us, end_us, sweep_time_s, pause_points,
                  loiter_time_s, oscillations, light_mode,
                  light_brightness_pct, capture_still_fn),
            daemon=True,
        )
        self._sweep_thread.start()

    def _do_pause_light(self, light_mode, light_brightness_pct, loiter_time_s,
                        capture_still_fn):
        """Handle light/snapshot behavior at a pause point."""
        if light_mode == "pause_only":
            self.light_on(light_brightness_pct)
            if self._sweep_stop.wait(loiter_time_s):
                self.light_off()
                return True
            self.light_off()
        elif light_mode == "snapshot_only":
            self.light_on(light_brightness_pct)
            if self._sweep_stop.wait(2.0):
                self.light_off()
                return True
            if capture_still_fn:
                try:
                    capture_still_fn()
                except Exception as e:
                    logger.error(f"Snapshot capture during sweep failed: {e}")
            remaining = max(0, loiter_time_s - 2.0)
            if remaining > 0:
                if self._sweep_stop.wait(remaining):
                    self.light_off()
                    return True
            if self._sweep_stop.wait(2.0):
                self.light_off()
                return True
            self.light_off()
        else:
            if self._sweep_stop.wait(loiter_time_s):
                return True
        return False

    def _sweep_loop(self, start_us, end_us, sweep_time_s, pause_points,
                    loiter_time_s, oscillations, light_mode="off",
                    light_brightness_pct=100, capture_still_fn=None):
        try:
            total_steps = max(int(sweep_time_s * 50), 10)
            step_delay = sweep_time_s / total_steps

            if pause_points > 0:
                pause_interval = total_steps // (pause_points + 1)
            else:
                pause_interval = 0

            for osc in range(max(oscillations, 1)):
                if self._sweep_stop.is_set():
                    return
                for direction in range(2):
                    if self._sweep_stop.is_set():
                        return
                    if direction == 0:
                        a, b = start_us, end_us
                    else:
                        a, b = end_us, start_us

                    for step in range(total_steps + 1):
                        if self._sweep_stop.is_set():
                            return
                        t = step / total_steps
                        pos = a + (b - a) * t
                        self.set_servo(int(pos))

                        if (pause_interval > 0 and step > 0
                                and step < total_steps
                                and step % pause_interval == 0):
                            if self._do_pause_light(light_mode, light_brightness_pct,
                                                    loiter_time_s, capture_still_fn):
                                return
                        else:
                            if self._sweep_stop.wait(step_delay):
                                return

                    # Loiter at extent
                    if loiter_time_s > 0:
                        if self._do_pause_light(light_mode, light_brightness_pct,
                                                loiter_time_s, capture_still_fn):
                            return

                    if oscillations <= 1 and direction == 0 and start_us == end_us:
                        break

            logger.info("Servo sweep complete")
        except Exception as e:
            logger.error(f"Servo sweep error: {e}")

    def is_sweeping(self):
        return self._sweep_thread is not None and self._sweep_thread.is_alive()

    # ── Lumen Light ──────────────────────────────────────────────────────

    def set_light(self, brightness_pct):
        brightness_pct = max(0, min(100, brightness_pct))
        with self._lock:
            self._light_brightness = brightness_pct
        pwm_us = SERVO_MIN_US + int((SERVO_MAX_US - SERVO_MIN_US) * brightness_pct / 100)
        if self._servo and self._servo.available:
            self._servo.set_pulse(LIGHT_GPIO, pwm_us)
        else:
            logger.debug(f"Light sim: {brightness_pct}% = {pwm_us} us")

    def get_light_brightness(self):
        with self._lock:
            return self._light_brightness

    def light_on(self, brightness_pct=None):
        with self._lock:
            self._light_on = True
            if brightness_pct is not None:
                # Honour the explicit value the caller asked for, including 0%
                # (which is effectively off — the user clearly chose it).
                self._light_brightness = max(0, min(100, brightness_pct))
            elif self._light_brightness <= 0:
                # No value passed and we have nothing remembered (e.g. after
                # light_off): fall back to full brightness so "on" actually
                # produces light.
                self._light_brightness = 100
            target = self._light_brightness
        self.set_light(target)

    def light_off(self):
        with self._lock:
            self._light_on = False
            self._light_brightness = 0
        if self._servo and self._servo.available:
            # The Lumen light treats a disabled/0 us pulse as "full brightness",
            # so we must actively hold the minimum pulse (1000 us) to keep it off.
            self._servo.set_pulse(LIGHT_GPIO, SERVO_MIN_US)
        else:
            logger.debug(f"Light sim: off ({SERVO_MIN_US} us)")

    def is_light_on(self):
        with self._lock:
            return self._light_on

    # ── Release Servo ────────────────────────────────────────────────────
    #
    # Continuous-rotation drive on RELEASE_GPIO.  1500 us holds the shaft
    # still; 1000 us winds string onto the post; 2000 us unwinds, freeing
    # the unit to float to the surface.  A recipe trigger runs unwind for
    # RELEASE_DEFAULT_RUN_S (60 s) then returns to stop.  Held at 1500 us
    # from boot (see _init_pigpio).

    def set_release(self, position_us):
        position_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(position_us)))
        with self._lock:
            self._release_position = position_us
        if self._servo and self._servo.available:
            self._servo.set_pulse(RELEASE_GPIO, position_us)
        else:
            logger.debug(f"Release sim: {position_us} us")

    def get_release_position(self):
        with self._lock:
            return self._release_position

    def get_release_direction(self):
        """Return 'winding' / 'unwinding' / 'stopped' based on the current pulse."""
        pos = self.get_release_position()
        if pos <= RELEASE_WIND_US + 50:
            return "winding"
        if pos >= RELEASE_UNWIND_US - 50:
            return "unwinding"
        return "stopped"

    def is_release_running(self):
        """Return True if a timed release run is currently in progress."""
        with self._lock:
            return self._release_run_active

    def release_stop(self):
        """Cancel any timed run and hold the shaft stationary (1500 us)."""
        self._cancel_release_run()
        self.set_release(RELEASE_STOP_US)

    # Backwards-compatible alias — older code paths still call release_off().
    release_off = release_stop

    def release_wind(self):
        """Sustained 1000 us wind.  Use release_stop() to return to idle.

        Cancels any in-progress timed unwind.
        """
        self._cancel_release_run()
        self.set_release(RELEASE_WIND_US)

    def release_unwind(self):
        """Sustained 2000 us unwind.  Use release_stop() to return to idle.

        Cancels any in-progress timed unwind.
        """
        self._cancel_release_run()
        self.set_release(RELEASE_UNWIND_US)

    def release_run_for(self, position_us, duration_s):
        """Hold ``position_us`` for ``duration_s`` seconds, then return to stop.

        Spawns a daemon thread; cancels any prior in-flight timed run.
        Returns immediately.
        """
        self._cancel_release_run()
        self._release_run_cancel.clear()
        with self._lock:
            self._release_run_active = True
        self._release_run_thread = threading.Thread(
            target=self._release_run_worker,
            args=(int(position_us), float(duration_s)),
            daemon=True,
            name="release-run",
        )
        self._release_run_thread.start()

    def _release_run_worker(self, position_us, duration_s):
        try:
            direction = ("winding" if position_us <= RELEASE_WIND_US + 50
                         else "unwinding" if position_us >= RELEASE_UNWIND_US - 50
                         else f"{position_us}us")
            logger.info(f"Release {direction} for {duration_s:.1f}s")
            self.set_release(position_us)
            cancelled = self._release_run_cancel.wait(duration_s)
            if cancelled:
                logger.info("Release timed run cancelled")
            else:
                logger.info("Release timed run finished")
        finally:
            try:
                self.set_release(RELEASE_STOP_US)
            except Exception:
                pass
            with self._lock:
                self._release_run_active = False

    def _cancel_release_run(self):
        t = self._release_run_thread
        if t and t.is_alive():
            self._release_run_cancel.set()
            t.join(timeout=2)
        with self._lock:
            self._release_run_active = False

    # ── Release-Shaft Rotation Sensor ──────────────────────────────────
    #
    # Counts *falling* edges on ROTATION_SENSOR_GPIO via a pigpio callback.
    # The sensor produces a slow 0 V -> 3.3 V ramp once per rotation and then
    # snaps sharply back to 0 V; falling on the snap-back gives one clean,
    # well-defined pulse per rotation instead of the noisy slow crossing on
    # the way up.  Combined with a 100 ms hardware glitch filter and a
    # 250 ms software min-inter-edge debounce, this is highly resistant to
    # the threshold-band chatter that used to fire spurious "pause" edges
    # while the shaft was stationary.
    #
    # The sensor and the ZOOM aux output share GPIO 26 — only one of the two
    # roles is wired on a given board, so init_rotation_sensor() is opt-in
    # and the caller is expected to skip it on RadCam-mode deployments.

    def init_rotation_sensor(self, enable=True):
        """Set up the release-shaft rotation sensor on ROTATION_SENSOR_GPIO.

        Idempotent.  When ``enable=False`` (e.g. RadCam mode where the pin is
        used as a zoom-servo output instead) this is a no-op and any prior
        callback is torn down so the pin can be driven as an output.
        Returns True if the sensor is live afterwards.
        """
        if not enable:
            self._teardown_rotation_sensor()
            return False
        if self._rotation_available:
            return True
        try:
            import pigpio
        except ImportError:
            logger.warning("Rotation sensor unavailable: pigpio not importable")
            return False
        try:
            pi = pigpio.pi()
            if not pi.connected:
                logger.warning("Rotation sensor unavailable: pigpiod not reachable")
                try: pi.stop()
                except Exception: pass
                return False
            pi.set_mode(ROTATION_SENSOR_GPIO, pigpio.INPUT)
            pi.set_pull_up_down(ROTATION_SENSOR_GPIO, pigpio.PUD_OFF)
            pi.set_glitch_filter(ROTATION_SENSOR_GPIO, ROTATION_GLITCH_FILTER_US)
            # Trigger on the fast snap-back (3.3 V -> 0 V) rather than the
            # slow ramp up through the input threshold; both rest levels
            # are stable for the full rotation period so this is much
            # more noise-tolerant than RISING_EDGE.
            cb = pi.callback(ROTATION_SENSOR_GPIO, pigpio.FALLING_EDGE,
                             self._on_rotation_edge)
        except Exception as e:
            logger.warning(f"Rotation sensor init failed: {e}")
            return False
        self._rotation_pi = pi
        self._rotation_cb = cb
        self._rotation_available = True
        logger.info(
            f"Rotation sensor ready on GPIO {ROTATION_SENSOR_GPIO} "
            f"(falling-edge, glitch filter {ROTATION_GLITCH_FILTER_US/1000:.0f} ms, "
            f"sw debounce {ROTATION_MIN_INTER_EDGE_S*1000:.0f} ms)"
        )
        return True

    def _teardown_rotation_sensor(self):
        try:
            if self._rotation_cb is not None:
                self._rotation_cb.cancel()
        except Exception:
            pass
        try:
            if self._rotation_pi is not None:
                self._rotation_pi.stop()
        except Exception:
            pass
        self._rotation_cb = None
        self._rotation_pi = None
        self._rotation_available = False

    def _on_rotation_edge(self, gpio, level, tick):
        # Runs in a pigpio callback thread.  Keep this short.
        # tick is a uint32 microsecond counter; use tickDiff to handle wrap.
        import pigpio
        with self._rotation_lock:
            # Software min-inter-edge debounce.  Belt-and-suspenders on top
            # of the pigpio glitch filter; catches any noise pair that
            # cleared the hardware filter (eg two 100+ ms plateaus in a
            # noisy threshold window).  Real rotations are always spaced
            # >= 500 ms apart at our max operating RPM, so a 250 ms window
            # rejects duplicates without ever discarding a valid edge.
            if self._rotation_last_tick is not None:
                interval_us = pigpio.tickDiff(self._rotation_last_tick, tick)
                if interval_us < int(ROTATION_MIN_INTER_EDGE_S * 1_000_000):
                    return
            self._rotation_count += 1
            if self._rotation_last_tick is not None:
                interval = pigpio.tickDiff(self._rotation_last_tick, tick)
                if 0 < interval < 60_000_000:   # sanity cap (1 min between rotations)
                    self._rotation_intervals_us.append(interval)
            self._rotation_last_tick = tick
            self._rotation_last_wall_s = time.monotonic()
            # Winch counter: signed turns, direction-aware.  Edges seen while
            # the scheduler thinks we're paused indicate the shaft moved
            # against the held 1500 us pulse (stall-torque overrun, line
            # tension, etc.) — count it as +1 (assumed descent) and latch the
            # error flag so the scheduler can log it.  We DO NOT abort: the
            # recipe keeps stepping through its pause->wind->pause->unwind
            # sequence as if nothing had happened.
            if self._winch_active:
                if self._winch_direction > 0:
                    self._winch_turns += 1
                elif self._winch_direction < 0:
                    self._winch_turns -= 1
                else:
                    self._winch_turns += 1
                    self._winch_state = "error"
                    self._winch_error = True

    def is_rotation_sensor_available(self):
        return self._rotation_available

    def get_rotation_count(self):
        with self._rotation_lock:
            return self._rotation_count

    def reset_rotation_count(self):
        with self._rotation_lock:
            self._rotation_count = 0
            self._rotation_last_tick = None
            self._rotation_last_wall_s = 0.0
            self._rotation_intervals_us.clear()

    def get_rotation_rpm(self):
        """Mean RPM over the most recent ROTATION_RPM_WINDOW intervals.

        Returns 0.0 if no rotation has been seen in the last
        ROTATION_RPM_STALE_S seconds (shaft considered stopped).
        """
        with self._rotation_lock:
            if not self._rotation_intervals_us or self._rotation_last_wall_s == 0:
                return 0.0
            if (time.monotonic() - self._rotation_last_wall_s) > ROTATION_RPM_STALE_S:
                return 0.0
            mean_us = sum(self._rotation_intervals_us) / len(self._rotation_intervals_us)
        if mean_us <= 0:
            return 0.0
        return 60_000_000.0 / mean_us

    def release_run_for_rotations(self, position_us, target_rotations,
                                  max_duration_s, on_complete=None):
        """Hold ``position_us`` until ``target_rotations`` rising edges of the
        rotation sensor have been observed, or ``max_duration_s`` elapses,
        whichever comes first.  Then return the shaft to stop.

        Uses a count *delta* captured at start so the global rotation counter
        (used by /status and the subtitle overlay) is not disturbed.  Spawns
        a daemon thread and returns immediately.  Cancels any prior in-flight
        release run.

        ``on_complete`` is an optional callable invoked with the result dict
        from the worker thread (same shape as get_last_release_rotation_result).
        It runs in the worker thread so it must be quick / non-blocking.
        """
        target_rotations = int(target_rotations)
        max_duration_s = float(max_duration_s)
        if target_rotations <= 0:
            raise ValueError("target_rotations must be > 0")
        if max_duration_s <= 0:
            raise ValueError("max_duration_s must be > 0")
        self._cancel_release_run()
        self._release_run_cancel.clear()
        with self._lock:
            self._release_run_active = True
        self._release_run_thread = threading.Thread(
            target=self._release_run_worker_rotations,
            args=(int(position_us), target_rotations, max_duration_s, on_complete),
            daemon=True,
            name="release-run-rotations",
        )
        self._release_run_thread.start()

    def _release_run_worker_rotations(self, position_us, target, max_duration_s,
                                       on_complete=None):
        """Worker for release_run_for_rotations.  Emits structured info-logs
        at start and end that callers (main.py) can forward to events.ndjson.
        """
        start_count = self.get_rotation_count()
        sensor_ok = self.is_rotation_sensor_available()
        outcome = "started"
        t_start = time.monotonic()
        deadline = t_start + max_duration_s
        delivered = 0
        try:
            if not sensor_ok:
                logger.warning(
                    "Rotation sensor not available — falling back to timed "
                    f"release for {max_duration_s:.1f}s at {position_us}us"
                )
                self.set_release(position_us)
                cancelled = self._release_run_cancel.wait(max_duration_s)
                outcome = "cancelled" if cancelled else "no_sensor_timeout"
                return
            logger.info(
                f"Release-by-rotations: target={target} rotations at "
                f"{position_us} us, safety cap {max_duration_s:.1f}s"
            )
            self.set_release(position_us)
            # Poll every 20 ms — gives ~0.04 rotation precision at 112 RPM.
            poll_s = 0.02
            # Same sensor-stall guard as the winch path.  At fast release
            # (~112 RPM) the first edge should arrive within ~0.5 s, so
            # 2.5 s of zero edges almost certainly means the input is
            # being clobbered (eg GPIO 26 collision with the zoom aux
            # output).  Abort instead of running the full safety cap.
            stall_threshold_s = 2.5
            while True:
                if self._release_run_cancel.is_set():
                    outcome = "cancelled"
                    break
                delivered = self.get_rotation_count() - start_count
                if delivered >= target:
                    outcome = "target_reached"
                    break
                now = time.monotonic()
                if delivered == 0 and (now - t_start) >= stall_threshold_s:
                    outcome = "sensor_stalled"
                    logger.error(
                        f"Release-by-rotations aborted: no rotation edges "
                        f"in {stall_threshold_s:.1f}s at {position_us} us "
                        f"— sensor input may be silenced."
                    )
                    break
                if now >= deadline:
                    outcome = "timed_out"
                    break
                if self._release_run_cancel.wait(poll_s):
                    outcome = "cancelled"
                    break
        finally:
            try:
                self.set_release(RELEASE_STOP_US)
            except Exception:
                pass
            with self._lock:
                self._release_run_active = False
            elapsed = time.monotonic() - t_start
            delivered = self.get_rotation_count() - start_count
            logger.info(
                f"Release-by-rotations done: outcome={outcome} "
                f"delivered={delivered}/{target} rotations in {elapsed:.1f}s"
            )
            # Stash the last-run result for callers to surface in events.ndjson.
            result = {
                "outcome": outcome,
                "delivered": delivered,
                "target": target,
                "elapsed_s": round(elapsed, 2),
                "position_us": position_us,
                "max_duration_s": max_duration_s,
                "sensor_available": sensor_ok,
            }
            self._last_release_rotation_result = result
            if on_complete is not None:
                try:
                    on_complete(result)
                except Exception as e:
                    logger.warning(f"release_run_for_rotations on_complete failed: {e}")

    def get_last_release_rotation_result(self):
        """Last finished release-by-rotations run, or None if none yet."""
        return getattr(self, "_last_release_rotation_result", None)

    # ── Recipe Winch (vertical-profile oscillation) ────────────────────
    #
    # Public API used by scheduler._winch_loop.  The scheduler owns the
    # leg sequencing (unwind N -> pause -> wind N -> pause); this layer
    # provides the per-leg motion primitive plus the signed turns counter
    # and 'idle/unwind/wind/pause/error' state read by /status, the ASS
    # subtitle overlay, and events.ndjson.

    def winch_begin(self):
        """Mark the start of a recipe winch run: zero turns, clear error,
        flag the rotation callback to start tracking signed turns."""
        with self._rotation_lock:
            self._winch_active = True
            self._winch_direction = 0
            self._winch_turns = 0
            self._winch_state = "idle"
            self._winch_error = False

    def winch_end(self):
        """End the recipe winch run: stop the pin and clear active flag.
        Leaves _winch_turns / _winch_state readable for final reporting."""
        try:
            self.set_release(RELEASE_STOP_US)
        except Exception:
            pass
        with self._rotation_lock:
            self._winch_active = False
            self._winch_direction = 0
            self._winch_state = "idle"

    def winch_set_leg(self, direction):
        """Set the upcoming-leg direction (+1 unwind / -1 wind) and clear the
        latched error flag so a fresh pause-overrun condition can be detected
        on the next pause.  Does NOT drive the servo — winch_run_leg() does."""
        with self._rotation_lock:
            self._winch_direction = 1 if direction > 0 else -1
            self._winch_state = "unwind" if direction > 0 else "wind"
            self._winch_error = False

    def winch_set_pause(self):
        """Enter the pause phase: direction 0 so any sensor edge will be
        flagged as an overrun, state 'pause' for telemetry."""
        try:
            self.set_release(RELEASE_STOP_US)
        except Exception:
            pass
        with self._rotation_lock:
            self._winch_direction = 0
            self._winch_state = "pause"

    def get_winch_turns(self):
        with self._rotation_lock:
            return self._winch_turns

    def get_winch_state(self):
        with self._rotation_lock:
            return self._winch_state

    def is_winch_active(self):
        with self._rotation_lock:
            return self._winch_active

    def winch_run_leg(self, position_us, target_rotations, max_duration_s,
                      stop_event=None):
        """BLOCKING: drive ``position_us`` until ``target_rotations`` more
        sensor edges have been seen on top of the count at entry, or
        ``max_duration_s`` elapses, or ``stop_event`` is set.  Always
        returns the pin to 1500 us before returning.

        Caller (the scheduler's _winch_loop) is responsible for having
        called winch_set_leg(direction) first so the rotation callback
        signs the turns correctly.

        Returns a dict: {outcome: target|timeout|cancelled,
                         delivered, elapsed_s, position_us}.
        """
        target_rotations = int(target_rotations)
        max_duration_s = float(max_duration_s)
        if target_rotations <= 0:
            raise ValueError("target_rotations must be > 0")
        if max_duration_s <= 0:
            raise ValueError("max_duration_s must be > 0")
        # Snapshot the GLOBAL count delta so the winch counter (which moves
        # signed) doesn't matter for the per-leg target check.
        start_count = self.get_rotation_count()
        sensor_ok = self.is_rotation_sensor_available()
        t_start = time.monotonic()
        deadline = t_start + max_duration_s
        outcome = "timeout"
        try:
            self.set_release(position_us)
            if not sensor_ok:
                # No sensor: degrade to a timed wait so the recipe still has
                # rough motion.  Scheduler will pick this up via outcome.
                logger.warning(
                    "Winch leg falling back to timed motion (no sensor); "
                    f"holding {position_us} us for {max_duration_s:.1f}s"
                )
                if stop_event is not None and stop_event.wait(max_duration_s):
                    outcome = "cancelled"
                else:
                    time.sleep(max(0.0, deadline - time.monotonic()))
                    outcome = "no_sensor_timeout"
                return None  # populated below in finally
            # Sensor-stall guard.  If the rotation sensor reports zero
            # edges within this many seconds of starting the leg the input
            # is almost certainly being clobbered (the GPIO 26 / "zoom"
            # collision that turned 3-rev legs into 9-rev runaways).
            # Abort early instead of running the full leg_cap_s budget.
            # 2.5 s is generous enough for healthy operation at every RPM
            # the winch supports (>= ~24 RPM -> first edge by 2.5 s).
            stall_threshold_s = 2.5
            poll_s = 0.02
            while True:
                if stop_event is not None and stop_event.is_set():
                    outcome = "cancelled"
                    break
                delivered = self.get_rotation_count() - start_count
                if delivered >= target_rotations:
                    outcome = "target"
                    break
                now = time.monotonic()
                if delivered == 0 and (now - t_start) >= stall_threshold_s:
                    outcome = "sensor_stalled"
                    logger.error(
                        f"Winch leg aborted: no rotation edges in "
                        f"{stall_threshold_s:.1f}s at {position_us} us "
                        f"(target {target_rotations} rev) — sensor input "
                        f"may be silenced (eg GPIO 26 collision with zoom)."
                    )
                    break
                if now >= deadline:
                    outcome = "timeout"
                    break
                if stop_event is not None:
                    if stop_event.wait(poll_s):
                        outcome = "cancelled"
                        break
                else:
                    time.sleep(poll_s)
        finally:
            try:
                self.set_release(RELEASE_STOP_US)
            except Exception:
                pass
        delivered = self.get_rotation_count() - start_count
        elapsed = time.monotonic() - t_start
        return {
            "outcome": outcome,
            "delivered": delivered,
            "target": target_rotations,
            "elapsed_s": round(elapsed, 2),
            "position_us": position_us,
            "sensor_available": sensor_ok,
        }

    # ── Auxiliary Servo PWM Outputs ────────────────────────────────────

    def _aux_pwm_conflicts_with_sensor(self, gpio):
        """Return True if ``gpio`` is the active rotation-sensor input pin.

        GPIO 26 is dual-purpose (RadCam zoom servo output / DropCam release
        rotation sensor input).  When the rotation sensor is initialised,
        driving the pin as a servo would override the input and silence the
        callback — which is exactly the bug that turned 3-rev winch legs
        into 7-9 rev runaways.  This guard is consulted by every aux-PWM
        write path so an over-eager recipe or stray API call cannot
        re-clobber the pin while the sensor is active.
        """
        return gpio == ROTATION_SENSOR_GPIO and self._rotation_available

    def set_aux_pwm(self, channel, position_us):
        """Set an auxiliary PWM channel. channel is one of: focus, zoom, pan, ext_servo.

        Returns True if the pin was driven, False if the call was skipped
        because of a sensor-conflict guard (the cached _aux_positions value
        is still updated so /status reflects the user's intent).
        """
        if channel not in AUX_PWM_GPIOS:
            raise ValueError(f"Unknown aux PWM channel: {channel}")
        position_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(position_us)))
        gpio = AUX_PWM_GPIOS[channel]
        with self._lock:
            self._aux_positions[channel] = position_us
        if self._aux_pwm_conflicts_with_sensor(gpio):
            logger.warning(
                f"Skipping aux PWM '{channel}' (GPIO {gpio} = {position_us} us) "
                f"— rotation sensor is using this pin in DropCam mode."
            )
            return False
        if self._servo and self._servo.available:
            self._servo.set_pulse(gpio, position_us)
        else:
            logger.debug(f"Aux PWM sim [{channel}]: {position_us} us")
        return True

    def get_aux_pwm(self, channel):
        if channel not in AUX_PWM_GPIOS:
            raise ValueError(f"Unknown aux PWM channel: {channel}")
        with self._lock:
            return self._aux_positions[channel]

    def get_all_aux_pwm(self):
        with self._lock:
            return dict(self._aux_positions)

    def aux_pwm_off(self, channel):
        """Stop sending pulses on an auxiliary channel (sets pulse width to 0)."""
        if channel not in AUX_PWM_GPIOS:
            raise ValueError(f"Unknown aux PWM channel: {channel}")
        gpio = AUX_PWM_GPIOS[channel]
        if self._aux_pwm_conflicts_with_sensor(gpio):
            logger.warning(
                f"Skipping aux_pwm_off '{channel}' (GPIO {gpio}) "
                f"— rotation sensor is using this pin."
            )
            return False
        if self._servo and self._servo.available:
            self._servo.set_pulse(gpio, 0)
        else:
            logger.debug(f"Aux PWM sim [{channel}]: off")
        return True

    def is_aux_pwm_available(self, channel):
        """Return True if writing to ``channel`` will reach the pin.

        Lets callers (eg the scheduler) skip noisy recipe-driven warnings
        for channels that are physically unwired in the current build
        (DropCam blocks zoom because the pin reads the rotation sensor).
        """
        if channel not in AUX_PWM_GPIOS:
            return False
        return not self._aux_pwm_conflicts_with_sensor(AUX_PWM_GPIOS[channel])

    # ── Cleanup ──────────────────────────────────────────────────────────

    def cleanup(self):
        logger.info("Cleaning up hardware...")
        self._sweep_stop.set()
        try:
            self.winch_end()
        except Exception:
            pass
        self._teardown_rotation_sensor()
        self.led_off()
        if self._led:
            try:
                self._led.cleanup()
            except Exception:
                pass
        if self._servo and self._servo.available:
            try:
                self._servo.set_pulse(SERVO_GPIO, 0)
                # Hold the Lumen light at 1000 us (off). A 0 us / disabled
                # signal drives the light to full brightness.
                self._servo.set_pulse(LIGHT_GPIO, SERVO_MIN_US)
                # Hold the release servo at 1500 us (stop) so the
                # continuous-rotation drive isn't spinning at shutdown.
                self._servo.set_pulse(RELEASE_GPIO, RELEASE_STOP_US)
                for gpio in AUX_PWM_GPIOS.values():
                    self._servo.set_pulse(gpio, 0)
            except Exception:
                pass
            try:
                self._servo.cleanup()
            except Exception:
                pass
        self._initialized = False


hw = HardwareController()
