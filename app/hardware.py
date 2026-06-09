"""
Hardware control for DropCam: RGB LED (WS2812), camera servo, lumen light,
release servo, and auxiliary servo-style PWM outputs.

GPIO assignments:
  - GPIO 10 (Pin 19, SPI MOSI): WS2812/NeoPixel RGB status LED
  - GPIO 21 (Pin 40): Camera tilt servo (1000-2000 us PWM)
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

logger = logging.getLogger(__name__)

# GPIO pin assignments
LED_GPIO = 10
SERVO_GPIO = 21
LIGHT_GPIO = 13
RELEASE_GPIO = 12
FOCUS_GPIO = 20
ZOOM_GPIO = 26
PAN_GPIO = 16
EXT_SERVO_GPIO = 19

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

# Backwards-compat alias used by older callers — interpreted as "stop".
RELEASE_OFF_US = RELEASE_STOP_US

# NeoPixel config
LED_COUNT = 1
LED_BRIGHTNESS = 255

# Try importing hardware libraries; provide stubs if unavailable (dev machine)
_pigpio_available = False
_neopixel_available = False
_pi = None
_strip = None

try:
    import pigpio
    _pigpio_available = True
except ImportError:
    logger.warning("pigpio not available; servo/light control will be simulated")

try:
    from rpi_ws281x import PixelStrip, Color, ws
    _neopixel_available = True
except ImportError:
    logger.warning("rpi_ws281x not available; LED control will be simulated")


class HardwareController:
    def __init__(self):
        self._pi = None
        self._strip = None
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

    def init(self):
        if self._initialized:
            return
        self._init_pigpio()
        self._init_neopixel()
        self._initialized = True
        atexit.register(self.cleanup)
        logger.info("Hardware controller initialized")

    def _init_pigpio(self):
        if not _pigpio_available:
            return
        try:
            self._pi = pigpio.pi()
            if not self._pi.connected:
                logger.error("pigpio daemon not running; servo/light will be simulated")
                self._pi = None
                return
            # Drive the Lumen light to its "off" pulse (1000 us) immediately,
            # because a floating/undriven signal on this pin makes the light
            # come on at full brightness.
            try:
                self._pi.set_servo_pulsewidth(LIGHT_GPIO, SERVO_MIN_US)
            except Exception as e:
                logger.warning(f"Could not preset light off: {e}")
            # Park the release servo at its stop pulse (1500 us) on boot so
            # the continuous-rotation drive is stationary while idle.
            try:
                self._pi.set_servo_pulsewidth(RELEASE_GPIO, RELEASE_STOP_US)
            except Exception as e:
                logger.warning(f"Could not preset release servo stop: {e}")
            # Center the camera tilt servo on boot. Without this, the GPIO
            # produces no pulses until something calls set_servo(), so the
            # shaft is uncommanded and may sit at an arbitrary angle even
            # though telemetry reports the cached default of 1500 us.
            try:
                self._pi.set_servo_pulsewidth(SERVO_GPIO, SERVO_MID_US)
            except Exception as e:
                logger.warning(f"Could not preset tilt servo center: {e}")
        except Exception as e:
            logger.error(f"Failed to connect to pigpio: {e}")
            self._pi = None

    def _init_neopixel(self):
        if not _neopixel_available:
            return
        try:
            self._strip = PixelStrip(
                LED_COUNT, LED_GPIO, 800000, 10, False, LED_BRIGHTNESS, 0,
                strip_type=ws.WS2811_STRIP_RGB,
            )
            self._strip.begin()
        except Exception as e:
            logger.error(f"Failed to initialize NeoPixel: {e}")
            self._strip = None

    # ── RGB LED ──────────────────────────────────────────────────────────

    def _set_pixel(self, r, g, b):
        if self._strip:
            # Work around SPI bit-boundary bleed on GPIO 10: the LSB of each
            # transmitted byte can leak into the MSB of the next byte.  Since
            # bytes are sent R, G, B, an odd R value injects ~128 into Green
            # and an odd G value injects ~128 into Blue.  Clearing the LSB of
            # R and G prevents this with imperceptible color loss (max 1/255).
            self._strip.setPixelColor(0, Color(r & 0xFE, g & 0xFE, b))
            self._strip.show()
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

    # ── Camera Servo ─────────────────────────────────────────────────────

    def set_servo(self, position_us):
        position_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(position_us)))
        with self._lock:
            self._servo_position = position_us
        if self._pi:
            self._pi.set_servo_pulsewidth(SERVO_GPIO, position_us)
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
        if self._pi:
            self._pi.set_servo_pulsewidth(LIGHT_GPIO, pwm_us)
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
        if self._pi:
            # The Lumen light treats a disabled/0 us pulse as "full brightness",
            # so we must actively hold the minimum pulse (1000 us) to keep it off.
            self._pi.set_servo_pulsewidth(LIGHT_GPIO, SERVO_MIN_US)
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
        if self._pi:
            self._pi.set_servo_pulsewidth(RELEASE_GPIO, position_us)
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

    # ── Auxiliary Servo PWM Outputs ────────────────────────────────────

    def set_aux_pwm(self, channel, position_us):
        """Set an auxiliary PWM channel. channel is one of: focus, zoom, pan, ext_servo."""
        if channel not in AUX_PWM_GPIOS:
            raise ValueError(f"Unknown aux PWM channel: {channel}")
        position_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(position_us)))
        gpio = AUX_PWM_GPIOS[channel]
        with self._lock:
            self._aux_positions[channel] = position_us
        if self._pi:
            self._pi.set_servo_pulsewidth(gpio, position_us)
        else:
            logger.debug(f"Aux PWM sim [{channel}]: {position_us} us")

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
        if self._pi:
            self._pi.set_servo_pulsewidth(gpio, 0)
        else:
            logger.debug(f"Aux PWM sim [{channel}]: off")

    # ── Cleanup ──────────────────────────────────────────────────────────

    def cleanup(self):
        logger.info("Cleaning up hardware...")
        self._sweep_stop.set()
        self.led_off()
        if self._pi:
            try:
                self._pi.set_servo_pulsewidth(SERVO_GPIO, 0)
                # Hold the Lumen light at 1000 us (off). A 0 us / disabled
                # signal drives the light to full brightness.
                self._pi.set_servo_pulsewidth(LIGHT_GPIO, SERVO_MIN_US)
                # Hold the release servo at 1500 us (stop) so the
                # continuous-rotation drive isn't spinning at shutdown.
                self._pi.set_servo_pulsewidth(RELEASE_GPIO, RELEASE_STOP_US)
                for gpio in AUX_PWM_GPIOS.values():
                    self._pi.set_servo_pulsewidth(gpio, 0)
                self._pi.stop()
            except Exception:
                pass
        self._initialized = False


hw = HardwareController()
