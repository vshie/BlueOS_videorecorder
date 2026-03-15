"""
Hardware control for DropCam: RGB LED (WS2812), camera servo, and lumen light.

GPIO assignments:
  - GPIO 10 (Pin 19, SPI MOSI): WS2812/NeoPixel RGB status LED
  - GPIO 21 (Pin 40): Camera tilt servo (1000-2000 us PWM)
  - GPIO 13 (Pin 33): Lumen light (1000-2000 us servo-style PWM)
"""

import threading
import time
import logging
import atexit

logger = logging.getLogger(__name__)

# GPIO pin assignments
LED_GPIO = 10
SERVO_GPIO = 21
LIGHT_GPIO = 13

# Servo PWM range (microseconds)
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_MID_US = 1500

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
        self._lock = threading.Lock()
        self._initialized = False
        self._led_color = (0, 0, 0)
        self._led_mode = "off"

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
        except Exception as e:
            logger.error(f"Failed to connect to pigpio: {e}")
            self._pi = None

    def _init_neopixel(self):
        if not _neopixel_available:
            return
        try:
            self._strip = PixelStrip(
                LED_COUNT, LED_GPIO, 800000, 10, False, LED_BRIGHTNESS, 0,
                strip_type=ws.WS2811_STRIP_GRB,
            )
            self._strip.begin()
        except Exception as e:
            logger.error(f"Failed to initialize NeoPixel: {e}")
            self._strip = None

    # ── RGB LED ──────────────────────────────────────────────────────────

    def set_led_color(self, r, g, b):
        self._led_stop.set()
        if self._led_thread and self._led_thread.is_alive():
            self._led_thread.join(timeout=2)
        with self._lock:
            self._led_color = (r, g, b)
            self._led_mode = "solid"
        self._set_pixel(r, g, b)

    def _set_pixel(self, r, g, b):
        if self._strip:
            self._strip.setPixelColor(0, Color(r, g, b))
            self._strip.show()
        else:
            logger.debug(f"LED sim: ({r},{g},{b})")

    def flash_led(self, r, g, b, rate_hz=1.0):
        self._led_stop.set()
        if self._led_thread and self._led_thread.is_alive():
            self._led_thread.join(timeout=2)
        with self._lock:
            self._led_color = (r, g, b)
            self._led_mode = "flash_slow" if rate_hz <= 1.0 else "flash_fast"
        self._led_stop.clear()
        self._led_thread = threading.Thread(
            target=self._flash_loop, args=(r, g, b, rate_hz), daemon=True
        )
        self._led_thread.start()

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

    def led_off(self):
        self._led_stop.set()
        if self._led_thread and self._led_thread.is_alive():
            self._led_thread.join(timeout=2)
        with self._lock:
            self._led_color = (0, 0, 0)
            self._led_mode = "off"
        self._set_pixel(0, 0, 0)

    def led_idle(self):
        """Solid green when idle. Reduced intensity avoids yellow tint on WS2812B."""
        self.set_led_color(0, 180, 15)

    def led_recording(self):
        self.flash_led(255, 0, 0, rate_hz=0.5)

    def led_warning(self):
        self.flash_led(255, 180, 0, rate_hz=2.0)

    def led_complete(self):
        self.set_led_color(0, 0, 255)

    def get_led_state(self):
        """Return current LED state for telemetry."""
        with self._lock:
            return {
                "color": list(self._led_color),
                "mode": self._led_mode,
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
                self._light_brightness = max(0, min(100, brightness_pct))
            b = self._light_brightness
        self.set_light(b if b > 0 else 100)

    def light_off(self):
        with self._lock:
            self._light_on = False
        if self._pi:
            self._pi.set_servo_pulsewidth(LIGHT_GPIO, 0)
        else:
            logger.debug("Light sim: off")

    def is_light_on(self):
        with self._lock:
            return self._light_on

    # ── Cleanup ──────────────────────────────────────────────────────────

    def cleanup(self):
        logger.info("Cleaning up hardware...")
        self._sweep_stop.set()
        self.led_off()
        if self._pi:
            try:
                self._pi.set_servo_pulsewidth(SERVO_GPIO, 0)
                self._pi.set_servo_pulsewidth(LIGHT_GPIO, 0)
                self._pi.stop()
            except Exception:
                pass
        self._initialized = False


hw = HardwareController()
