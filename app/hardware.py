"""
Hardware control for DropCam: RGB LED (WS2812), camera servo, lumen light,
release servo, and auxiliary servo-style PWM outputs.

Wiring (DeckHand PCB, BR-103953 Rev A — silkscreen labels updated with
Rev A firmware to reflect the GPIO 10 / GPIO 20 role swap; PCB copper
unchanged, only the silkscreen and this file's constants moved):
  - GPIO 20 (Pin 38, SPI1 MOSI): WS2812/NeoPixel RGB status LED, driven
                      via the SPI1 peripheral. Requires
                      ``dtoverlay=spi1-1cs`` in the host config.txt so
                      ``/dev/spidev1.0`` exists and MOSI is muxed to
                      ALT4. Physically reaches the LED via J104 pin 3
                      (which the Rev A silkscreen relabels as LED_DIN;
                      earlier revs called it the rotation-sensor pin).
  - I2C1 GPIO 2/3 -> PCA9685 @ 0x40 drives all servo/PWM outputs. Channels
    are wired 1:1 to J105 header pins (silkscreen labels shown below), and
    the same driver code runs identically on Pi 4 and Pi 5:
        Ch 0 = TILT      (camera tilt servo; DropCam 1000-2000 us,
                          RadCam 865-2250 us)
        Ch 1 = LUMEN     (lumen light, 1000 us = off, 2000 us = full)
        Ch 2 = RELEASE   (continuous-rotation release drive; 1500 = stop,
                          1000 = wind, 2000 = unwind)
        Ch 3 = EXTSERVO  (external / aux servo, 1000-2000 us)
        Ch 4 = FOCUS     (RadCam focus, 870-2130 us)
        Ch 5 = ZOOM      (RadCam zoom, 935-1850 us)
        Ch 6 = PAN       (RadCam pan, 1000-2000 us)
        Ch 7 = SPARE     (unused header pin reserved for future actuators)
  - GPIO 4  (Pin 7):  PCA9685 ~OE (active-low, 10 k pull-up; held HIGH by
                      default, so we must drive it LOW at init to enable
                      outputs. Otherwise every channel stays high-Z.)
  - GPIO 10 (Pin 19): Release-shaft rotation sensor input. Produces an
                      analog 0 V -> 3.3 V ramp per rotation snapping back
                      to 0 V; read as a Schmitt-triggered digital input
                      (falling edge = one rotation) via lgpio alert. Free
                      to use as a plain GPIO because the DeckHand host
                      config uses ``dtoverlay=uart4`` (no ``ctsrts``) —
                      so CTS4 does NOT claim GPIO 10 — with a targeted
                      ``gpio=11=a4,pn`` override keeping RTS4 on GPIO 11
                      for kernel-driven RS-485 DE. Physically reaches
                      the sensor via J104 pin 4 (which the Rev A
                      silkscreen relabels as ROT_SENSOR; earlier revs
                      called this the LED pin).

Note on constant naming: the ``*_GPIO`` names below were kept for backwards
compatibility with earlier direct-PWM builds; their VALUES on the DeckHand
PCB are PCA9685 channel indices (0-15) passed through the servo backend's
``set_pulse(channel, us)`` interface.
"""

import math
import threading
import time
import logging
import atexit
from collections import deque

from gpio_backend import make_servo_backend, make_led_backend, get_gpio_line

logger = logging.getLogger(__name__)

# WS2812 status LED lives on GPIO 20 (SPI1 MOSI). The gpio_backend layer
# routes this to /dev/spidev1.0 via Ws2812SpiLedBackend, which requires
# ``dtoverlay=spi1-1cs`` on the host (managed by deckhand_host_setup.py).
# The value is passed to make_led_backend() so it can pick the correct
# SPI bus (GPIO 20 -> spidev1.0; legacy GPIO 10 -> spidev0.0 or
# rpi_ws281x); once picked, actual bit-encoding is handled in-driver.
LED_GPIO = 20

# PCA9685 output channels — see wiring table in the module docstring.
SERVO_GPIO = 0        # TILT     (camera tilt servo)
LIGHT_GPIO = 1        # LUMEN    (lumen light PWM)
RELEASE_GPIO = 2      # RELEASE  (continuous-rotation drive)
EXT_SERVO_GPIO = 3    # EXTSERVO (external aux servo)
FOCUS_GPIO = 4        # FOCUS    (RadCam focus)
ZOOM_GPIO = 5         # ZOOM     (RadCam zoom)
PAN_GPIO = 6          # PAN      (RadCam pan)
SPARE_GPIO = 7        # SPARE    (unused / reserved)

# PCA9685 output-enable pin on the Pi. Active low; a 10 k pull-up keeps
# every channel tri-stated at reset, so init() must drive this LOW before
# any servo will move.
PCA9685_OE_GPIO = 4

# Release-servo shaft rotation sensor input on the DeckHand PCB. Reaches
# the Pi via J104 pin 4 -> BCM GPIO 10. The sensor produces a slow 0 V ->
# 3.3 V ramp once per shaft rotation and snaps sharply back to 0 V;
# falling on the snap-back gives one clean, well-defined pulse per
# rotation instead of the noisy slow crossing on the way up.
#
# History: on the direct-wired prototype this was GPIO 26 (shared with
# the zoom aux output — the DeckHand PCB removed that collision by
# routing zoom to PCA9685 channel 5). Rev A of the DeckHand PCB put the
# sensor on GPIO 20 and the LED on GPIO 10. From the Rev A firmware
# release (see silkscreen relabel on J104) we swapped: the sensor moved
# to GPIO 10 (freed by dropping ``ctsrts`` from ``dtoverlay=uart4``),
# and the LED moved to GPIO 20 (SPI1 MOSI). The physical PCB copper is
# unchanged; only the silkscreen and these two constants moved.
ROTATION_SENSOR_GPIO = 10
#
# IMPORTANT — why this pin is *polled*, not interrupt/alert driven:
#   The sensor output is a slow 0 V -> 3.3 V analog ramp that snaps back to
#   0 V once per rotation. When the shaft is stationary (or the sensor is
#   floating) the line can park in the input's threshold band and chatter
#   at high frequency.  An lgpio edge *alert* raises a kernel GPIO
#   interrupt on EVERY physical edge — the ``debounce_us`` glitch filter
#   only suppresses which edges reach our Python callback, it does NOT stop
#   the interrupt.  So threshold-band chatter produced a runaway IRQ storm
#   (``irq/NN-lg`` pegging a core, lgpio daemon burning CPU) that starved
#   the whole Pi and forced reboots — invisible to any Python-side rate
#   guard because it happened below our callback.
#
#   The polled path below registers NO edge alert and therefore NO GPIO
#   interrupt.  A fixed-rate sampling thread reads the level and does edge
#   detection in software with temporal hysteresis.  CPU cost is constant
#   regardless of how noisy the line is, so an IRQ storm is structurally
#   impossible.  This is the guard against the instability we hit.
#
# Sampling rate for the polling thread.  The digital HIGH/LOW phases of the
# sawtooth are each a large fraction of the rotation period (hundreds of ms
# at operating RPM), so 500 Hz oversamples every real transition heavily
# while costing well under ~2 % of one core.
ROTATION_POLL_HZ = 500
# Internal bias for the input.  "down" parks a disconnected/failed sensor
# LOW (reads as "no rotations") rather than floating.  The live sensor
# actively drives the line so the weak pull does not affect normal reads.
ROTATION_INPUT_PULL = "down"
# Temporal-hysteresis debounce (software Schmitt trigger).  A candidate new
# level must persist for this many consecutive samples before we accept the
# transition, so brief threshold-band wobble on the slow ramp can never be
# mistaken for the clean fast snap-back.  4 samples @ 500 Hz = 8 ms stable,
# far shorter than any real HIGH/LOW dwell but far longer than chatter.
ROTATION_CONFIRM_SAMPLES = 4
# Software min-inter-edge debounce, belt-and-suspenders on top of the
# hysteresis.  At the max real RPM (~120) two rotations can arrive no
# closer than ~500 ms apart, so 250 ms rejects any duplicate/noise edge
# pair without ever discarding a real rotation.
ROTATION_MIN_INTER_EDGE_S = 0.25
ROTATION_RPM_WINDOW = 5               # smooth RPM over the last N rotations
ROTATION_RPM_STALE_S = 3.0            # no rotation in this long -> RPM = 0

# Direction-reversal phantom-edge guard.  A leg always parks the shaft the
# instant it sees the snap, i.e. right on the mark.  The next leg in the
# OPPOSITE direction re-crosses that same mark almost immediately (near-zero
# net rotation), firing one spurious "phantom" snap before any real revolution.
# That phantom lands within ~0.2 s of motion start at ANY drive speed (it is a
# tiny fixed re-cross angle), whereas the first REAL revolution takes >= ~0.54 s
# even at the fastest release (~112 RPM) and ~1.07 s at winch speed.  So on a
# reversal (or the very first move after boot) we drop the first snap ONLY if it
# arrives sooner than this guard.  A genuine first revolution — e.g. a cold
# start where the shaft was NOT parked on the mark — arrives later and is kept,
# so we never overshoot by counting one too few.
ROTATION_REVERSAL_GUARD_S = 0.4

AUX_PWM_GPIOS = {
    "focus": FOCUS_GPIO,
    "zoom": ZOOM_GPIO,
    "pan": PAN_GPIO,
    "ext_servo": EXT_SERVO_GPIO,
    "spare": SPARE_GPIO,
}

# Servo PWM range (microseconds) — DropCam tilt / lumen / release / pan
SERVO_MIN_US = 1000
SERVO_MAX_US = 2000
SERVO_MID_US = 1500

# RadCam measured actuator limits (µs). Tilt shares the TILT PCA channel but
# travels farther than a standard hobby servo; focus/zoom are lens drives.
RADCAM_TILT_MIN_US = 865
RADCAM_TILT_MAX_US = 2250
RADCAM_FOCUS_MIN_US = 870
RADCAM_FOCUS_MAX_US = 2130
RADCAM_ZOOM_MIN_US = 935
RADCAM_ZOOM_MAX_US = 1850

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
# Re-calibrated 2026-07-15 via scripts/rotation_dir_waveform.py: GPIO10 level
# sampled at 2 kHz while driving each direction, counting falling edges.
# The winch deadband is ASYMMETRIC — unwind reaches speed much sooner than
# wind, so a symmetric ±50 µs pair is badly mismatched:
#   unwind 1550 (+50 µs) -> 55.1 RPM (1.07 s/rev)
#   wind   1450 (−50 µs) -> 21.6 RPM (2.77 s/rev)   <-- far too slow
#   wind   1425 (−75 µs) -> 47.0 RPM (1.26 s/rev)
# Wind needs a larger offset from 1500 to match unwind.  Confirmed by a PWM
# sweep: wind 1415 (−85 µs) -> 55.8 RPM, matching unwind 55.1 RPM (both
# ~1.08 s/rev, comfortably under ROTATION_STALL_THRESHOLD_S).
WINCH_UNWIND_US = 1550
WINCH_WIND_US = 1415
WINCH_UNWIND_RPM = 55
WINCH_WIND_RPM = 56

# Canonical wind-finish (legacy): after an unwind move, creep in the WIND
# direction until one edge so every move ends on the same electrical stop
# angle.  This existed only to paper over the ~½ turn direction offset caused
# by counting FALLING edges in both directions (falling = clean snap when
# unwinding, but the fuzzy slow-ramp crossing when winding).  The
# direction-dependent snap detector (_rotation_snap_polarity) now keys on the
# fast snap in BOTH directions, which lands at the SAME mechanical angle, so
# unwind and wind already stop together.  The creep is therefore redundant —
# and worse, with snap-based counting it would wind a nearly full extra rev to
# reach the next wind snap — so it is disabled.
ROTATION_CANONICAL_FINISH = False
ROTATION_CANONICAL_PWM_US = None  # filled to WINCH_WIND_US at use site
ROTATION_CANONICAL_TIMEOUT_S = 5.0
ROTATION_CANONICAL_SETTLE_S = 0.15

# A rotation-counted move is only declared "stalled" (silenced sensor / jammed
# spool) if NO edge arrives within this window.  It MUST exceed the worst-case
# wait for the first legitimate edge — up to one full slow revolution when a
# move starts just after an edge.  The old 2.5 s value was SHORTER than the
# wind rev period at the mis-calibrated 1450 µs (2.77 s/rev), so wind moves
# were aborted before their first edge ever arrived (counted 0, stopped after
# ~1 coasted rev).  4.0 s clears one slow rev with margin while still catching
# a truly dead sensor well inside RELEASE_MAX_DURATION_S.
ROTATION_STALL_THRESHOLD_S = 4.0

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
        # _rotation_count is incremented from the polling thread; reads take
        # _rotation_lock for atomicity with reset_rotation_count().
        self._rotation_gpio_line = None  # gpio_backend.GpioLine, shared
        self._rotation_count = 0
        self._rotation_lock = threading.Lock()
        self._rotation_last_tick = None     # monotonic us of last accepted edge
        self._rotation_last_wall_s = 0.0
        self._rotation_intervals_us = deque(maxlen=ROTATION_RPM_WINDOW)
        self._rotation_available = False
        # Polling thread (replaces the old lgpio edge alert; see the block
        # comment on ROTATION_SENSOR_GPIO for why we no longer use an IRQ).
        self._rotation_poll_thread = None
        self._rotation_poll_stop = None       # threading.Event
        self._rotation_debounced_level = None  # last confirmed 0/1 level
        # Which GPIO transition is the clean "snap-back" for the CURRENT motor
        # direction.  The sensor is a 1:1 sawtooth (slow ramp + fast snap across
        # the dead-zone gap once per rev); the snap is a FALLING edge when
        # unwinding but a RISING edge when winding, and it always occurs at the
        # SAME mechanical angle (the gap).  Keying on the snap for each direction
        # gives an equally clean edge both ways AND makes unwind/wind stop at the
        # same angle (which is why the canonical wind-finish is no longer needed).
        #   +1 -> count FALLING edges (unwind snap)
        #   -1 -> count RISING  edges (wind snap)
        # Updated by set_release(); on stop we keep the last direction so coast
        # edges are still attributed to the correct snap polarity.
        self._rotation_snap_polarity = 1

        # Direction-reversal phantom-edge suppression.
        #
        # A rotation-counted leg always parks the shaft the instant it sees the
        # snap, i.e. right ON the mark's dead-zone gap.  The very next leg in
        # the OPPOSITE direction therefore re-crosses that same gap almost
        # immediately (near-zero net rotation) and produces one spurious snap
        # in the new polarity BEFORE any real revolution.  Left uncounted-for,
        # that phantom edge makes the first reversed leg finish one physical
        # revolution short (command 3 -> only 2 turns).  The same happens on the
        # very first leg after boot because the shaft is parked on the mark from
        # the previous session.
        #
        # We suppress it deterministically: when set_release() commands a
        # direction that differs from the last commanded move direction (or on
        # the first move, when _rotation_last_move_dir is still 0), we arm
        # _rotation_skip_next_edge so _register_rotation_edge() drops exactly the
        # next snap.  Same-direction repeats never re-cross the parked mark, so
        # they are left untouched.
        #   0  -> no move commanded yet (cold start; first move is skipped)
        #  +1  -> last commanded move was unwind
        #  -1  -> last commanded move was wind
        self._rotation_last_move_dir = 0
        self._rotation_skip_next_edge = False
        # Monotonic-us timestamp of the last commanded move, used to time-guard
        # the phantom skip (see ROTATION_REVERSAL_GUARD_S).
        self._rotation_move_start_tick = None

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
        # When True, rotation edges do not update signed winch turns
        # (used during the post-unwind canonical wind-finish home).
        self._winch_suppress_edges = False

    def init(self):
        if self._initialized:
            return
        self._init_servo()
        self._init_led()
        self._initialized = True
        atexit.register(self.cleanup)
        logger.info("Hardware controller initialized")

    def _init_servo(self):
        # Bring the servo backend up first. On the DeckHand PCB this
        # runs the PCA9685 init sequence which broadcasts ALL_LED_OFF
        # SHUT before touching PRE_SCALE, so no channel can twitch
        # during reconfig even while ~OE is still enabled.
        self._servo = make_servo_backend()
        line = get_gpio_line()

        # Preset the safety-critical channels to their idle pulse widths
        # BEFORE dropping ~OE. The PCA9685's SHUT state (set in
        # _init_chip) holds every output LOW at the pin; writing a
        # non-SHUT pulse to a specific channel clears the SHUT bit for
        # that channel only. Doing this before releasing ~OE means the
        # outputs go straight from "held LOW by ~OE" to "already at
        # their safe defaults" — the servos never see uncommanded
        # pulses.  Skipped if the backend didn't come up (sim mode).
        if self._servo.available:
            try:
                # Lumen light: 1000 us = off. A floating/undriven signal
                # on this pin makes the light come on at full brightness.
                self._servo.set_pulse(LIGHT_GPIO, SERVO_MIN_US)
            except Exception as e:
                logger.warning(f"Could not preset light off: {e}")
            try:
                # Continuous-rotation release drive: 1500 us = stop.
                self._servo.set_pulse(RELEASE_GPIO, RELEASE_STOP_US)
            except Exception as e:
                logger.warning(f"Could not preset release servo stop: {e}")
            try:
                # Camera tilt servo: 1500 us = centre.
                self._servo.set_pulse(SERVO_GPIO, SERVO_MID_US)
            except Exception as e:
                logger.warning(f"Could not preset tilt servo center: {e}")

        # Now enable the PCA9685 output stage. GPIO 4 is wired to
        # ~OE (10 k pull-up on-board), so until we drive it LOW every
        # channel is forced LOW regardless of the PWM registers.
        # Safe no-op on legacy direct-PWM boards (no ~OE line) and on
        # dev machines (get_gpio_line() returns None).
        if line is not None:
            try:
                line.claim_output(PCA9685_OE_GPIO, 0)
                logger.info(f"PCA9685 ~OE (GPIO {PCA9685_OE_GPIO}) driven LOW (outputs enabled)")
            except Exception as e:
                logger.warning(f"Could not drive PCA9685 ~OE low: {e}")

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

    def led_processing(self):
        """Slow yellow flash while post-processing (rotation metadata, file
        transfer, or a legacy TS→MP4 remux).  Yellow — not green — signals that
        the unit is busy and should not be interrupted / powered off yet."""
        self.flash_led(20, 14, 0, rate_hz=0.5)

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
        # Allow the wider RadCam tilt window; DropCam UI/recipes stay inside
        # the classic 1000-2000 µs band which is a subset of this range.
        position_us = max(RADCAM_TILT_MIN_US, min(RADCAM_TILT_MAX_US, int(position_us)))
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
        # Select the snap-back edge polarity for the commanded direction so the
        # poll loop counts the clean fast edge (falling when unwinding, rising
        # when winding).  Neutral (stop) leaves the last polarity so the shaft's
        # brief coast in the same direction is still counted correctly.
        #
        # Also arm the phantom-edge skip on a direction reversal (or the first
        # move after boot): the shaft is parked on the mark, so reversing
        # re-crosses it and fires one spurious snap before any real revolution.
        # See _rotation_last_move_dir / _rotation_skip_next_edge in __init__.
        if position_us > RELEASE_STOP_US:
            new_dir = 1
        elif position_us < RELEASE_STOP_US:
            new_dir = -1
        else:
            new_dir = 0  # stop / neutral: leave direction + skip state untouched
        if new_dir != 0:
            self._rotation_snap_polarity = new_dir
            with self._rotation_lock:
                if new_dir != self._rotation_last_move_dir:
                    self._rotation_skip_next_edge = True
                self._rotation_last_move_dir = new_dir
                self._rotation_move_start_tick = int(time.monotonic() * 1_000_000)
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
    # Counts *falling* edges on ROTATION_SENSOR_GPIO (BCM GPIO 10 on the
    # Rev-A DeckHand PCB after the silkscreen relabel; previously GPIO 20).
    # The sensor produces a slow 0 V -> 3.3 V ramp once per rotation and
    # then snaps sharply back to 0 V; the fast snap-back (HIGH -> LOW) gives
    # one clean, well-defined pulse per rotation, whereas the slow ramp up
    # through the input threshold is noisy.
    #
    # We *poll* the line rather than arm an lgpio edge alert.  See the block
    # comment on ROTATION_SENSOR_GPIO: an edge alert raises a kernel GPIO
    # interrupt on every physical edge (the debounce_us glitch filter does
    # not suppress the interrupt, only the reported callback), so a noisy /
    # floating line caused a runaway ``irq/NN-lg`` storm that starved the Pi
    # and forced reboots.  A fixed-rate polling thread with temporal
    # hysteresis registers no interrupt at all, so its CPU cost is constant
    # no matter how noisy the line gets — the storm is structurally
    # impossible.  This runs identically on Pi 4 and Pi 5 through lgpio.
    # init_rotation_sensor() is still opt-in (RadCam-mode boards may leave
    # the pin unconfigured).

    def init_rotation_sensor(self, enable=True):
        """Set up the release-shaft rotation sensor on ROTATION_SENSOR_GPIO.

        Idempotent.  When ``enable=False`` (e.g. RadCam mode where the pin
        is repurposed for something else) this is a no-op and any running
        polling thread is torn down.  Uses the shared lgpio GpioLine so it
        works identically on Pi 4 and Pi 5. Returns True if the sensor is
        live afterwards.
        """
        if not enable:
            self._teardown_rotation_sensor()
            return False
        if self._rotation_available:
            return True
        line = get_gpio_line()
        if line is None:
            logger.warning("Rotation sensor unavailable: lgpio not importable")
            return False
        try:
            # Plain input (NO edge alert -> NO GPIO interrupt).  A weak pull
            # keeps a disconnected line from floating; the live sensor drives
            # the line hard enough that the pull is irrelevant in normal use.
            line.claim_input(ROTATION_SENSOR_GPIO, pull=ROTATION_INPUT_PULL)
        except Exception as e:
            logger.warning(f"Rotation sensor init failed: {e}")
            return False
        self._rotation_gpio_line = line
        stop_event = threading.Event()
        self._rotation_poll_stop = stop_event
        thread = threading.Thread(
            target=self._rotation_poll_loop,
            args=(line, stop_event),
            name="rotation-poll",
            daemon=True,
        )
        self._rotation_poll_thread = thread
        self._rotation_available = True
        thread.start()
        confirm_ms = ROTATION_CONFIRM_SAMPLES * 1000.0 / ROTATION_POLL_HZ
        logger.info(
            f"Rotation sensor ready on GPIO {ROTATION_SENSOR_GPIO} "
            f"(polled {ROTATION_POLL_HZ} Hz, pull={ROTATION_INPUT_PULL}, "
            f"hysteresis {ROTATION_CONFIRM_SAMPLES} samples ~{confirm_ms:.0f} ms, "
            f"sw debounce {ROTATION_MIN_INTER_EDGE_S*1000:.0f} ms). "
            f"No GPIO interrupt armed — IRQ-storm-proof."
        )
        return True

    def _teardown_rotation_sensor(self):
        stop_event = self._rotation_poll_stop
        if stop_event is not None:
            stop_event.set()
        thread = self._rotation_poll_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._rotation_poll_thread = None
        self._rotation_poll_stop = None
        self._rotation_debounced_level = None
        if self._rotation_gpio_line is not None:
            try:
                self._rotation_gpio_line.release(ROTATION_SENSOR_GPIO)
            except Exception:
                pass
        self._rotation_gpio_line = None
        self._rotation_available = False

    def _rotation_poll_loop(self, line, stop_event):
        """Sample GPIO at ROTATION_POLL_HZ and detect the debounced snap-back.

        Temporal hysteresis (software Schmitt): a candidate new level must
        be read ROTATION_CONFIRM_SAMPLES times in a row before we accept the
        transition, so threshold-band wobble on the slow ramp can never be
        mistaken for the clean fast snap-back.  The snap is a FALLING edge
        when unwinding and a RISING edge when winding (see
        _rotation_snap_polarity); we count only that transition, so the slow
        ramp crossing is ignored entirely.  No interrupts are involved, so
        this loop's CPU cost is fixed regardless of line noise.
        """
        gpio = ROTATION_SENSOR_GPIO
        period = 1.0 / float(ROTATION_POLL_HZ)
        confirm = int(ROTATION_CONFIRM_SAMPLES)
        try:
            level = line.read(gpio)
        except Exception:
            level = 0
        self._rotation_debounced_level = level
        candidate = None
        candidate_run = 0
        next_t = time.monotonic()
        while not stop_event.is_set():
            next_t += period
            try:
                raw = line.read(gpio)
            except Exception:
                # Transient read failure: don't busy-spin, don't crash.
                if stop_event.wait(period):
                    break
                continue
            if raw == level:
                candidate = None
                candidate_run = 0
            else:
                if raw == candidate:
                    candidate_run += 1
                else:
                    candidate = raw
                    candidate_run = 1
                if candidate_run >= confirm:
                    prev_level = level
                    level = raw
                    self._rotation_debounced_level = level
                    candidate = None
                    candidate_run = 0
                    # Count the confirmed snap-back for the current direction:
                    # falling when unwinding (+1), rising when winding (-1).
                    # The opposite transition is the slow ramp crossing and is
                    # deliberately ignored, so it can never be double-counted.
                    if self._rotation_snap_polarity >= 0:
                        snapped = (prev_level == 1 and level == 0)
                    else:
                        snapped = (prev_level == 0 and level == 1)
                    if snapped:
                        self._register_rotation_edge(
                            int(time.monotonic() * 1_000_000)
                        )
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                if stop_event.wait(sleep_s):
                    break
            else:
                # Fell behind (scheduler hiccup); resync to avoid drift.
                next_t = time.monotonic()

    def _register_rotation_edge(self, tick_us):
        """Record one accepted rotation edge at monotonic-us ``tick_us``.

        Called from the polling thread.  Applies the software min-inter-edge
        debounce and updates the rotation count, RPM window, and (when a
        winch leg is active) the signed winch-turn counter.
        """
        with self._rotation_lock:
            # Direction-reversal phantom edge: the first snap after a change of
            # commanded direction can be the shaft re-crossing the mark it was
            # parked on (near-zero net rotation).  Drop it ONLY if it arrives
            # within the guard window; a snap that late is a genuine first
            # revolution (e.g. a cold start not parked on the mark) and must be
            # counted, otherwise we would overshoot by one.  Either way the
            # skip is disarmed after the first snap of the leg.
            if self._rotation_skip_next_edge:
                self._rotation_skip_next_edge = False
                start = self._rotation_move_start_tick
                guard_us = int(ROTATION_REVERSAL_GUARD_S * 1_000_000)
                if start is not None and (tick_us - start) < guard_us:
                    self._rotation_last_tick = tick_us
                    return
            # Software min-inter-edge debounce.  Belt-and-suspenders on top
            # of the polling hysteresis.  Real rotations are always spaced
            # >= 500 ms apart at our max operating RPM, so a 250 ms window
            # rejects any duplicate/noise edge without discarding a valid one.
            if self._rotation_last_tick is not None:
                interval_us = tick_us - self._rotation_last_tick
                if interval_us < int(ROTATION_MIN_INTER_EDGE_S * 1_000_000):
                    return
            self._rotation_count += 1
            if self._rotation_last_tick is not None:
                interval = tick_us - self._rotation_last_tick
                if 0 < interval < 60_000_000:   # sanity cap (1 min between rotations)
                    self._rotation_intervals_us.append(interval)
            self._rotation_last_tick = tick_us
            self._rotation_last_wall_s = time.monotonic()
            # Winch counter: signed turns, direction-aware.  Edges seen while
            # the scheduler thinks we're paused indicate the shaft moved
            # against the held 1500 us pulse (stall-torque overrun, line
            # tension, etc.) — count it as +1 (assumed descent) and latch the
            # error flag so the scheduler can log it.  We DO NOT abort: the
            # recipe keeps stepping through its pause->wind->pause->unwind
            # sequence as if nothing had happened.
            if self._winch_active and not self._winch_suppress_edges:
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

    def _pwm_is_unwind(self, position_us: int) -> bool:
        """True if ``position_us`` is on the unwind side of the stop band."""
        return int(position_us) > RELEASE_STOP_US

    def _pwm_is_wind(self, position_us: int) -> bool:
        return int(position_us) < RELEASE_STOP_US

    def _wait_rotation_edges(self, target_edges, max_duration_s,
                             cancel_event=None, stop_event=None,
                             stall_threshold_s=ROTATION_STALL_THRESHOLD_S):
        """Block until ``target_edges`` more sensor edges arrive (from now).

        Returns ``(outcome, delivered)`` where outcome is one of
        ``target_reached``, ``cancelled``, ``sensor_stalled``, ``timed_out``.
        """
        start_count = self.get_rotation_count()
        t_start = time.monotonic()
        deadline = t_start + float(max_duration_s)
        poll_s = 0.02
        while True:
            if cancel_event is not None and cancel_event.is_set():
                return "cancelled", self.get_rotation_count() - start_count
            if stop_event is not None and stop_event.is_set():
                return "cancelled", self.get_rotation_count() - start_count
            delivered = self.get_rotation_count() - start_count
            if delivered >= target_edges:
                return "target_reached", delivered
            now = time.monotonic()
            if delivered == 0 and (now - t_start) >= stall_threshold_s:
                return "sensor_stalled", delivered
            if now >= deadline:
                return "timed_out", delivered
            if cancel_event is not None:
                if cancel_event.wait(poll_s):
                    return "cancelled", self.get_rotation_count() - start_count
            elif stop_event is not None:
                if stop_event.wait(poll_s):
                    return "cancelled", self.get_rotation_count() - start_count
            else:
                time.sleep(poll_s)

    def _canonical_wind_finish(self, cancel_event=None, stop_event=None) -> dict:
        """Creep in the wind direction until one falling edge, then stop.

        Brings the shaft to the wind-side threshold angle so unwind and
        wind moves share a common stop pose.  Winch signed-turns are not
        updated during this home (``_winch_suppress_edges``).
        """
        if not ROTATION_CANONICAL_FINISH:
            return {"needed": False, "outcome": "disabled", "delivered": 0}
        if not self.is_rotation_sensor_available():
            return {"needed": False, "outcome": "no_sensor", "delivered": 0}

        pwm = WINCH_WIND_US if ROTATION_CANONICAL_PWM_US is None else int(
            ROTATION_CANONICAL_PWM_US
        )
        time.sleep(ROTATION_CANONICAL_SETTLE_S)
        with self._rotation_lock:
            self._winch_suppress_edges = True
        t0 = time.monotonic()
        outcome = "started"
        delivered = 0
        try:
            self.set_release(pwm)
            outcome, delivered = self._wait_rotation_edges(
                1, ROTATION_CANONICAL_TIMEOUT_S,
                cancel_event=cancel_event, stop_event=stop_event,
                stall_threshold_s=ROTATION_CANONICAL_TIMEOUT_S,
            )
        finally:
            try:
                self.set_release(RELEASE_STOP_US)
            except Exception:
                pass
            with self._rotation_lock:
                self._winch_suppress_edges = False
        elapsed = time.monotonic() - t0
        logger.info(
            "Canonical wind-finish: outcome=%s delivered=%d in %.2fs (pwm=%d)",
            outcome, delivered, elapsed, pwm,
        )
        return {
            "needed": True,
            "outcome": outcome,
            "delivered": delivered,
            "elapsed_s": round(elapsed, 2),
            "pwm_us": pwm,
        }

    def release_run_for_rotations(self, position_us, target_rotations,
                                  max_duration_s, on_complete=None):
        """Hold ``position_us`` until ``target_rotations`` falling edges of the
        rotation sensor have been observed, or ``max_duration_s`` elapses,
        whichever comes first.  Then return the shaft to stop.

        If the commanded PWM is unwind-side and ``ROTATION_CANONICAL_FINISH``
        is enabled, a short wind-direction creep follows so the shaft always
        parks on the wind threshold angle (matched stop pose both ways).

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
        sensor_ok = self.is_rotation_sensor_available()
        outcome = "started"
        t_start = time.monotonic()
        delivered = 0
        main_delivered = 0
        canonical = {"needed": False}
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
            outcome, delivered = self._wait_rotation_edges(
                target, max_duration_s,
                cancel_event=self._release_run_cancel,
            )
            main_delivered = delivered
            if outcome == "sensor_stalled":
                logger.error(
                    f"Release-by-rotations aborted: no rotation edges in "
                    f"{ROTATION_STALL_THRESHOLD_S:.1f}s at {position_us} us — "
                    f"sensor input may be silenced."
                )
            # Canonical finish: only after a successful unwind-side move.
            if (
                outcome == "target_reached"
                and self._pwm_is_unwind(position_us)
                and not self._release_run_cancel.is_set()
            ):
                canonical = self._canonical_wind_finish(
                    cancel_event=self._release_run_cancel,
                )
        finally:
            try:
                self.set_release(RELEASE_STOP_US)
            except Exception:
                pass
            with self._lock:
                self._release_run_active = False
            elapsed = time.monotonic() - t_start
            logger.info(
                f"Release-by-rotations done: outcome={outcome} "
                f"delivered={main_delivered}/{target} rotations in {elapsed:.1f}s"
                f"{' +canonical' if canonical.get('needed') else ''}"
            )
            result = {
                "outcome": outcome,
                "delivered": main_delivered,
                "target": target,
                "elapsed_s": round(elapsed, 2),
                "position_us": position_us,
                "max_duration_s": max_duration_s,
                "sensor_available": sensor_ok,
                "canonical_finish": canonical,
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
        outcome = "timeout"
        main_delivered = 0
        canonical = {"needed": False}
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
                    time.sleep(max(0.0, (t_start + max_duration_s) - time.monotonic()))
                    outcome = "no_sensor_timeout"
                return None  # populated below in finally
            out, main_delivered = self._wait_rotation_edges(
                target_rotations, max_duration_s, stop_event=stop_event,
            )
            # Map helper outcomes onto the winch leg vocabulary.
            outcome = {
                "target_reached": "target",
                "cancelled": "cancelled",
                "sensor_stalled": "sensor_stalled",
                "timed_out": "timeout",
            }.get(out, out)
            if outcome == "sensor_stalled":
                logger.error(
                    f"Winch leg aborted: no rotation edges in "
                    f"{ROTATION_STALL_THRESHOLD_S:.1f}s at {position_us} us "
                    f"(target {target_rotations} rev) — rotation "
                    f"sensor signal appears to be lost."
                )
            if (
                outcome == "target"
                and self._pwm_is_unwind(position_us)
                and not (stop_event is not None and stop_event.is_set())
            ):
                canonical = self._canonical_wind_finish(stop_event=stop_event)
        finally:
            try:
                self.set_release(RELEASE_STOP_US)
            except Exception:
                pass
        elapsed = time.monotonic() - t_start
        return {
            "outcome": outcome,
            "delivered": main_delivered,
            "target": target_rotations,
            "elapsed_s": round(elapsed, 2),
            "position_us": position_us,
            "sensor_available": sensor_ok,
            "canonical_finish": canonical,
        }

    # ── Auxiliary Servo PWM Outputs ────────────────────────────────────

    def _aux_pwm_conflicts_with_sensor(self, gpio):
        """Return True if ``gpio`` would collide with the rotation sensor.

        Historical: on the direct-wired prototype boards GPIO 26 was
        dual-purpose (RadCam zoom output / DropCam rotation sensor input),
        so every aux-PWM write consulted this guard to avoid clobbering
        the sensor. On the DeckHand PCB the rotation sensor is a Pi
        header GPIO (10 on Rev-A) and every aux channel is a PCA9685
        output (channels 0-7), so the two namespaces cannot overlap and
        this always returns False. Kept as a defensive shim so the
        aux_pwm write paths and is_aux_pwm_available() continue to work
        unchanged.
        """
        return False

    def set_aux_pwm(self, channel, position_us):
        """Set an auxiliary PWM channel. channel is one of: focus, zoom, pan, ext_servo.

        Returns True if the pin was driven, False if the call was skipped
        because of a sensor-conflict guard (the cached _aux_positions value
        is still updated so /status reflects the user's intent).
        """
        if channel not in AUX_PWM_GPIOS:
            raise ValueError(f"Unknown aux PWM channel: {channel}")
        if channel == "focus":
            lo, hi = RADCAM_FOCUS_MIN_US, RADCAM_FOCUS_MAX_US
        elif channel == "zoom":
            lo, hi = RADCAM_ZOOM_MIN_US, RADCAM_ZOOM_MAX_US
        else:
            lo, hi = SERVO_MIN_US, SERVO_MAX_US
        position_us = max(lo, min(hi, int(position_us)))
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
