"""Cross-Pi GPIO backends for DropCam (Raspberry Pi 4 and Pi 5).

On the DeckHand PCB all servo/PWM outputs go through a PCA9685 driven over
the primary I2C bus (GPIO 2/3), so the same code path works on the Pi 4
and Pi 5 with no board-specific PWM timing. The direct-GPIO servo
backends below (pigpio / lgpio / RP1 hardware-PWM) are kept as fallbacks
for legacy boards or dev machines without I2C hardware.

The Pi 5 routes all user GPIO through the RP1 I/O chip, which breaks the
register-poking libraries that work on the Pi 4:

  * pigpio      (servo/PWM)  -> hard-coded to refuse to run on a Pi 5
  * rpi_ws281x  (WS2812 LED) -> "ws2811_init failed: Hardware revision is
                                 not supported"

This module hides the low-level needs behind small backend classes and
selects an implementation that works on the detected board:

  Servos / PWM:
    DeckHand PCB     -> PCA9685 over I2C (Pi 4 + Pi 5, identical)
    Legacy fallback  -> pigpio (Pi 4 DMA) / lgpio / RP1 hardware-PWM (Pi 5)

  WS2812 RGB LED:
    DeckHand Rev-A firmware -> GPIO 20 / SPI1 MOSI (/dev/spidev1.0)
                               via Ws2812SpiLedBackend. Requires
                               ``dtoverlay=spi1-1cs`` on the host.
    Legacy (GPIO 10 wiring) -> Ws281xLedBackend on Pi 4 (rpi_ws281x);
                               Ws2812SpiLedBackend on /dev/spidev0.0 on Pi 5.
    Other PWM pins (12/13/18/19) -> Ws281xLedBackend if requested.

  Single-pin GPIO (OE / RS485 DE / rotation sensor input):
    Both boards      -> lgpio via GpioLine (chip auto-selected)

Every backend degrades to a no-op "simulation" mode when its library or
hardware is unavailable (e.g. a developer laptop), so the rest of the app
keeps running.

NOTE: lgpio servo pulses are *software* timed and have more jitter than
pigpio's DMA waveforms; PCA9685 is DMA-free hardware PWM and does not
share that limitation.
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any

logger = logging.getLogger(__name__)


def read_pi_model() -> str:
    """Return the Raspberry Pi model string, or '' if it can't be read."""
    for path in ("/proc/device-tree/model", "/sys/firmware/devicetree/base/model"):
        try:
            with open(path, "rb") as handle:
                return handle.read().decode("utf-8", "replace").replace("\x00", "").strip()
        except OSError:
            continue
    return ""


def is_pi5() -> bool:
    return "raspberry pi 5" in read_pi_model().lower()


# ── Servo / PWM backends ─────────────────────────────────────────────────


class ServoBackend:
    """Base / simulation servo backend (no hardware)."""

    name = "sim"
    available = False

    def set_pulse(self, gpio: int, pulse_us: int) -> None:
        logger.debug("Servo sim: gpio=%d %d us", gpio, pulse_us)

    def cleanup(self) -> None:
        pass


class PigpioServoBackend(ServoBackend):
    """Pi 4 path: pigpio daemon, DMA-timed servo pulses."""

    name = "pigpio"

    def __init__(self) -> None:
        import pigpio

        self._pi = pigpio.pi()
        if not self._pi.connected:
            raise RuntimeError("pigpio daemon not running")
        self.available = True

    def set_pulse(self, gpio: int, pulse_us: int) -> None:
        self._pi.set_servo_pulsewidth(gpio, int(pulse_us))

    def cleanup(self) -> None:
        try:
            self._pi.stop()
        except Exception:
            pass


class LgpioServoBackend(ServoBackend):
    """Pi 5 (and fallback) path: lgpio software-timed servo pulses."""

    name = "lgpio"
    SERVO_FREQ_HZ = 50

    def __init__(self) -> None:
        import lgpio

        self._lg = lgpio
        self._chip = None
        self._handle = self._open_header_chip()
        self._claimed: set[int] = set()
        self.available = True

    def _open_header_chip(self) -> int:
        """Open the gpiochip that owns the 40-pin header and return its handle.

        On a Pi 5 that is the RP1 chip (label ``pinctrl-rp1``, e.g.
        /dev/gpiochip4); on a Pi 4 it is the SoC chip with the most lines
        (/dev/gpiochip0, 54 lines). We probe each chip, prefer one whose label
        mentions RP1, and otherwise fall back to the chip exposing the most
        lines. Chips we don't keep are closed again.
        """
        chips: list[tuple[int, int, int, str]] = []  # (num, handle, lines, label)
        for num in range(8):
            try:
                handle = self._lg.gpiochip_open(num)
            except Exception:
                continue
            if handle < 0:
                continue
            lines, label = 0, ""
            try:
                info = self._lg.gpio_get_chip_info(handle)  # [status, lines, name, label]
                if isinstance(info, (list, tuple)) and len(info) >= 4:
                    lines, label = int(info[1]), str(info[3])
            except Exception:
                pass
            chips.append((num, handle, lines, label))

        if not chips:
            raise RuntimeError("no usable gpiochip found")

        chosen = next((c for c in chips if "rp1" in c[3].lower()), None)
        if chosen is None:
            chosen = max(chips, key=lambda c: c[2])

        for num, handle, _lines, _label in chips:
            if handle != chosen[1]:
                try:
                    self._lg.gpiochip_close(handle)
                except Exception:
                    pass

        self._chip = chosen[0]
        logger.info(
            "lgpio: using /dev/gpiochip%d (%s, %d lines)",
            chosen[0], chosen[3] or "?", chosen[2],
        )
        return chosen[1]

    def _claim(self, gpio: int) -> None:
        if gpio not in self._claimed:
            self._lg.gpio_claim_output(self._handle, gpio, 0)
            self._claimed.add(gpio)

    def set_pulse(self, gpio: int, pulse_us: int) -> None:
        pulse_us = int(pulse_us)
        if pulse_us <= 0:
            # Match pigpio.set_servo_pulsewidth(gpio, 0): stop sending pulses.
            try:
                self._lg.tx_servo(self._handle, gpio, 0, self.SERVO_FREQ_HZ)
            except Exception:
                pass
            return
        self._claim(gpio)
        self._lg.tx_servo(self._handle, gpio, pulse_us, self.SERVO_FREQ_HZ)

    def cleanup(self) -> None:
        for gpio in list(self._claimed):
            try:
                self._lg.tx_servo(self._handle, gpio, 0, self.SERVO_FREQ_HZ)
            except Exception:
                pass
        try:
            self._lg.gpiochip_close(self._handle)
        except Exception:
            pass


class Pca9685ServoBackend(ServoBackend):
    """DeckHand path: 16-channel PCA9685 PWM controller over I2C at 0x40.

    ``set_pulse(channel, pulse_us)`` reuses the existing pigpio/lgpio-style
    ServoBackend interface; the ``channel`` argument here is the PCA9685
    output channel (0-15) rather than a Pi GPIO number. hardware.py owns
    the mapping from function (TILT/LUMEN/RELEASE/...) to channel index.
    """

    name = "pca9685"

    def __init__(self, address: int = 0x40, bus_number: int = 1,
                 freq_hz: int = 50, ext_clock_hz: int = 24_576_000) -> None:
        from pca9685 import Pca9685

        self._pca = Pca9685(
            address=address, bus_number=bus_number,
            freq_hz=freq_hz, ext_clock_hz=ext_clock_hz,
        )
        self.available = True

    def set_pulse(self, gpio: int, pulse_us: int) -> None:
        self._pca.set_pulse(int(gpio), int(pulse_us))

    def cleanup(self) -> None:
        try:
            self._pca.all_off()
        except Exception:
            pass
        try:
            self._pca.close()
        except Exception:
            pass


# GPIOs routed to the RP1 hardware-PWM peripheral when `dtoverlay=pwm-2chan`
# is loaded on a Pi 5. These give jitter-free 50 Hz servo pulses, unlike the
# software-timed lgpio path. GPIO 18 -> PWM channel 2, GPIO 19 -> channel 3.
HW_PWM_PINS = (18, 19)


class HardwarePwmServoBackend(ServoBackend):
    """Jitter-free servo pulses via the kernel hardware-PWM (/sys/class/pwm).

    On a Pi 5 the RP1 PWM0 peripheral exposes 4 channels; with
    ``dtoverlay=pwm-2chan`` loaded, GPIO 12/13/18/19 map to channels 0/1/2/3.
    This drives a fixed 50 Hz (20 ms) period and varies the duty cycle to set
    the pulse width, which the RP1 clocks in hardware (no scheduling jitter).
    Requires a privileged container so /sys/class/pwm is writable.
    """

    name = "rp1-hw-pwm"
    PERIOD_NS = 20_000_000  # 50 Hz

    def __init__(self, pins=HW_PWM_PINS) -> None:
        self._chip_path, gpio_to_channel = self._find_chip()
        self._map = {g: gpio_to_channel[g] for g in pins if g in gpio_to_channel}
        if not self._map:
            raise RuntimeError("no hardware-PWM channels available for requested pins")
        self._exported: dict[int, int] = {}  # gpio -> channel
        self.available = True
        logger.info(
            "rp1-hw-pwm: %s, pins->channels %s", self._chip_path, self._map
        )

    @staticmethod
    def _find_chip() -> tuple[str, dict[int, int]]:
        import glob

        chips = sorted(glob.glob("/sys/class/pwm/pwmchip*"))
        # RP1 PWM0 on the Pi 5 exposes 4 channels (npwm == 4); prefer it.
        best = None
        for path in chips:
            try:
                with open(os.path.join(path, "npwm")) as handle:
                    n = int(handle.read().strip())
            except (OSError, ValueError):
                continue
            if n >= 4:
                return path, {12: 0, 13: 1, 18: 2, 19: 3}
            if n >= 2 and best is None:
                best = path
        if best is not None:
            # 2-channel chip (older overlay): pwm-2chan default maps 18->0, 19->1.
            return best, {18: 0, 19: 1}
        raise RuntimeError(
            "no hardware-PWM chip found (is 'dtoverlay=pwm-2chan' in config.txt + reboot?)"
        )

    def _channel_dir(self, channel: int) -> str:
        return os.path.join(self._chip_path, f"pwm{channel}")

    @staticmethod
    def _write(path: str, value) -> None:
        with open(path, "w") as handle:
            handle.write(f"{value}\n")

    def _ensure_exported(self, gpio: int, channel: int) -> None:
        if gpio in self._exported:
            return
        ch_dir = self._channel_dir(channel)
        if not os.path.isdir(ch_dir):
            self._write(os.path.join(self._chip_path, "export"), channel)
            # The pwmN control files can appear a moment after export.
            for _ in range(100):
                if os.path.isdir(ch_dir) and os.access(
                    os.path.join(ch_dir, "period"), os.W_OK
                ):
                    break
                time.sleep(0.01)
        # Duty must be <= period; set duty 0 first, then the fixed servo period.
        try:
            self._write(os.path.join(ch_dir, "duty_cycle"), 0)
        except OSError:
            pass
        self._write(os.path.join(ch_dir, "period"), self.PERIOD_NS)
        self._exported[gpio] = channel

    def set_pulse(self, gpio: int, pulse_us: int) -> None:
        channel = self._map.get(gpio)
        if channel is None:
            raise KeyError(f"gpio {gpio} has no hardware-PWM channel")
        ch_dir = self._channel_dir(channel)
        pulse_us = int(pulse_us)
        if pulse_us <= 0:
            if gpio in self._exported:
                try:
                    self._write(os.path.join(ch_dir, "enable"), 0)
                except OSError:
                    pass
            return
        self._ensure_exported(gpio, channel)
        duty_ns = max(0, min(int(pulse_us) * 1000, self.PERIOD_NS))
        self._write(os.path.join(ch_dir, "duty_cycle"), duty_ns)
        self._write(os.path.join(ch_dir, "enable"), 1)

    def cleanup(self) -> None:
        for gpio, channel in list(self._exported.items()):
            ch_dir = self._channel_dir(channel)
            try:
                self._write(os.path.join(ch_dir, "enable"), 0)
            except OSError:
                pass


class CompositeServoBackend(ServoBackend):
    """Route hardware-PWM pins to one backend and the rest to another."""

    def __init__(self, hw: ServoBackend, fallback: ServoBackend, hw_pins) -> None:
        self._hw = hw
        self._fallback = fallback
        self._hw_pins = set(hw_pins) if (hw and hw.available) else set()
        parts = []
        if self._hw_pins:
            parts.append(hw.name)
        if fallback and fallback.available:
            parts.append(fallback.name)
        self.name = "+".join(parts) if parts else "sim"
        self.available = bool(self._hw_pins) or bool(fallback and fallback.available)

    def set_pulse(self, gpio: int, pulse_us: int) -> None:
        if gpio in self._hw_pins:
            try:
                self._hw.set_pulse(gpio, pulse_us)
                return
            except Exception as exc:
                logger.warning("hw-pwm set_pulse(%d) failed, using fallback: %s", gpio, exc)
        if self._fallback and self._fallback.available:
            self._fallback.set_pulse(gpio, pulse_us)

    def cleanup(self) -> None:
        for backend in (self._hw, self._fallback):
            if backend:
                try:
                    backend.cleanup()
                except Exception:
                    pass


def make_servo_backend() -> ServoBackend:
    """Pick the best available servo backend for this board.

    Preferred (DeckHand PCB, both Pi 4 and Pi 5): PCA9685 over I2C. This is
    tried first because the DeckHand shield routes every servo/PWM output
    through the PCA9685 (channels 0-7 on J105) and the previous direct-GPIO
    PWM pins are physically NC on the shield.

    Legacy fallback (older direct-wired boards, or dev machines):
      Pi 4: pigpio (DMA, jitter-free); lgpio fallback.
      Pi 5: RP1 hardware PWM for GPIO 18/19 + lgpio for the rest.
      Any: simulation no-op when nothing else works.
    """
    try:
        backend = Pca9685ServoBackend()
        logger.info("Servo backend: %s", backend.name)
        return backend
    except Exception as exc:
        logger.info("PCA9685 servo backend unavailable, falling back: %s", exc)

    if not is_pi5():
        for name in ("pigpio", "lgpio"):
            try:
                backend = PigpioServoBackend() if name == "pigpio" else LgpioServoBackend()
                logger.info("Servo backend: %s", backend.name)
                return backend
            except Exception as exc:
                logger.warning("Servo backend %s unavailable: %s", name, exc)
        logger.warning("No servo backend available; servo/PWM will be simulated")
        return ServoBackend()

    # Pi 5: software lgpio for general pins ...
    try:
        lg_backend: ServoBackend = LgpioServoBackend()
    except Exception as exc:
        logger.warning("lgpio unavailable on Pi 5: %s", exc)
        lg_backend = ServoBackend()
    # ... plus hardware PWM for the jitter-sensitive PWM pins.
    try:
        hw_backend: ServoBackend = HardwarePwmServoBackend()
    except Exception as exc:
        logger.warning(
            "Hardware PWM unavailable (servos on PWM pins will use lgpio): %s", exc
        )
        backend = lg_backend
        logger.info("Servo backend: %s", backend.name)
        return backend

    composite = CompositeServoBackend(hw_backend, lg_backend, HW_PWM_PINS)
    logger.info("Servo backend: %s", composite.name)
    return composite


# ── Single-pin GPIO helper (lgpio, shared across Pi 4 / Pi 5) ────────────

class GpioLine:
    """Thin lgpio wrapper for a single output or alert-input GPIO line.

    Used by the DeckHand integration for:
      * PCA9685 ~OE   (GPIO 4, output, active-low)
      * RS-485 DE/~RE (GPIO 11, output; HIGH=transmit, LOW=receive) —
        only in the SW-toggle fallback path; the preferred path is
        kernel TIOCSRS485 which drives RTS4 (GPIO 11) itself.
      * Release-shaft rotation sensor (GPIO 10 on Rev-A silkscreen,
        alert input, falling edge)

    lgpio works on both the Pi 4 SoC GPIO chip and the Pi 5 RP1 GPIO chip
    with the same API, so this replaces the legacy pigpio callback path for
    the rotation sensor and gives us a board-agnostic OE/DE toggle.
    """

    def __init__(self) -> None:
        import lgpio

        self._lg = lgpio
        self._handle = self._open_header_chip()
        self._outputs: set[int] = set()
        self._inputs: set[int] = set()
        self._alerts: dict[int, tuple[Any, Any]] = {}  # type: ignore[name-defined]

    def _open_header_chip(self) -> int:
        """Open the gpiochip that owns the 40-pin header.

        Same probe logic as LgpioServoBackend: prefer an RP1-labeled chip
        (Pi 5), otherwise pick the chip with the most lines (Pi 4 SoC).
        """
        chips: list[tuple[int, int, int, str]] = []
        for num in range(8):
            try:
                handle = self._lg.gpiochip_open(num)
            except Exception:
                continue
            if handle < 0:
                continue
            lines, label = 0, ""
            try:
                info = self._lg.gpio_get_chip_info(handle)
                if isinstance(info, (list, tuple)) and len(info) >= 4:
                    lines, label = int(info[1]), str(info[3])
            except Exception:
                pass
            chips.append((num, handle, lines, label))

        if not chips:
            raise RuntimeError("no usable gpiochip found")

        chosen = next((c for c in chips if "rp1" in c[3].lower()), None)
        if chosen is None:
            chosen = max(chips, key=lambda c: c[2])

        for _num, handle, _lines, _label in chips:
            if handle != chosen[1]:
                try:
                    self._lg.gpiochip_close(handle)
                except Exception:
                    pass
        return chosen[1]

    def claim_output(self, gpio: int, level: int = 0) -> None:
        """Claim ``gpio`` as an output driven to ``level`` (0/1). Idempotent."""
        if gpio in self._outputs:
            self._lg.gpio_write(self._handle, gpio, 1 if level else 0)
            return
        self._lg.gpio_claim_output(self._handle, gpio, 1 if level else 0)
        self._outputs.add(gpio)

    def write(self, gpio: int, level: int) -> None:
        if gpio not in self._outputs:
            self.claim_output(gpio, level)
            return
        self._lg.gpio_write(self._handle, gpio, 1 if level else 0)

    def claim_input(self, gpio: int, pull: str = "down") -> None:
        """Claim ``gpio`` as a plain digital input for *polled* reads.

        Unlike :meth:`claim_alert_falling` this registers **no** edge alert
        and therefore no kernel GPIO interrupt, so a noisy / floating line
        can never trigger an IRQ storm (``irq/NN-lg`` pegging a core). The
        caller is expected to sample the level with :meth:`read` at a fixed
        rate and do its own edge detection / debouncing.

        ``pull`` selects the internal bias: ``"down"`` (default) parks a
        disconnected line LOW, ``"up"`` parks it HIGH, ``"none"`` leaves it
        floating. Idempotent: a re-claim frees the line first so the pull
        can change.
        """
        pull = (pull or "none").lower()
        if pull == "up":
            flags = getattr(self._lg, "SET_PULL_UP", 0)
        elif pull == "down":
            flags = getattr(self._lg, "SET_PULL_DOWN", 0)
        else:
            flags = getattr(self._lg, "SET_PULL_NONE", 0)
        if gpio in self._inputs:
            try:
                self._lg.gpio_free(self._handle, gpio)
            except Exception:
                pass
            self._inputs.discard(gpio)
        self._lg.gpio_claim_input(self._handle, gpio, flags)
        self._inputs.add(gpio)

    def read(self, gpio: int) -> int:
        """Return the current level (0/1) of an input claimed via claim_input."""
        return int(self._lg.gpio_read(self._handle, gpio))

    def claim_alert_falling(self, gpio: int, callback, debounce_us: int = 0) -> None:
        """Watch ``gpio`` for falling edges and invoke ``callback(tick_ns)``.

        ``debounce_us`` sets the lgpio glitch filter (a rising OR falling
        edge that lasts less than this window is ignored) — the equivalent
        of pigpio's ``set_glitch_filter``. lgpio callback signature is
        ``(chip, gpio, level, tick_ns)``; we forward the tick to the caller.
        """
        LG_FALLING = getattr(self._lg, "FALLING_EDGE", 1)
        self._lg.gpio_claim_alert(self._handle, gpio, LG_FALLING, 0)
        if debounce_us and hasattr(self._lg, "gpio_set_debounce_micros"):
            try:
                self._lg.gpio_set_debounce_micros(self._handle, gpio, int(debounce_us))
            except Exception as exc:
                logger.debug("debounce not supported for gpio %d: %s", gpio, exc)
        cb = self._lg.callback(self._handle, gpio, LG_FALLING, callback)
        self._alerts[gpio] = (cb, callback)

    def release(self, gpio: int) -> None:
        if gpio in self._alerts:
            cb, _fn = self._alerts.pop(gpio)
            try:
                cb.cancel()
            except Exception:
                pass
        if gpio in self._outputs:
            self._outputs.remove(gpio)
        if gpio in self._inputs:
            self._inputs.remove(gpio)
        try:
            self._lg.gpio_free(self._handle, gpio)
        except Exception:
            pass

    def close(self) -> None:
        for gpio in list(self._alerts.keys()):
            self.release(gpio)
        for gpio in list(self._outputs) + list(self._inputs):
            try:
                self._lg.gpio_free(self._handle, gpio)
            except Exception:
                pass
        try:
            self._lg.gpiochip_close(self._handle)
        except Exception:
            pass


_gpio_line_singleton: GpioLine | None = None
_gpio_line_error: Exception | None = None


def get_gpio_line() -> GpioLine | None:
    """Return a shared GpioLine, or None if lgpio isn't available.

    Callers use this to toggle OE/DE or wire an alert without each caring
    which /dev/gpiochipN to open. Failures (missing lib, no chip) are
    cached so we don't repeatedly try to import lgpio at runtime.
    """
    global _gpio_line_singleton, _gpio_line_error
    if _gpio_line_singleton is not None:
        return _gpio_line_singleton
    if _gpio_line_error is not None:
        return None
    try:
        _gpio_line_singleton = GpioLine()
        return _gpio_line_singleton
    except Exception as exc:
        _gpio_line_error = exc
        logger.info("GpioLine unavailable (single-pin GPIO will be simulated): %s", exc)
        return None


# ── WS2812 RGB LED backends ──────────────────────────────────────────────


class LedBackend:
    """Base / simulation LED backend (no hardware)."""

    name = "sim"
    available = False

    def set_color(self, r: int, g: int, b: int) -> None:
        logger.debug("LED sim: (%d,%d,%d)", r, g, b)

    def cleanup(self) -> None:
        pass


class Ws281xLedBackend(LedBackend):
    """Legacy path: rpi_ws281x on a PWM/PCM/SPI0 pin (10, 12, 13, 18, 19, 21).

    Cannot drive GPIO 20 — the rpi_ws281x library is restricted to the
    PWM/PCM peripherals plus SPI0 MOSI. On the DeckHand Rev-A wiring
    the LED lives on GPIO 20 (SPI1 MOSI), so callers should use
    :class:`Ws2812SpiLedBackend` there. This backend stays available
    for legacy direct-wired boards and for developers experimenting
    with alternative WS2812 pins on the same code base.
    """

    name = "rpi_ws281x"

    def __init__(self, gpio: int, count: int, brightness: int) -> None:
        from rpi_ws281x import Color, PixelStrip, ws

        self._Color = Color
        self._strip = PixelStrip(
            count, gpio, 800000, 10, False, brightness, 0, strip_type=ws.WS2811_STRIP_RGB
        )
        self._strip.begin()
        self._gpio = int(gpio)
        self.available = True

    def set_color(self, r: int, g: int, b: int) -> None:
        # SPI0 bit-boundary bleed workaround (GPIO 10 only): the LSB of
        # each transmitted byte can leak into the MSB of the next when
        # the library uses SPI to encode WS2812 bits. Clear the LSB of R
        # and G on that pin only (imperceptible, max 1/255 loss). On the
        # PWM/PCM pins the peripheral produces a clean waveform and
        # doesn't need the workaround.
        if self._gpio == 10:
            r &= 0xFE
            g &= 0xFE
        self._strip.setPixelColor(0, self._Color(r, g, b))
        self._strip.show()

    def cleanup(self) -> None:
        try:
            self.set_color(0, 0, 0)
        except Exception:
            pass


class Ws2812SpiLedBackend(LedBackend):
    """WS2812 driven by an SPI peripheral (MOSI produces the encoded stream).

    On the DeckHand Rev-A firmware the LED lives on **GPIO 20 = SPI1
    MOSI**, so use ``spi_bus=1, spi_device=0`` (i.e. ``/dev/spidev1.0``)
    — enabled on the host by ``dtoverlay=spi1-1cs``. Legacy boards that
    still route the LED to GPIO 10 (SPI0 MOSI) use ``spi_bus=0``.

    Self-contained SPI bit-banging driver (no external NeoPixel library,
    so it works on the image's older Python). Each WS2812 data bit is
    encoded as one SPI byte clocked at 6.5 MHz: 0 -> 0b11000000, 1 ->
    0b11111100. A run of zero bytes up front provides the >50 us
    reset/latch. Bytes are sent in R,G,B order to match the physical
    LED's WS2811_STRIP_RGB wire order.
    """

    name = "ws2812-spi"
    LED_ZERO = 0b11000000
    LED_ONE = 0b11111100
    PREAMBLE = 42  # zero bytes -> ~52 us reset at 6.5 MHz
    SPEED_HZ = 6_500_000

    def __init__(self, count: int, spi_bus: int = 0, spi_device: int = 0) -> None:
        import spidev

        self._count = max(1, int(count))
        self._spi = spidev.SpiDev()
        self._spi.open(spi_bus, spi_device)
        self._spi.max_speed_hz = self.SPEED_HZ
        self._spi.mode = 0
        self._spi.lsbfirst = False
        self._bus = spi_bus
        self._device = spi_device
        # Precompute the per-bit -> SPI-byte lookup for speed.
        self._bit = (self.LED_ZERO, self.LED_ONE)
        # Distinguish the two possible SPI buses in logs so operators can
        # tell whether the DeckHand SPI1 path is active or the legacy
        # SPI0 fallback kicked in.
        self.name = f"ws2812-spi{spi_bus}"
        self.available = True

    def _encode_byte(self, value: int) -> list[int]:
        bit = self._bit
        return [bit[(value >> i) & 1] for i in range(7, -1, -1)]

    def set_color(self, r: int, g: int, b: int) -> None:
        out = [0] * self.PREAMBLE
        for _ in range(self._count):
            out += self._encode_byte(r & 0xFF)
            out += self._encode_byte(g & 0xFF)
            out += self._encode_byte(b & 0xFF)
        writer = getattr(self._spi, "writebytes2", self._spi.writebytes)
        writer(out)

    def cleanup(self) -> None:
        try:
            self.set_color(0, 0, 0)
        except Exception:
            pass
        try:
            self._spi.close()
        except Exception:
            pass


# GPIOs that the rpi_ws281x library can drive (PWM channels + PCM + SPI0
# MOSI). Anything else must go through Ws2812SpiLedBackend.
_WS281X_CAPABLE_PINS = {10, 12, 13, 18, 19, 21}


def make_led_backend(gpio: int, count: int, brightness: int) -> LedBackend:
    """Pick the best available WS2812 LED backend for the requested pin.

    Selection matrix:

      * **GPIO 20** (DeckHand Rev-A): the WS2812 hangs off SPI1 MOSI, so
        the only working driver is :class:`Ws2812SpiLedBackend` on
        ``/dev/spidev1.0``. If the host hasn't enabled SPI1 yet (no
        ``dtoverlay=spi1-1cs``) we fall back to sim so the rest of the
        app keeps running.

      * **GPIO 10** (legacy DeckHand wiring, SPI0 MOSI): prefer
        :class:`Ws281xLedBackend` on Pi 4 (rpi_ws281x has better timing
        via DMA), fall back to :class:`Ws2812SpiLedBackend` on
        ``/dev/spidev0.0``. On Pi 5 rpi_ws281x doesn't work, so
        Ws2812SpiLedBackend is the primary path.

      * **PWM/PCM pins (12, 13, 18, 19, 21)**: use :class:`Ws281xLedBackend`
        directly (rpi_ws281x picks the correct peripheral per pin).

      * **Anything else**: sim backend.
    """
    if gpio == 20:
        try:
            backend = Ws2812SpiLedBackend(count, spi_bus=1, spi_device=0)
            logger.info("LED backend: %s (SPI1 MOSI / GPIO 20)", backend.name)
            return backend
        except Exception as exc:
            logger.warning(
                "LED backend %s unavailable (is dtoverlay=spi1-1cs in config.txt?): %s",
                "ws2812-spi1", exc,
            )
        logger.warning("No LED backend available for GPIO 20; LED will be simulated")
        return LedBackend()

    # Legacy: GPIO 10 (SPI0 MOSI) — prefer rpi_ws281x on Pi 4, SPI0 elsewhere.
    if gpio == 10:
        order = ["ws2812-spi0"] if is_pi5() else ["rpi_ws281x", "ws2812-spi0"]
        for name in order:
            try:
                if name == "rpi_ws281x":
                    backend = Ws281xLedBackend(gpio, count, brightness)
                else:
                    backend = Ws2812SpiLedBackend(count, spi_bus=0, spi_device=0)
                logger.info("LED backend: %s (legacy GPIO 10)", backend.name)
                return backend
            except Exception as exc:
                logger.warning("LED backend %s unavailable: %s", name, exc)
        logger.warning("No LED backend available for GPIO 10; LED will be simulated")
        return LedBackend()

    # PWM/PCM pins that rpi_ws281x can drive directly.
    if gpio in _WS281X_CAPABLE_PINS:
        try:
            backend = Ws281xLedBackend(gpio, count, brightness)
            logger.info("LED backend: %s (GPIO %d)", backend.name, gpio)
            return backend
        except Exception as exc:
            logger.warning("LED backend rpi_ws281x on GPIO %d unavailable: %s", gpio, exc)

    logger.warning(
        "No LED backend can drive GPIO %d; LED will be simulated "
        "(supported pins: 10 (SPI0), 12/13/18/19 (PWM), 20 (SPI1), 21 (PCM))",
        gpio,
    )
    return LedBackend()
