"""Cross-Pi GPIO backends for DropCam (Raspberry Pi 4 and Pi 5).

The Pi 5 routes all user GPIO through the RP1 I/O chip, which breaks the
register-poking libraries that work on the Pi 4:

  * pigpio      (servo/PWM)  -> hard-coded to refuse to run on a Pi 5
  * rpi_ws281x  (WS2812 LED) -> "ws2811_init failed: Hardware revision is
                                 not supported"

This module hides the two low-level needs behind small backend classes and
selects an implementation that works on the detected board:

  Servos / PWM:
    Pi 4 and earlier -> pigpio (DMA-timed, jitter-free); fall back to lgpio
    Pi 5             -> lgpio  (software-timed servo pulses via /dev/gpiochipN)

  WS2812 RGB LED (wired to GPIO 10 / SPI0 MOSI):
    Pi 4 and earlier -> rpi_ws281x; fall back to rpi5-ws2812 (SPI)
    Pi 5             -> rpi5-ws2812 (SPI, /dev/spidev0.0)

Every backend degrades to a no-op "simulation" mode when its library or
hardware is unavailable (e.g. a developer laptop), so the rest of the app
keeps running.

NOTE: lgpio servo pulses are *software* timed and have more jitter than
pigpio's DMA waveforms, so a held servo may fidget a little. For the
camera/release/light servos here that is acceptable.
"""

from __future__ import annotations

import logging
import os
import time

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

    Pi 4: pigpio (DMA, jitter-free) for everything; lgpio fallback.
    Pi 5: hardware PWM (RP1, jitter-free) for the PWM-capable pins (GPIO
          18/19), lgpio (software) for the remaining pins. If the PWM overlay
          isn't loaded, everything falls back to lgpio.
    """
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
    """Pi 4 path: rpi_ws281x on GPIO 10 (SPI MOSI), RGB wire order."""

    name = "rpi_ws281x"

    def __init__(self, gpio: int, count: int, brightness: int) -> None:
        from rpi_ws281x import Color, PixelStrip, ws

        self._Color = Color
        self._strip = PixelStrip(
            count, gpio, 800000, 10, False, brightness, 0, strip_type=ws.WS2811_STRIP_RGB
        )
        self._strip.begin()
        self.available = True

    def set_color(self, r: int, g: int, b: int) -> None:
        # SPI bit-boundary bleed workaround on GPIO 10: the LSB of each
        # transmitted byte can leak into the MSB of the next, so clear the
        # LSB of R and G (imperceptible, max 1/255 loss).
        self._strip.setPixelColor(0, self._Color(r & 0xFE, g & 0xFE, b))
        self._strip.show()

    def cleanup(self) -> None:
        try:
            self.set_color(0, 0, 0)
        except Exception:
            pass


class Ws2812SpiLedBackend(LedBackend):
    """Pi 5 (and universal) path: WS2812 over SPI (MOSI / GPIO 10).

    Self-contained SPI bit-banging driver (no external NeoPixel library, so it
    works on the image's Python 3.8). Each WS2812 data bit is encoded as one
    SPI byte clocked at 6.5 MHz: 0 -> 0b11000000, 1 -> 0b11111100. A run of
    zero bytes up front provides the >50 us reset/latch.

    Bytes are sent in R,G,B order, which matches the Pi 4 (WS2811_STRIP_RGB)
    behaviour for this physical LED. (The reset is the same encoding used by
    the rpi5-ws2812 project, validated against the hardware.)
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
        # Precompute the per-bit -> SPI-byte lookup for speed.
        self._bit = (self.LED_ZERO, self.LED_ONE)
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


def make_led_backend(gpio: int, count: int, brightness: int) -> LedBackend:
    """Pick the best available WS2812 LED backend for this board."""
    order = ["ws2812-spi"] if is_pi5() else ["rpi_ws281x", "ws2812-spi"]
    for name in order:
        try:
            if name == "rpi_ws281x":
                backend = Ws281xLedBackend(gpio, count, brightness)
            else:
                backend = Ws2812SpiLedBackend(count)
            logger.info("LED backend: %s", backend.name)
            return backend
        except Exception as exc:
            logger.warning("LED backend %s unavailable: %s", name, exc)
    logger.warning("No LED backend available; LED will be simulated")
    return LedBackend()
