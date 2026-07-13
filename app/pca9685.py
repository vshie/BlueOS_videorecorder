"""Minimal PCA9685 16-channel PWM driver for the DeckHand shield.

The DeckHand PCB wires a PCA9685PW at I2C address 0x40 on the Pi's primary
I2C bus (GPIO 2/3) and clocks it from an external 24.576 MHz oscillator
(pin 25, EXTCLK) rather than the chip's internal 25 MHz oscillator. This
driver:

  * writes MODE1/MODE2 for external-clock, totem-pole output
  * computes and writes the PRE_SCALE register for a fixed 50 Hz frame
    (standard servo update rate), correcting for the external clock
  * exposes ``set_pulse(channel, us)`` which maps microseconds into the
    12-bit ON/OFF counter registers

Address pins A0-A5 are tied to GND on the board, so 0x40 is fixed.

Servo channel usage on this board (labelled by function on J105):
    0=TILT   1=LUMEN   2=RELEASE 3=EXTSERVO
    4=FOCUS  5=ZOOM    6=PAN     7=SPARE
Channels 8-15 are unconnected.
"""

from __future__ import annotations

import logging
import math
import time

logger = logging.getLogger(__name__)


DEFAULT_ADDRESS = 0x40
DEFAULT_I2C_BUS = 1

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06
ALL_LED_OFF_H = 0xFD

MODE1_RESTART = 0x80
MODE1_EXTCLK = 0x40
MODE1_AI = 0x20
MODE1_SLEEP = 0x10

MODE2_OUTDRV = 0x04  # totem-pole outputs (also implies OUTNE=00, so ~OE
                     # HIGH -> outputs LOW; safe default while reconfiguring)

# LEDn_OFF_H / ALL_LED_OFF_H bit 4 = "full OFF" (channel forced LOW,
# regardless of the 12-bit counter). LEDn_ON_H / ALL_LED_ON_H bit 4 =
# "full ON" (channel forced HIGH). These are the canonical 0%/100% duty
# encodings from the datasheet §7.3.3 — used both by us and by
# ArduPilot's RCOutput_PCA9685.
FULL_BIT = 0x1000

DEFAULT_FREQ_HZ = 50
EXT_CLOCK_HZ = 24_576_000
STEPS = 4096


class Pca9685:
    """Small blocking PCA9685 driver over python-smbus2.

    Instantiating this class talks to the chip immediately (probe + reset),
    so callers should catch exceptions and fall back to a simulation
    backend when the board or bus is not present.
    """

    def __init__(
        self,
        address: int = DEFAULT_ADDRESS,
        bus_number: int = DEFAULT_I2C_BUS,
        freq_hz: int = DEFAULT_FREQ_HZ,
        ext_clock_hz: int = EXT_CLOCK_HZ,
    ) -> None:
        from smbus2 import SMBus  # imported lazily so laptops without smbus2 still load

        self._SMBus = SMBus
        self.address = address
        self.bus_number = bus_number
        self.freq_hz = int(freq_hz)
        self.ext_clock_hz = int(ext_clock_hz)
        self._bus = SMBus(bus_number)
        try:
            self._init_chip()
        except Exception:
            try:
                self._bus.close()
            except Exception:
                pass
            raise

    def _write(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(self.address, reg, value & 0xFF)

    def _read(self, reg: int) -> int:
        return self._bus.read_byte_data(self.address, reg) & 0xFF

    def _init_chip(self) -> None:
        """Bring the PCA9685 up on the external 24.576 MHz clock at 50 Hz.

        Sequence per datasheet §7.3.1.1 (switching to EXTCLK) with the
        belt-and-suspenders "SHUT all outputs first" recommendation from
        §7.3.3 so no channel can emit a transient pulse while we retime
        the counter:

          1. Broadcast ALL_LED_OFF full-OFF so every channel is held LOW
             at the output pin regardless of its stale count registers.
          2. Put chip to sleep (SLEEP=1) to unlock PRE_SCALE + EXTCLK.
          3. Write MODE1 = SLEEP | EXTCLK to select the external clock
             (mandated single-step switchover; EXTCLK can only be
             cleared by a power cycle or software reset after this).
          4. Program PRE_SCALE for the target servo frequency using the
             external clock.
          5. Restore MODE1 with RESTART | AI | EXTCLK: clears SLEEP,
             restarts any paused PWM counter (§7.3.1.1, no-op on cold
             boot since RESTART was 0), and enables auto-increment for
             burst writes.
          6. Wait 500 µs for the oscillator to stabilise (datasheet
             §7.3.1.1) — we use 600 µs for margin.
          7. Set MODE2 = OUTDRV (totem-pole; also leaves OUTNE=00, so
             driving ~OE HIGH forces outputs LOW). This matches the
             RN101/RN102 series-resistor pattern for 3.3 V servo signals.

        Channels remain in the full-OFF state from step 1 until callers
        explicitly write ``set_pulse(ch, us)`` — that write clears the
        SHUT bit on that channel alone. This lets hardware.py preset
        safe pulse widths before ~OE is dropped.
        """
        self._shut_all_outputs()
        self._write(MODE1, MODE1_SLEEP)
        self._write(MODE1, MODE1_SLEEP | MODE1_EXTCLK)
        prescale = self._prescale_value(self.freq_hz, self.ext_clock_hz)
        self._write(PRESCALE, prescale)
        self._write(MODE1, MODE1_RESTART | MODE1_AI | MODE1_EXTCLK)
        time.sleep(0.0006)
        self._write(MODE2, MODE2_OUTDRV)
        achieved_hz = self.ext_clock_hz / (STEPS * (prescale + 1))
        logger.info(
            "PCA9685 @ 0x%02x on i2c-%d: extclk %d Hz, prescale=%d, "
            "target=%d Hz, actual=%.3f Hz",
            self.address, self.bus_number, self.ext_clock_hz,
            prescale, self.freq_hz, achieved_hz,
        )

    @staticmethod
    def _prescale_value(freq_hz: int, clock_hz: int) -> int:
        """PRE_SCALE = ceil(clock / (4096 * freq)) - 1, clamped 3-255.

        Using ``ceil`` (rather than ``round``) guarantees the actual PWM
        frequency is never *greater* than the requested value, so a
        max-width servo pulse can never overflow the frame period. This
        matches ArduPilot's RCOutput_PCA9685 rationale. For our 24.576
        MHz / 50 Hz case both round and ceil produce prescale = 119
        (which yields exactly 50.000 Hz), so this is defensive for
        future frequency changes rather than a behavior change today.
        """
        if freq_hz <= 0:
            raise ValueError("freq_hz must be positive")
        raw = clock_hz / (STEPS * float(freq_hz))
        prescale = int(math.ceil(raw)) - 1
        return max(3, min(255, prescale))

    def set_pwm_counts(self, channel: int, on: int, off: int) -> None:
        """Write raw ON/OFF settings to a PWM channel (0-15).

        ``on`` and ``off`` are 13-bit values: bits 11:0 are the 12-bit
        counter count, and bit 12 (``FULL_BIT``, 0x1000) is the special
        "full ON" / "full OFF" flag from datasheet §7.3.3. Bit 12 in
        ``off`` overrides the counter and forces the pin LOW; bit 12 in
        ``on`` forces the pin HIGH. Full-OFF takes precedence over
        full-ON if both are set (per datasheet).
        """
        if not 0 <= channel <= 15:
            raise ValueError(f"channel must be 0-15, got {channel}")
        on &= 0x1FFF
        off &= 0x1FFF
        base = LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(
            self.address, base,
            [on & 0xFF, (on >> 8) & 0x1F, off & 0xFF, (off >> 8) & 0x1F],
        )

    def set_pulse(self, channel: int, pulse_us: int) -> None:
        """Drive ``channel`` to a servo pulse of ``pulse_us`` microseconds.

        ``pulse_us <= 0`` turns the channel fully off using the FULL_OFF
        special bit (matches pigpio.set_servo_pulsewidth(gpio, 0)). A
        pulse >= one frame is clamped to the FULL_ON special bit. This
        matches the ArduPilot / Adafruit encoding, so a bus analyzer
        will see exactly the same LEDn register writes.
        """
        pulse_us = int(pulse_us)
        if pulse_us <= 0:
            self.set_pwm_counts(channel, 0, FULL_BIT)
            return
        period_us = 1_000_000.0 / self.freq_hz
        if pulse_us >= period_us:
            self.set_pwm_counts(channel, FULL_BIT, 0)
            return
        off_count = int(round(STEPS * pulse_us / period_us))
        off_count = max(1, min(STEPS - 1, off_count))
        self.set_pwm_counts(channel, 0, off_count)

    def _shut_all_outputs(self) -> None:
        """Force every LEDn output LOW via the ALL_LED_OFF_H SHUT bit.

        Writes bit 4 (0x10 = ``FULL_BIT >> 8``) into ALL_LED_OFF_H
        (0xFD). Per datasheet §7.3.3, ALL_LED_OFF_H bit 4 broadcasts
        "full OFF" to every channel's LEDn_OFF_H register; the outputs
        snap LOW at the pin regardless of the 12-bit counters — no
        auto-increment / block write needed. That matters here because
        we call this from ``_init_chip`` BEFORE MODE1_AI has been set
        (POR default has AI=0), so a multi-byte auto-increment write
        would silently land all bytes at register 0xFA.
        """
        self._write(ALL_LED_OFF_H, (FULL_BIT >> 8) & 0xFF)

    def all_off(self) -> None:
        """Turn every channel fully off. Public alias for _shut_all_outputs."""
        self._shut_all_outputs()

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass
