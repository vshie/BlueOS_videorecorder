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
import time

logger = logging.getLogger(__name__)


DEFAULT_ADDRESS = 0x40
DEFAULT_I2C_BUS = 1

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

MODE1_RESTART = 0x80
MODE1_EXTCLK = 0x40
MODE1_AI = 0x20
MODE1_SLEEP = 0x10

MODE2_OUTDRV = 0x04  # totem-pole outputs

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

        Sequence per datasheet 7.3.1.1 for switching to EXTCLK:
          1. Put chip to sleep (SLEEP=1) to unlock PRESCALE + EXTCLK.
          2. Write MODE1 = SLEEP | EXTCLK to select the external clock.
          3. Program PRESCALE for the target servo frequency.
          4. Restore MODE1 with AI=1 (auto-increment for burst writes) and
             SLEEP=0, then wait 500 us for the oscillator to stabilise.
          5. Set MODE2 = OUTDRV=1 (totem-pole, which the RN101/RN102 series
             resistors expect for the 3.3 V servo signal lines).
        """
        self._write(MODE1, MODE1_SLEEP)
        self._write(MODE1, MODE1_SLEEP | MODE1_EXTCLK)
        prescale = self._prescale_value(self.freq_hz, self.ext_clock_hz)
        self._write(PRESCALE, prescale)
        self._write(MODE1, MODE1_AI | MODE1_EXTCLK)
        time.sleep(0.0006)
        self._write(MODE2, MODE2_OUTDRV)
        logger.info(
            "PCA9685 @ 0x%02x on i2c-%d: extclk %d Hz, prescale=%d, target=%d Hz",
            self.address, self.bus_number, self.ext_clock_hz, prescale, self.freq_hz,
        )

    @staticmethod
    def _prescale_value(freq_hz: int, clock_hz: int) -> int:
        """PRESCALE = round(clock / (4096 * freq)) - 1, clamped to the datasheet range."""
        if freq_hz <= 0:
            raise ValueError("freq_hz must be positive")
        raw = clock_hz / (STEPS * float(freq_hz)) - 1.0
        prescale = int(round(raw))
        return max(3, min(255, prescale))

    def set_pwm_counts(self, channel: int, on: int, off: int) -> None:
        """Write raw 12-bit ON/OFF counts to a PWM channel (0-15)."""
        if not 0 <= channel <= 15:
            raise ValueError(f"channel must be 0-15, got {channel}")
        on &= 0x0FFF
        off &= 0x0FFF
        base = LED0_ON_L + 4 * channel
        self._bus.write_i2c_block_data(
            self.address, base,
            [on & 0xFF, (on >> 8) & 0x0F, off & 0xFF, (off >> 8) & 0x0F],
        )

    def set_pulse(self, channel: int, pulse_us: int) -> None:
        """Drive ``channel`` to a servo pulse of ``pulse_us`` microseconds.

        ``pulse_us <= 0`` turns the channel fully off (0% duty, no pulse),
        matching pigpio.set_servo_pulsewidth(gpio, 0). ``pulse_us`` above one
        frame is clamped to 100% duty.
        """
        pulse_us = int(pulse_us)
        if pulse_us <= 0:
            self.set_pwm_counts(channel, 0, 0x1000)  # full OFF bit
            return
        period_us = 1_000_000.0 / self.freq_hz
        if pulse_us >= period_us:
            self.set_pwm_counts(channel, 0x1000, 0)  # full ON bit
            return
        off_count = int(round(STEPS * pulse_us / period_us))
        off_count = max(1, min(STEPS - 1, off_count))
        self.set_pwm_counts(channel, 0, off_count)

    def all_off(self) -> None:
        """Turn every channel fully off. Uses the ALL_LED_OFF broadcast register."""
        try:
            self._bus.write_i2c_block_data(self.address, 0xFA, [0, 0, 0, 0x10])
        except Exception:
            for ch in range(16):
                try:
                    self.set_pwm_counts(ch, 0, 0x1000)
                except Exception:
                    pass

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass
