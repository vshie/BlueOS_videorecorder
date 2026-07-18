"""Environment sensor monitor for DropCam / RadCam.

Samples a Blue Robotics Bar30 (MS5837-30BA on I2C-6, address 0x76) and a
water-temperature sensor on I2C-1 at 2 Hz while running. The temperature
sensor is either a Celsius 2 (TMP119, address 0x48) which is preferred,
or the legacy Celsius (TSYS01, address 0x77) if TMP119 is not present.

If neither the Bar30 nor a temperature sensor is detected the monitor
stays silent: no UI card, no ASS fields, no *_events.ndjson rows.

Vendored driver logic adapted from the Blue Robotics reference libraries
so the extension only needs smbus2 (already in the image):
  https://github.com/bluerobotics/ms5837-python
  https://github.com/bluerobotics/tmp119-python
  https://github.com/bluerobotics/tsys01-python
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

try:
    from smbus2 import SMBus
except Exception:  # pragma: no cover - only hit when smbus2 missing
    SMBus = None  # type: ignore

logger = logging.getLogger(__name__)


# ── Blue Robotics MS5837-30BA (Bar30) ────────────────────────────────────

_MS5837_ADDR = 0x76
_MS5837_RESET = 0x1E
_MS5837_ADC_READ = 0x00
_MS5837_PROM_READ = 0xA0
_MS5837_CONVERT_D1_256 = 0x40
_MS5837_CONVERT_D2_256 = 0x50
_OSR = 5  # 8192 oversampling, ~18 ms per conversion
_MS5837_CONVERT_DELAY_S = 2.5e-6 * (2 ** (8 + _OSR))

DENSITY_SALTWATER = 1029  # kg/m^3

# Reference MSL pressure used by the BR library's depth() method.
_MSL_PRESSURE_PA = 101300.0
_GRAVITY = 9.80665


class _Bar30:
    """Vendored MS5837-30BA driver — Bar30 saltwater pressure + depth."""

    def __init__(self, bus: SMBus, lock: threading.Lock):
        self._bus = bus
        self._lock = lock
        self._C: list[int] = []
        self.pressure_mbar: float | None = None
        self.temperature_c: float | None = None
        self.depth_m: float | None = None

    def init(self) -> bool:
        with self._lock:
            self._bus.write_byte(_MS5837_ADDR, _MS5837_RESET)
            time.sleep(0.02)
            self._C = []
            for i in range(7):
                w = self._bus.read_word_data(_MS5837_ADDR, _MS5837_PROM_READ + 2 * i)
                # SMBus word reads are little-endian; MS5837 registers are big-endian.
                self._C.append(((w & 0xFF) << 8) | (w >> 8))
        crc = (self._C[0] & 0xF000) >> 12
        return crc == self._crc4(list(self._C))

    def read(self) -> bool:
        with self._lock:
            self._bus.write_byte(_MS5837_ADDR, _MS5837_CONVERT_D1_256 + 2 * _OSR)
            time.sleep(_MS5837_CONVERT_DELAY_S)
            d = self._bus.read_i2c_block_data(_MS5837_ADDR, _MS5837_ADC_READ, 3)
            D1 = d[0] << 16 | d[1] << 8 | d[2]
            self._bus.write_byte(_MS5837_ADDR, _MS5837_CONVERT_D2_256 + 2 * _OSR)
            time.sleep(_MS5837_CONVERT_DELAY_S)
            d = self._bus.read_i2c_block_data(_MS5837_ADDR, _MS5837_ADC_READ, 3)
            D2 = d[0] << 16 | d[1] << 8 | d[2]
        self._compensate(D1, D2)
        return True

    def _compensate(self, D1: int, D2: int) -> None:
        """MS5837-30BA first- and second-order compensation (datasheet)."""
        C = self._C
        dT = D2 - C[5] * 256
        SENS = C[1] * 32768 + (C[3] * dT) / 256
        OFF = C[2] * 65536 + (C[4] * dT) / 128
        temperature = 2000 + dT * C[6] / 8388608
        Ti = OFFi = SENSi = 0
        if temperature / 100 < 20:
            Ti = (3 * dT * dT) / 8589934592
            OFFi = (3 * (temperature - 2000) * (temperature - 2000)) / 2
            SENSi = (5 * (temperature - 2000) * (temperature - 2000)) / 8
            if temperature / 100 < -15:
                OFFi = OFFi + 7 * (temperature + 1500) * (temperature + 1500)
                SENSi = SENSi + 4 * (temperature + 1500) * (temperature + 1500)
        elif temperature / 100 >= 20:
            Ti = 2 * (dT * dT) / 137438953472
            OFFi = (1 * (temperature - 2000) * (temperature - 2000)) / 16
            SENSi = 0
        OFF2 = OFF - OFFi
        SENS2 = SENS - SENSi
        pressure_mbar = (((D1 * SENS2) / 2097152 - OFF2) / 8192) / 10.0
        self.temperature_c = (temperature - Ti) / 100.0
        self.pressure_mbar = float(pressure_mbar)
        pressure_pa = self.pressure_mbar * 100.0
        self.depth_m = (pressure_pa - _MSL_PRESSURE_PA) / (DENSITY_SALTWATER * _GRAVITY)

    @staticmethod
    def _crc4(n_prom: list[int]) -> int:
        n_prom[0] = n_prom[0] & 0x0FFF
        n_prom.append(0)
        n_rem = 0
        for i in range(16):
            if i % 2 == 1:
                n_rem ^= n_prom[i >> 1] & 0x00FF
            else:
                n_rem ^= n_prom[i >> 1] >> 8
            for _ in range(8, 0, -1):
                if n_rem & 0x8000:
                    n_rem = (n_rem << 1) ^ 0x3000
                else:
                    n_rem = n_rem << 1
        return (n_rem >> 12) & 0x000F


# ── Blue Robotics TMP119 (Celsius 2) ─────────────────────────────────────

_TMP119_ADDR = 0x48
_TMP119_TEMP_REG = 0x00
_TMP119_CONFIG_REG = 0x01
_TMP119_DEVICE_ID_REG = 0x0F
_TMP119_DEVICE_ID = 0x2117
_TMP119_AVG_MASK = 0x0060
_TMP119_CONV_MASK = 0x0380
_TMP119_LSB_C = 0.0078125


class _TMP119:
    """Vendored TMP119 driver — Celsius 2 water temperature."""

    def __init__(self, bus: SMBus, lock: threading.Lock):
        self._bus = bus
        self._lock = lock
        self.temperature_c: float | None = None

    def _read_reg(self, register: int) -> int:
        d = self._bus.read_i2c_block_data(_TMP119_ADDR, register, 2)
        return (d[0] << 8) | d[1]

    def _write_reg(self, register: int, value: int) -> None:
        value &= 0xFFFF
        self._bus.write_i2c_block_data(
            _TMP119_ADDR, register, [(value >> 8) & 0xFF, value & 0xFF]
        )

    def init(self) -> bool:
        with self._lock:
            device_id = self._read_reg(_TMP119_DEVICE_ID_REG)
            if device_id != _TMP119_DEVICE_ID:
                return False
            # Fastest continuous conversion: clear AVG and CONV bits.
            cfg = self._read_reg(_TMP119_CONFIG_REG)
            cfg &= ~(_TMP119_AVG_MASK | _TMP119_CONV_MASK)
            self._write_reg(_TMP119_CONFIG_REG, cfg)
        return True

    def read(self) -> bool:
        with self._lock:
            raw = self._read_reg(_TMP119_TEMP_REG)
        if raw > 32767:
            raw -= 65536
        self.temperature_c = raw * _TMP119_LSB_C
        return True


# ── Blue Robotics TSYS01 (legacy Celsius) ────────────────────────────────

_TSYS01_ADDR = 0x77
_TSYS01_RESET = 0x1E
_TSYS01_CONVERT = 0x48
_TSYS01_ADC_READ = 0x00


class _TSYS01:
    """Vendored TSYS01 driver — legacy Celsius water temperature."""

    def __init__(self, bus: SMBus, lock: threading.Lock):
        self._bus = bus
        self._lock = lock
        self._k: list[int] = []
        self.temperature_c: float | None = None

    def init(self) -> bool:
        with self._lock:
            self._bus.write_byte(_TSYS01_ADDR, _TSYS01_RESET)
            time.sleep(0.1)
            self._k = []
            # Read calibration coefficients k4..k0 (registers 0xAA..0xA2).
            for prom in range(0xAA, 0xA2 - 2, -2):
                w = self._bus.read_word_data(_TSYS01_ADDR, prom)
                self._k.append(((w & 0xFF) << 8) | (w >> 8))
        # PROM addresses that hold zero calibration are a strong sign the
        # device is not really a TSYS01. Accept as present if any coeff is
        # nonzero; the datasheet reserves k[0] as CRC in unrelated packages.
        return any(v != 0 for v in self._k[1:])

    def read(self) -> bool:
        with self._lock:
            self._bus.write_byte(_TSYS01_ADDR, _TSYS01_CONVERT)
            time.sleep(0.01)
            d = self._bus.read_i2c_block_data(_TSYS01_ADDR, _TSYS01_ADC_READ, 3)
        adc = d[0] << 16 | d[1] << 8 | d[2]
        adc16 = adc / 256
        k = self._k
        self.temperature_c = (
            -2 * k[4] * 10 ** -21 * adc16 ** 4
            + 4 * k[3] * 10 ** -16 * adc16 ** 3
            + -2 * k[2] * 10 ** -11 * adc16 ** 2
            + 1 * k[1] * 10 ** -6 * adc16
            + -1.5 * k[0] * 10 ** -2
        )
        return True


# ── Monitor ──────────────────────────────────────────────────────────────

# Consecutive read errors before we mark a channel absent again.
_MAX_ERRORS = 5


class EnvSensorMonitor:
    """Background 2 Hz sampler for Bar30 + water-temperature sensor.

    Buses:
      - I2C-1: TMP119 (0x48) preferred, TSYS01 (0x77) fallback
      - I2C-6: Bar30 (MS5837-30BA @ 0x76), saltwater density

    Silent when nothing is detected: absent devices contribute no fields to
    /telemetry, the ASS subtitle stream, or the event log.
    """

    def __init__(self, sample_hz: float = 2.0):
        self._sample_hz = float(sample_hz)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()

        # Per-bus locks so a slow bus-1 transaction doesn't block bus 6.
        self._bus1_lock = threading.Lock()
        self._bus6_lock = threading.Lock()

        self._bus1: SMBus | None = None
        self._bus6: SMBus | None = None

        self._bar30: _Bar30 | None = None
        self._temp: _TMP119 | _TSYS01 | None = None
        self._temp_kind: str | None = None  # "tmp119" | "tsys01"
        self._temp_label: str | None = None

        # Cached snapshot (updated under self._lock).
        self._snapshot: dict[str, Any] = self._empty_snapshot()

        # Consecutive error counters.
        self._bar30_errors = 0
        self._temp_errors = 0

    @staticmethod
    def _empty_snapshot() -> dict[str, Any]:
        return {
            "present": False,
            "bar30_present": False,
            "temp_present": False,
            "temp_sensor": None,
            "temp_label": None,
            "pressure_mbar": None,
            "depth_m": None,
            "bar30_temp_c": None,  # MS5837 onboard temp (slower / less accurate)
            "temp_c": None,        # Celsius / Celsius 2 water temp
            "last_error": None,
            "sample_ts": None,
        }

    # ── Probing ──────────────────────────────────────────────────────────

    def probe(self) -> None:
        """Open buses and detect sensors. Called once from start()."""
        if SMBus is None:
            logger.warning("smbus2 unavailable; environment sensors disabled")
            return

        try:
            self._bus1 = SMBus(1)
        except Exception as e:
            logger.info("Env sensors: /dev/i2c-1 unavailable (%s)", e)
            self._bus1 = None
        try:
            self._bus6 = SMBus(6)
        except Exception as e:
            logger.info("Env sensors: /dev/i2c-6 unavailable (%s)", e)
            self._bus6 = None

        if self._bus6 is not None:
            bar30 = _Bar30(self._bus6, self._bus6_lock)
            try:
                if bar30.init():
                    self._bar30 = bar30
                    logger.info("Env sensors: Bar30 (MS5837-30BA) detected on i2c-6 @ 0x76")
                else:
                    logger.info("Env sensors: MS5837 PROM CRC failed on i2c-6")
            except Exception as e:
                logger.info("Env sensors: no Bar30 on i2c-6 (%s)", e)

        if self._bus1 is not None:
            # TMP119 first (Celsius 2), TSYS01 fallback.
            tmp = _TMP119(self._bus1, self._bus1_lock)
            try:
                if tmp.init():
                    self._temp = tmp
                    self._temp_kind = "tmp119"
                    self._temp_label = "Celsius 2 (TMP119)"
                    logger.info(
                        "Env sensors: Celsius 2 (TMP119) detected on i2c-1 @ 0x48"
                    )
            except Exception as e:
                logger.debug("Env sensors: TMP119 probe failed: %s", e)

            if self._temp is None:
                tsys = _TSYS01(self._bus1, self._bus1_lock)
                try:
                    if tsys.init():
                        self._temp = tsys
                        self._temp_kind = "tsys01"
                        self._temp_label = "Celsius (TSYS01)"
                        logger.info(
                            "Env sensors: Celsius (TSYS01) detected on i2c-1 @ 0x77"
                        )
                except Exception as e:
                    logger.debug("Env sensors: TSYS01 probe failed: %s", e)

        with self._lock:
            self._snapshot.update(
                {
                    "bar30_present": self._bar30 is not None,
                    "temp_present": self._temp is not None,
                    "temp_sensor": self._temp_kind,
                    "temp_label": self._temp_label,
                    "present": self._bar30 is not None or self._temp is not None,
                }
            )

        if not self._snapshot["present"]:
            logger.info("Env sensors: none detected; monitor will stay idle")

    # ── Thread lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self.probe()
        if not self._snapshot["present"]:
            # Nothing to sample — do not spin a thread.
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="env-sensors", daemon=True
        )
        self._thread.start()
        logger.info("Env sensor monitor started (2 Hz)")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
        # Close bus handles.
        for bus in (self._bus1, self._bus6):
            try:
                if bus is not None:
                    bus.close()
            except Exception:
                pass

    # ── Public accessors ─────────────────────────────────────────────────

    def is_present(self) -> bool:
        with self._lock:
            return bool(self._snapshot.get("present"))

    def get_snapshot(self) -> dict[str, Any]:
        """Copy of the last cached snapshot (safe to serialize)."""
        with self._lock:
            return dict(self._snapshot)

    def get_summary(self) -> dict[str, Any]:
        """Compact summary embedded in /telemetry.environment.

        Returns {"present": False} when nothing is detected so the UI can
        cleanly gate the Environment card off.
        """
        snap = self.get_snapshot()
        if not snap.get("present"):
            return {"present": False}
        return snap

    def get_log_fields(self) -> dict[str, Any]:
        """Fields for ASS + events log while recording.

        Empty dict when no sensor is present.
        """
        snap = self.get_snapshot()
        if not snap.get("present"):
            return {}
        out: dict[str, Any] = {}
        if snap.get("bar30_present"):
            p = snap.get("pressure_mbar")
            d = snap.get("depth_m")
            bt = snap.get("bar30_temp_c")
            if p is not None:
                out["pressure_mbar"] = round(float(p), 2)
            if d is not None:
                out["depth_m"] = round(float(d), 3)
            if bt is not None:
                out["bar30_temp_c"] = round(float(bt), 2)
        if snap.get("temp_present"):
            t = snap.get("temp_c")
            if t is not None:
                out["temp_c"] = round(float(t), 2)
            if snap.get("temp_sensor"):
                out["temp_sensor"] = snap["temp_sensor"]
        return out

    # ── Sampling loop ────────────────────────────────────────────────────

    def _run(self) -> None:
        period = 1.0 / max(self._sample_hz, 0.1)
        while not self._stop.is_set():
            t0 = time.monotonic()
            self._sample_once()
            elapsed = time.monotonic() - t0
            wait = period - elapsed
            if wait > 0:
                if self._stop.wait(wait):
                    break

    def _sample_once(self) -> None:
        pressure = depth = bar30_temp = temp = None
        last_error: str | None = None

        if self._bar30 is not None:
            try:
                self._bar30.read()
                pressure = self._bar30.pressure_mbar
                depth = self._bar30.depth_m
                bar30_temp = self._bar30.temperature_c
                self._bar30_errors = 0
            except Exception as e:
                last_error = f"bar30: {e}"
                self._bar30_errors += 1
                if self._bar30_errors >= _MAX_ERRORS:
                    logger.warning(
                        "Env sensors: dropping Bar30 after %d consecutive errors: %s",
                        self._bar30_errors, e,
                    )
                    self._bar30 = None

        if self._temp is not None:
            try:
                self._temp.read()
                temp = self._temp.temperature_c
                self._temp_errors = 0
            except Exception as e:
                last_error = f"temp: {e}"
                self._temp_errors += 1
                if self._temp_errors >= _MAX_ERRORS:
                    logger.warning(
                        "Env sensors: dropping %s after %d consecutive errors: %s",
                        self._temp_kind, self._temp_errors, e,
                    )
                    self._temp = None
                    self._temp_kind = None
                    self._temp_label = None

        with self._lock:
            self._snapshot.update(
                {
                    "bar30_present": self._bar30 is not None,
                    "temp_present": self._temp is not None,
                    "temp_sensor": self._temp_kind,
                    "temp_label": self._temp_label,
                    "pressure_mbar": pressure,
                    "depth_m": depth,
                    "bar30_temp_c": bar30_temp,
                    "temp_c": temp,
                    "last_error": last_error,
                    "sample_ts": time.time(),
                    "present": self._bar30 is not None or self._temp is not None,
                }
            )
