"""MCP7940N RTC driver + system-time sync helpers for the DeckHand PCB.

The DeckHand shield ships a Microchip MCP7940N battery-backed real-time
clock at fixed I2C address 0x6F, running from a 32.768 kHz crystal (Y101)
and backed by a CR1220 (BT101). This module:

  * Reads / writes the RTC over python-smbus2
  * Ensures the on-chip oscillator (``ST``) and battery-backup path
    (``VBATEN``) are enabled, so the RTC keeps ticking through power-off
  * Sets Pi/BlueOS system time from the RTC at container start (via
    ``settimeofday`` — works inside the privileged container because it
    shares the host kernel)
  * Runs a background thread that writes the (now correct) system time
    back to the RTC once the OS confirms NTP has synchronised, so a
    subsequent power-cycle boots with an accurate clock

All time values are UTC. The Pi's system clock is kept in UTC on BlueOS,
so storing UTC in the RTC keeps everything consistent regardless of the
timezone the user's browser sends.

MCP7940N register map (only the bits we use):
  0x00 RTCSEC   b7 = ST (osc start), b6..0 = seconds (BCD)
  0x01 RTCMIN   b6..0 = minutes (BCD)
  0x02 RTCHOUR  b6 = 12/24 mode (0 = 24h), b5..0 = hours (BCD, 00-23)
  0x03 RTCWKDAY b5 = OSCRUN (r), b4 = PWRFAIL (r), b3 = VBATEN,
                b2..0 = weekday (1-7, arbitrary anchor)
  0x04 RTCDATE  b5..0 = day-of-month (BCD)
  0x05 RTCMTH   b5 = LP (leap-year, read-only), b4..0 = month (BCD)
  0x06 RTCYEAR  b7..0 = year (BCD, 00-99 -> 2000-2099)
"""

from __future__ import annotations

import ctypes
import logging
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Callable

logger = logging.getLogger(__name__)


DEFAULT_ADDRESS = 0x6F
DEFAULT_I2C_BUS = 1

RTCSEC = 0x00
RTCMIN = 0x01
RTCHOUR = 0x02
RTCWKDAY = 0x03
RTCDATE = 0x04
RTCMTH = 0x05
RTCYEAR = 0x06

ST_BIT = 0x80        # RTCSEC bit 7 — oscillator start
VBATEN_BIT = 0x08    # RTCWKDAY bit 3 — battery backup enable
PWRFAIL_BIT = 0x10   # RTCWKDAY bit 4 — power failure detected (read)
OSCRUN_BIT = 0x20    # RTCWKDAY bit 5 — oscillator running (read)


def _bcd_to_int(value: int) -> int:
    return (value >> 4) * 10 + (value & 0x0F)


def _int_to_bcd(value: int) -> int:
    return ((value // 10) << 4) | (value % 10)


@dataclass
class RtcStatus:
    """Snapshot of the last known RTC state for telemetry."""
    present: bool = False
    last_error: str | None = None
    last_read_utc: datetime | None = None
    last_synced_to_system_utc: datetime | None = None
    last_written_utc: datetime | None = None
    battery_backup: bool | None = None
    oscillator_running: bool | None = None
    power_failed: bool | None = None


class Mcp7940n:
    """Small blocking MCP7940N driver over python-smbus2."""

    def __init__(
        self,
        address: int = DEFAULT_ADDRESS,
        bus_number: int = DEFAULT_I2C_BUS,
    ) -> None:
        from smbus2 import SMBus

        self._SMBus = SMBus
        self.address = address
        self.bus_number = bus_number
        self._bus = SMBus(bus_number)
        # Probe: reading RTCSEC also tells us whether the oscillator is
        # even wired up. Raise from __init__ on failure so callers can
        # fall back gracefully.
        try:
            self._bus.read_byte_data(self.address, RTCSEC)
        except Exception:
            try:
                self._bus.close()
            except Exception:
                pass
            raise

    def close(self) -> None:
        try:
            self._bus.close()
        except Exception:
            pass

    def ensure_running(self) -> None:
        """Start the oscillator (ST=1) and enable battery backup (VBATEN=1).

        Both bits are idempotent — the RTC keeps state across power-off via
        the CR1220, so on a warm boot the oscillator is usually already
        running and this is a no-op. We still write them at startup so a
        fresh board or a dead-battery replacement comes up correctly.
        """
        sec = self._bus.read_byte_data(self.address, RTCSEC)
        if not (sec & ST_BIT):
            self._bus.write_byte_data(self.address, RTCSEC, sec | ST_BIT)
            logger.info("MCP7940N: oscillator start bit (ST) set")
        wkday = self._bus.read_byte_data(self.address, RTCWKDAY)
        if not (wkday & VBATEN_BIT):
            self._bus.write_byte_data(self.address, RTCWKDAY, wkday | VBATEN_BIT)
            logger.info("MCP7940N: battery backup (VBATEN) enabled")

    def read_status(self) -> dict[str, bool]:
        wkday = self._bus.read_byte_data(self.address, RTCWKDAY)
        return {
            "battery_backup": bool(wkday & VBATEN_BIT),
            "oscillator_running": bool(wkday & OSCRUN_BIT),
            "power_failed": bool(wkday & PWRFAIL_BIT),
        }

    def clear_power_failed(self) -> None:
        """Reset the PWRFAIL flag after reading it (writing 0 clears)."""
        wkday = self._bus.read_byte_data(self.address, RTCWKDAY)
        self._bus.write_byte_data(self.address, RTCWKDAY, wkday & ~PWRFAIL_BIT)

    def read_datetime(self) -> datetime:
        """Read the current RTC time as a timezone-aware UTC datetime.

        Assumes 24-hour mode (which ``write_datetime`` guarantees) and a
        year in 2000-2099. Callers should sanity-check the returned year
        before trusting it — a fresh board with a dead backup battery
        will report 2000-01-01 00:00 which is not useful.
        """
        block = self._bus.read_i2c_block_data(self.address, RTCSEC, 7)
        sec = _bcd_to_int(block[0] & 0x7F)     # mask ST
        minute = _bcd_to_int(block[1] & 0x7F)
        hour = _bcd_to_int(block[2] & 0x3F)    # 24h mode: bits 5..0
        day = _bcd_to_int(block[4] & 0x3F)
        month = _bcd_to_int(block[5] & 0x1F)
        year = 2000 + _bcd_to_int(block[6])
        return datetime(year, month, day, hour, minute, sec, tzinfo=timezone.utc)

    def write_datetime(self, dt: datetime) -> None:
        """Write a UTC ``datetime`` to the RTC.

        Also asserts ST=1 and VBATEN=1 in the same block so the write
        is atomic and any prior stopped-oscillator state is cleared.
        """
        if dt.tzinfo is None:
            raise ValueError("write_datetime requires a timezone-aware UTC datetime")
        utc = dt.astimezone(timezone.utc)
        if not 2000 <= utc.year <= 2099:
            raise ValueError(f"MCP7940N only supports years 2000-2099, got {utc.year}")
        # Always assert VBATEN when writing RTCWKDAY so battery backup
        # stays enabled. Weekday is 1-7 with an arbitrary anchor; we
        # always write 1..7 from Python's isoweekday so it round-trips
        # consistently.
        wkday_byte = VBATEN_BIT | (utc.isoweekday() & 0x07)
        block = [
            _int_to_bcd(utc.second) | ST_BIT,
            _int_to_bcd(utc.minute) & 0x7F,
            _int_to_bcd(utc.hour) & 0x3F,   # 24-hour mode: bit 6 = 0
            wkday_byte,
            _int_to_bcd(utc.day) & 0x3F,
            _int_to_bcd(utc.month) & 0x1F,
            _int_to_bcd(utc.year - 2000),
        ]
        self._bus.write_i2c_block_data(self.address, RTCSEC, block)


# ── System-clock plumbing ──────────────────────────────────────────────

class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


def set_system_time_utc(dt: datetime) -> None:
    """Set the Linux system clock to ``dt`` (UTC) via clock_settime.

    Requires CAP_SYS_TIME, which the extension container gets because it
    runs privileged with the host /dev bind. Uses clock_settime(2) rather
    than settimeofday(2) because glibc has been deprecating the latter,
    and clock_settime accepts nanosecond precision.
    """
    if dt.tzinfo is None:
        raise ValueError("set_system_time_utc requires a timezone-aware UTC datetime")
    utc = dt.astimezone(timezone.utc)
    epoch = utc.timestamp()
    ts = _Timespec()
    ts.tv_sec = int(epoch)
    ts.tv_nsec = int((epoch - ts.tv_sec) * 1_000_000_000)
    libc = ctypes.CDLL("libc.so.6", use_errno=True)
    CLOCK_REALTIME = 0
    if libc.clock_settime(CLOCK_REALTIME, ctypes.byref(ts)) != 0:
        err = ctypes.get_errno()
        raise OSError(err, f"clock_settime failed: {os.strerror(err)}")


# ── High-level manager ─────────────────────────────────────────────────

class RtcSyncManager:
    """Coordinates RTC <-> system-clock sync across the extension lifetime.

    Usage:
        rtc = RtcSyncManager(is_time_synced_fn=is_time_synced)
        rtc.sync_from_rtc_on_boot()   # once, early in _boot()
        rtc.start_background_sync()   # start NTP-writeback watcher

    The manager is fully defensive — every operation catches its own
    exceptions and updates ``self.status.last_error`` so the /telemetry
    route can surface the diagnostic without ever failing the request.
    """

    # Only accept RTC-sourced time when the year looks post-manufacture.
    MIN_PLAUSIBLE_YEAR = 2024

    def __init__(
        self,
        is_time_synced_fn: Callable[[], bool | None],
        address: int = DEFAULT_ADDRESS,
        bus_number: int = DEFAULT_I2C_BUS,
        poll_interval_s: float = 30.0,
    ) -> None:
        self._is_time_synced = is_time_synced_fn
        self._address = address
        self._bus_number = bus_number
        self._poll_interval_s = float(poll_interval_s)
        self._rtc: Mcp7940n | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.status = RtcStatus()

    def _ensure_rtc(self) -> Mcp7940n | None:
        if self._rtc is not None:
            return self._rtc
        try:
            rtc = Mcp7940n(address=self._address, bus_number=self._bus_number)
            rtc.ensure_running()
        except Exception as exc:
            with self._lock:
                self.status.present = False
                self.status.last_error = f"{type(exc).__name__}: {exc}"
            logger.info("MCP7940N RTC not available: %s", exc)
            return None
        self._rtc = rtc
        with self._lock:
            self.status.present = True
            self.status.last_error = None
        return rtc

    def sync_from_rtc_on_boot(self) -> bool:
        """Set the Pi system clock from the RTC at container startup.

        Returns True if the system clock was set. Skips (returns False)
        when the RTC is missing, the RTC year is implausible, the system
        already reports a synced NTP clock, or the write fails. The
        no-op cases are logged at INFO so operators can see why.
        """
        rtc = self._ensure_rtc()
        if rtc is None:
            return False

        # If NTP has already brought the clock up to date (rare on this
        # hardware, since we run before networking is guaranteed), don't
        # regress it.
        try:
            already_synced = self._is_time_synced()
        except Exception:
            already_synced = None
        if already_synced is True:
            logger.info("RTC sync-on-boot skipped: system clock already NTP-synchronised")
            return False

        try:
            status = rtc.read_status()
            rtc_time = rtc.read_datetime()
        except Exception as exc:
            with self._lock:
                self.status.last_error = f"read: {exc}"
            logger.warning("RTC read failed at boot: %s", exc)
            return False

        with self._lock:
            self.status.last_read_utc = rtc_time
            self.status.battery_backup = status.get("battery_backup")
            self.status.oscillator_running = status.get("oscillator_running")
            self.status.power_failed = status.get("power_failed")

        if rtc_time.year < self.MIN_PLAUSIBLE_YEAR:
            logger.info(
                "RTC time (%s) is implausible (< %d), leaving system clock alone",
                rtc_time.isoformat(), self.MIN_PLAUSIBLE_YEAR,
            )
            return False

        try:
            set_system_time_utc(rtc_time)
        except Exception as exc:
            with self._lock:
                self.status.last_error = f"clock_settime: {exc}"
            logger.warning("Could not set system clock from RTC: %s", exc)
            return False

        with self._lock:
            self.status.last_synced_to_system_utc = rtc_time
            self.status.last_error = None
        logger.info("System clock set from RTC: %s UTC", rtc_time.isoformat())
        # Best-effort: clear the PWRFAIL flag so we can observe future events.
        try:
            rtc.clear_power_failed()
        except Exception:
            pass
        return True

    def write_system_time_to_rtc(self) -> bool:
        """Write current system time (UTC) back to the RTC. Best effort."""
        rtc = self._ensure_rtc()
        if rtc is None:
            return False
        now = datetime.now(timezone.utc)
        try:
            rtc.write_datetime(now)
        except Exception as exc:
            with self._lock:
                self.status.last_error = f"write: {exc}"
            logger.warning("RTC write failed: %s", exc)
            return False
        with self._lock:
            self.status.last_written_utc = now
            self.status.last_error = None
        logger.info("RTC updated from NTP-synced system clock: %s UTC", now.isoformat())
        return True

    def start_background_sync(self) -> None:
        """Watch is_time_synced() and push time back to the RTC once true.

        Fires exactly once per False->True transition so we don't write
        the RTC unnecessarily on every poll. The thread continues to
        run so a re-sync (eg the browser bumps the clock forward) also
        gets recorded.
        """
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="rtc-sync", daemon=True,
        )
        self._thread.start()
        logger.info("RTC background sync thread started")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        if self._rtc is not None:
            self._rtc.close()

    def _run(self) -> None:
        last_synced = False
        while not self._stop.is_set():
            try:
                synced = self._is_time_synced()
            except Exception:
                synced = None
            # Write on any transition into "synced" (False/None -> True).
            if synced is True and not last_synced:
                self.write_system_time_to_rtc()
            if synced is True:
                last_synced = True
            elif synced is False:
                last_synced = False
            self._stop.wait(self._poll_interval_s)

    def get_status(self) -> dict[str, Any]:
        """Return a JSON-friendly snapshot for /telemetry."""
        with self._lock:
            s = self.status
            return {
                "present": s.present,
                "battery_backup": s.battery_backup,
                "oscillator_running": s.oscillator_running,
                "power_failed": s.power_failed,
                "last_error": s.last_error,
                "last_read_utc": s.last_read_utc.isoformat() if s.last_read_utc else None,
                "last_synced_to_system_utc": (
                    s.last_synced_to_system_utc.isoformat()
                    if s.last_synced_to_system_utc else None
                ),
                "last_written_utc": (
                    s.last_written_utc.isoformat() if s.last_written_utc else None
                ),
            }
