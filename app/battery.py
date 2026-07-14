"""
Battery monitor for the Doris battery pack (Daly BMS over RS485).

Background polling thread that:
  - Opens the serial port, reads a full snapshot every poll_interval_s
  - Reconnects with backoff on serial errors / missing dependencies
  - Tracks time since last full charge via FullChargeTracker
  - Appends a CSV row per poll to a per-power-up file
        - Filename starts as an incrementing number, e.g. battery_0007.csv,
          because a battery-backed RTC is not always current on boot
        - Once is_time_synced() reports True the file is renamed to
          battery_0007_YYYYMMDD_HHMMSS.csv (keeping the sequence number)
  - Drives the LED battery-low alarm with hysteresis when voltage drops
    below low_voltage and clears it once it rises above clear_voltage

Wiring: on the DeckHand PCB the Daly BMS is wired to UART4 on the Pi
(GPIO 8 TX, GPIO 9 RX) through an SN65HVD75 RS485 transceiver. GPIO 11
drives DE/~RE (tied together): HIGH before writing, LOW to receive.
GPIO 11 is also PL011's hardware RTS4 alt-function pin, so on a kernel
new enough to support ``TIOCSRS485`` (Linux ≥ 5.14, i.e. Bookworm-based
BlueOS 1.5.x) the driver handles DE with hardware-precision timing —
same as an FT232's auto-direction in a BLUART adapter. _probe_device()
opportunistically tries that ioctl first and only falls back to the
software DE toggle (via lgpio + a5_bus.set_direction_control()) when
the kernel doesn't support it. The default serial_port is "auto" — the
port scanner uses the sysfs of_node MMIO address to find whichever
/dev/ttyAMA<N> is currently mapped to UART4 hardware (0x7e201800 on
BCM2711), because the ttyAMA index depends on device-tree overlay
load order and is not fixed across systems.
"""

from __future__ import annotations

import logging
import os
import re
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable

logger = logging.getLogger(__name__)

DEFAULT_CONFIG: dict[str, Any] = {
    "enabled": True,
    # DeckHand PCB wires the BMS to UART4 (GPIO 8/9) through an SN65HVD75
    # RS485 transceiver. The Linux tty name for a given hardware UART is
    # NOT fixed on the Pi — it depends on the order in which `dtoverlay=
    # uartN` overlays enumerate at boot, so UART4 can appear as anything
    # from /dev/ttyAMA1 to /dev/ttyAMA4 depending on which other UART
    # overlays are enabled in /boot/config.txt. "auto" (the default)
    # discovers UART4 by matching the sysfs of_node MMIO address
    # (7e201800) first, then probes any other Pi UARTs and USB serial
    # adapters so mixed / legacy setups still work.
    "serial_port": "auto",
    "board_number": 1,
    "baud_rate": 9600,
    "serial_timeout_s": 0.5,
    "request_retries": 3,
    # How to read the BMS:
    #   "poll"      - actively query a full snapshot (status/cells/temps/SOC).
    #                 Richest data, but some Daly packs time out on these reads.
    #   "broadcast" - passively listen for / solicit the pack's 0x90 SOC frame
    #                 (total voltage, current, SOC only). Works with packs that
    #                 auto-report or only answer 0x90.
    #   "auto"      - try a full poll snapshot; if it fails, fall back to a
    #                 broadcast SOC read for that cycle (default).
    "read_mode": "auto",
    "broadcast_window_s": 3.0,
    "poll_interval_s": 5.0,
    "low_voltage": 13.0,
    "clear_voltage": 13.2,
    "csv_logging_enabled": True,
    "full_charge_soc_percent": 98.0,
    "rest_current_threshold_a": 0.5,
    "log_dir": "/app/videorecordings/battery_logs",
    "charge_state_path": "/app/videorecordings/battery_logs/charge_state.json",
    # RS485 direction control (SN65HVD75 DE/~RE, tied together on the
    # DeckHand PCB). Set to null to disable direction toggling for boards
    # with auto-direction transceivers or external USB adapters.
    "rs485_de_gpio": 11,
    # Time to hold DE HIGH after write() before flipping back to receive.
    # 300 us covers a 15-byte @ 9600 baud frame's last-byte drain plus
    # some slack for the transceiver's tPHZ (~50 ns).
    "rs485_de_release_delay_s": 0.001,
}

# Serial paths we deliberately skip when auto-scanning. /dev/ttyS0 is the
# mini-UART / console alias and /dev/ttyprintk is a kernel logger sink —
# neither is wired to the BMS. /dev/ttyAMA0 was skipped historically when
# the BMS lived on a USB adapter; on the DeckHand PCB we deliberately
# include ttyAMA* so /dev/ttyAMA4 (UART4) auto-detects.
_AUTO_SCAN_SKIP_PREFIXES = ("/dev/ttyS", "/dev/ttyprintk")

_SEQUENCE_RE = re.compile(r"^battery_(\d+)(?:_.*)?\.csv$", re.IGNORECASE)


def _merged_config(user_cfg: dict[str, Any] | None) -> dict[str, Any]:
    cfg = dict(DEFAULT_CONFIG)
    if user_cfg:
        for key, value in user_cfg.items():
            if value is not None:
                cfg[key] = value
    return cfg


class BatteryMonitor:
    """Single-pack Daly BMS poller with CSV logging and LED low-voltage alarm.

    `hw` should expose `set_battery_alarm(bool)` (see hardware.py); if missing
    the monitor still polls and logs but logs a warning on first alarm.
    `config_getter` is called each poll to refresh tunables without restart.
    `time_synced_fn` is called each poll; the first True triggers a rename
    that adds a wall-clock timestamp to the CSV filename.
    """

    def __init__(
        self,
        hw: Any,
        config_getter: Callable[[], dict[str, Any] | None],
        time_synced_fn: Callable[[], bool | None] | None = None,
    ) -> None:
        self._hw = hw
        self._config_getter = config_getter
        self._time_synced_fn = time_synced_fn or (lambda: None)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        self._reader: Any = None
        self._reader_device: str | None = None
        self._connected = False
        self._last_snapshot: dict[str, Any] | None = None
        self._last_error: str | None = None
        self._alarm_active = False
        # Which read path produced the last snapshot ("poll" | "broadcast").
        self._last_read_mode: str | None = None
        # Ports that responded to a probe but with bad data, so we deprioritize
        # them on the next scan instead of churning them every cycle.
        self._port_failures: dict[str, int] = {}

        # CSV state — assigned on first successful poll once we know log_dir
        self._csv_path: Path | None = None
        self._csv_sequence: int | None = None
        self._csv_fieldnames: list[str] = []
        self._csv_timestamped = False
        self._csv_init_failed = False

        # Charge tracker — built lazily once we know state_path + threshold
        self._charge_tracker: Any = None
        self._tracker_path: Path | None = None
        self._tracker_threshold: float | None = None

    # ── Thread lifecycle ─────────────────────────────────────────────────

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="battery-monitor", daemon=True)
        self._thread.start()
        logger.info("Battery monitor started")

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
        self._disconnect()

    # ── Public accessors ─────────────────────────────────────────────────

    def get_data(self) -> dict[str, Any]:
        """Full snapshot for the /battery route."""
        with self._lock:
            snap = self._last_snapshot
            return {
                "connected": self._connected,
                "serial_port": self._reader_device,
                "read_mode": self._last_read_mode,
                "last_error": self._last_error,
                "low_voltage_alarm": self._alarm_active,
                "csv_path": str(self._csv_path) if self._csv_path else None,
                "snapshot": snap,
            }

    def get_data_summary(self) -> dict[str, Any]:
        """Compact summary embedded in /telemetry."""
        with self._lock:
            snap = self._last_snapshot or {}
            summary = snap.get("summary") or {}
            return {
                "connected": self._connected,
                "voltage_v": summary.get("total_voltage_v"),
                "current_a": summary.get("current_a"),
                "soc_percent": summary.get("soc_percent"),
                "cell_delta_mv": summary.get("cell_delta_mv"),
                "since_full_charge": summary.get("since_full_charge"),
                "low_voltage_alarm": self._alarm_active,
                "last_error": self._last_error,
            }

    def get_voltage(self) -> float | None:
        with self._lock:
            snap = self._last_snapshot or {}
            v = (snap.get("summary") or {}).get("total_voltage_v")
            try:
                return float(v) if v is not None else None
            except (TypeError, ValueError):
                return None

    # ── Internals ────────────────────────────────────────────────────────

    def _current_config(self) -> dict[str, Any]:
        try:
            user = self._config_getter() or {}
        except Exception as exc:
            logger.warning("Battery config lookup failed: %s", exc)
            user = {}
        return _merged_config(user)

    @staticmethod
    def _read_mode(cfg: dict[str, Any]) -> str:
        mode = str(cfg.get("read_mode", DEFAULT_CONFIG["read_mode"])).strip().lower()
        return mode if mode in ("poll", "broadcast", "auto") else "auto"

    def _read_snapshot(self, cfg: dict[str, Any]) -> dict[str, Any]:
        """Read one snapshot honouring read_mode. May raise on failure."""
        assert self._reader is not None
        mode = self._read_mode(cfg)
        window = float(cfg.get("broadcast_window_s", DEFAULT_CONFIG["broadcast_window_s"]))

        if mode == "poll":
            snap = self._reader.read_snapshot()
            self._last_read_mode = "poll"
            return snap

        if mode == "broadcast":
            soc = self._reader.read_soc_broadcast(duration=window, solicit=True)
            if not soc:
                raise RuntimeError("No broadcast SOC frames received")
            self._last_read_mode = "broadcast"
            return self._reader.snapshot_from_soc(soc)

        # auto: prefer a full poll snapshot, fall back to broadcast SOC.
        try:
            snap = self._reader.read_snapshot()
            self._last_read_mode = "poll"
            return snap
        except Exception as poll_exc:
            logger.debug("Poll snapshot failed, trying broadcast: %s", poll_exc)
            soc = self._reader.read_soc_broadcast(duration=window, solicit=True)
            if not soc:
                raise RuntimeError(
                    f"Poll read failed ({poll_exc}); no broadcast SOC frames either"
                ) from poll_exc
            self._last_read_mode = "broadcast"
            return self._reader.snapshot_from_soc(soc)

    def _run(self) -> None:
        backoff_s = 2.0
        while not self._stop.is_set():
            cfg = self._current_config()
            if not cfg.get("enabled", True):
                self._stop.wait(5.0)
                continue

            try:
                if self._reader is None:
                    self._connect(cfg)
                snapshot = self._read_snapshot(cfg)
            except Exception as exc:
                self._handle_error(exc)
                wait = min(backoff_s, 30.0)
                backoff_s = min(backoff_s * 2.0, 30.0)
                self._stop.wait(wait)
                continue

            backoff_s = 2.0

            try:
                self._process_snapshot(snapshot, cfg)
            except Exception as exc:
                logger.exception("Battery snapshot processing failed: %s", exc)

            interval = float(cfg.get("poll_interval_s", 5.0))
            self._stop.wait(max(interval, 0.5))

    def _connect(self, cfg: dict[str, Any]) -> None:
        """Find a serial port that answers as a Daly BMS and adopt it.

        If `serial_port` config is "auto" (default) or empty, enumerate all
        currently-visible serial devices, probe each one in turn, and use
        the first that responds. Otherwise the configured path is the only
        candidate. Ports that fail the probe are released (closed) so they
        stay available to other consumers.
        """
        self._disconnect()

        explicit = cfg.get("serial_port") or "auto"
        if explicit and explicit not in ("auto", "AUTO", "scan"):
            candidates = [explicit]
        else:
            candidates = self._iter_candidate_ports()
            if not candidates:
                raise RuntimeError("No serial devices found to probe")

        board = int(cfg.get("board_number", 1))
        baud = int(cfg.get("baud_rate", 9600))
        timeout = float(cfg.get("serial_timeout_s", 0.5))
        retries = int(cfg.get("request_retries", 3))
        mode = self._read_mode(cfg)
        window = float(cfg.get("broadcast_window_s", DEFAULT_CONFIG["broadcast_window_s"]))
        # NB: build a *lazy* DE-toggle factory rather than claiming GPIO 11
        # up front. If the tty ends up being a PL011 that accepts TIOCSRS485
        # (kernel-driven DE) we must NOT snatch GPIO 11 out of ALT4 RTS4 —
        # claiming it as a lgpio OUTPUT would reconfigure the pinmux and
        # silently break the kernel's DE control. The factory only fires
        # inside the probe if kernel mode is refused.
        de_toggle_factory = self._make_de_toggle_factory(cfg)
        de_release_delay_s = float(
            cfg.get("rs485_de_release_delay_s",
                    DEFAULT_CONFIG["rs485_de_release_delay_s"])
        )

        errors: list[str] = []
        for device in candidates:
            reader = self._probe_device(
                device, board, baud, timeout, retries, mode, window,
                de_toggle_factory, de_release_delay_s,
            )
            if reader is None:
                errors.append(f"{device}: no Daly response")
                continue
            self._reader = reader
            self._reader_device = device
            self._port_failures.pop(device, None)
            with self._lock:
                self._connected = True
                self._last_error = None
            logger.info(
                "Battery monitor connected to %s board=%d @ %d baud (mode=%s, auto-detected from %d candidate(s))",
                device, board, baud, mode, len(candidates),
            )
            return

        # None responded — record the failure pattern so we don't spam the
        # log with the same useless port at the top of the list forever.
        for device in candidates:
            self._port_failures[device] = self._port_failures.get(device, 0) + 1

        msg = "; ".join(errors) if errors else "no candidates probed"
        raise RuntimeError(f"No Daly BMS found ({msg})")

    # Remembers which DE GPIOs we've already logged the "direction control
    # on GPIO N" message for, so a reconnect loop doesn't spam it once per
    # probe cycle. Also acts as a "we've already claimed this pin" flag —
    # GpioLine.claim_output() is idempotent, but re-logging is not.
    _de_toggle_logged: set[int] = set()
    # Same idea for the UART4 discovery message: log it once per resolved
    # tty path so a probe/reconnect loop doesn't spam the same info line.
    _uart4_logged: str | None = None
    # And once per tty path for the "kernel is now doing RTS for us" log,
    # so the probe loop doesn't repeat it on every reconnect.
    _kernel_rs485_logged: set[str] = set()

    @staticmethod
    def _make_de_toggle_factory(
        cfg: dict[str, Any],
    ) -> Callable[[], Callable[[bool], None] | None] | None:
        """Return a *factory* that, when called, claims the RS485 DE GPIO
        and returns a toggle callback. The claim only happens when the
        factory is actually invoked — critical when the same GPIO doubles
        as PL011 RTS4 for kernel-mode DE control. On the DeckHand Rev-A
        firmware ``dtoverlay=uart4`` (no ``,ctsrts``) leaves GPIO 10 free
        for the rotation sensor, and a targeted ``gpio=11=a4,pn`` line
        in ``/boot/firmware/config.txt`` forces GPIO 11 into ALT4 RTS4
        so the PL011 driver still owns the DE pin. If we claim GPIO 11
        as a lgpio OUTPUT before the probe decides between kernel-mode
        and software-mode, the pinmux flip breaks the kernel RTS control
        that ``TIOCSRS485`` relies on — silently, because the
        ioctl itself still succeeds.

        Returns None if direction control is fully disabled (``rs485_de_gpio``
        <= 0 or null) or if the GPIO helper isn't importable (dev laptop,
        missing lgpio). Otherwise returns a zero-arg factory.
        """
        de_gpio = cfg.get("rs485_de_gpio")
        if de_gpio is None:
            return None
        try:
            de_gpio_int = int(de_gpio)
        except (TypeError, ValueError):
            return None
        if de_gpio_int < 0:
            return None
        try:
            from gpio_backend import get_gpio_line
        except Exception:
            return None
        line = get_gpio_line()
        if line is None:
            return None

        def _factory() -> Callable[[bool], None] | None:
            try:
                line.claim_output(de_gpio_int, 0)
            except Exception as exc:
                logger.warning("RS485 DE (GPIO %d) claim failed: %s", de_gpio_int, exc)
                return None
            if de_gpio_int not in BatteryMonitor._de_toggle_logged:
                logger.info("RS485 half-duplex direction control on GPIO %d (software toggle)", de_gpio_int)
                BatteryMonitor._de_toggle_logged.add(de_gpio_int)

            def _toggle(transmit: bool) -> None:
                try:
                    line.write(de_gpio_int, 1 if transmit else 0)
                except Exception as exc:
                    logger.debug("RS485 DE write failed: %s", exc)

            return _toggle

        return _factory

    def _probe_device(
        self,
        device: str,
        board: int,
        baud: int,
        timeout: float,
        retries: int,
        mode: str = "auto",
        window: float = 3.0,
        de_toggle_factory: Callable[[], Callable[[bool], None] | None] | None = None,
        de_release_delay_s: float = 0.001,
    ) -> Any | None:
        """Open `device`, send one quick BMS read, return the reader if it
        responds with plausible Daly data, else close and return None.

        The probe method matches read_mode: a polled get_soc() for "poll",
        a broadcast SOC listen for "broadcast", and poll-then-broadcast for
        "auto" (so packs that only auto-report still get detected)."""
        from doris_battery.bms_reader import DorisBMSReader

        reader = DorisBMSReader(
            device,
            board_number=board,
            # Tight retry count for the probe — a real BMS answers on the
            # first try; non-BMS adapters just waste time on retries.
            request_retries=1,
            baudrate=baud,
            # Short read timeout for the probe so we move on quickly when
            # a port has no responder.
            serial_timeout=min(timeout, 0.4),
            pack_name="dropcam",
        )
        try:
            reader.connect()
        except Exception as exc:
            logger.debug("Probe %s: open failed: %s", device, exc)
            return None

        # Prefer kernel-driven half-duplex (TIOCSRS485 on the PL011 driver)
        # over the software DE toggle: the kernel flips RTS in hardware with
        # sub-microsecond timing, whereas Python-side gpio_write() incurs
        # milliseconds of jitter that can chop the head off the pack's
        # reply. Only makes sense on Pi UARTs where RTS is wired to the
        # transceiver DE (DeckHand PCB routes SN65HVD75 DE/~RE to GPIO 11,
        # which is also PL011 RTS4 alt-function). On USB serial adapters the
        # ioctl fails silently and we fall through to the software path.
        used_kernel_rs485 = False
        if device.startswith("/dev/ttyAMA") or device == "/dev/serial0":
            try:
                from rs485_kernel_mode import try_enable_rs485
                if try_enable_rs485(reader._bms.serial):
                    used_kernel_rs485 = True
                    if device not in BatteryMonitor._kernel_rs485_logged:
                        logger.info(
                            "RS485 direction control: kernel TIOCSRS485 on %s "
                            "(hardware RTS timing, no GPIO toggle needed)",
                            device,
                        )
                        BatteryMonitor._kernel_rs485_logged.add(device)
            except Exception as exc:
                logger.debug("Probe %s: rs485_kernel_mode import/ioctl error: %s", device, exc)

        # Software DE toggle: only install when the kernel didn't take over.
        # Skips the fallback for USB adapters (de_toggle_factory is None)
        # since their FT232 handles direction in hardware already. NB: the
        # factory is invoked *here* — that's the first moment we actually
        # claim GPIO 11 as a lgpio OUTPUT, so if kernel mode won earlier we
        # never touch its ALT4 RTS4 pinmux.
        if not used_kernel_rs485 and de_toggle_factory is not None:
            de_toggle = de_toggle_factory()
            if de_toggle is not None:
                try:
                    reader._bms.set_direction_control(
                        de_toggle,
                        release_delay_s=de_release_delay_s,
                    )
                except AttributeError:
                    logger.debug("Probe %s: reader has no set_direction_control()", device)
                except Exception as exc:
                    logger.debug("Probe %s: set_direction_control failed: %s", device, exc)

        # Keep the broadcast probe window short so scanning stays responsive;
        # solicited 0x90 frames normally arrive within a few hundred ms.
        probe_window = min(window, 2.0)
        voltage = self._probe_voltage(reader, device, mode, probe_window)
        if voltage is None:
            logger.debug("Probe %s: no Daly response (likely not a BMS)", device)
            reader.disconnect()
            return None

        try:
            v_num = float(voltage)
        except (TypeError, ValueError):
            logger.debug("Probe %s: voltage=%r is not numeric", device, voltage)
            reader.disconnect()
            return None

        # Sanity-check the response. A 4S LiFePO4 pack sits ~10-15 V; allow a
        # wide window so we don't reject odd states, but reject 0 V which
        # often comes back from misframed reads.
        if v_num <= 1.0 or v_num > 80.0:
            logger.debug("Probe %s: voltage %.2f V outside plausible range", device, v_num)
            reader.disconnect()
            return None

        # Bump up to the configured retry count for the real polling.
        try:
            reader._bms.request_retries = retries
            reader._bms.serial_timeout = timeout
            if reader._bms.serial is not None:
                reader._bms.serial.timeout = timeout
        except Exception:
            pass

        logger.info("Probe %s: Daly BMS responded (V=%.2f, mode=%s)", device, v_num, mode)
        return reader

    @staticmethod
    def _probe_voltage(reader: Any, device: str, mode: str, window: float) -> Any:
        """Return a total_voltage reading from a probe, or None.

        poll      -> single polled get_soc()
        broadcast -> short broadcast/solicit listen
        auto      -> polled first, broadcast fallback
        """
        def _poll() -> Any:
            try:
                soc = reader._bms.get_soc()
            except Exception as exc:
                logger.debug("Probe %s: polled read raised: %s", device, exc)
                return None
            return soc.get("total_voltage") if isinstance(soc, dict) else None

        def _broadcast() -> Any:
            try:
                soc = reader.read_soc_broadcast(duration=window, solicit=True)
            except Exception as exc:
                logger.debug("Probe %s: broadcast read raised: %s", device, exc)
                return None
            return soc.get("total_voltage") if isinstance(soc, dict) else None

        if mode == "poll":
            return _poll()
        if mode == "broadcast":
            return _broadcast()
        # auto
        voltage = _poll()
        if voltage is None:
            voltage = _broadcast()
        return voltage

    @staticmethod
    def _find_pi_uart_by_mmio(mmio_hex_addresses: tuple[str, ...]) -> str | None:
        """Return the /dev/ttyAMA<N> currently mapped to one of the given
        UART hardware MMIO addresses, or None if no match is found.

        Linux enumerates the Pi's PL011 UARTs into /dev/ttyAMA<N> in the
        order the device-tree overlays are loaded, so a given UART
        controller does NOT keep the same tty name across systems. The
        stable identifier is the sysfs of_node symlink, which points to
        the device-tree node named ``serial@<mmio>``. We prefer the
        first match in ``mmio_hex_addresses`` so callers can list
        multiple candidates in priority order (Pi 4 first, then Pi 5).

        Example on a Pi 4 with ``dtoverlay=uart3,uart4,uart5`` enabled:
            ttyAMA0 -> serial@7e201000   (primary console UART0)
            ttyAMA1 -> serial@7e201600   (UART3)
            ttyAMA2 -> serial@7e201800   (UART4)  <-- matches on 7e201800
            ttyAMA3 -> serial@7e201a00   (UART5)
        """
        import glob

        mapping: dict[str, str] = {}  # mmio -> /dev/ttyAMA<N>
        for tty_path in sorted(glob.glob("/sys/class/tty/ttyAMA*")):
            of_link = os.path.join(tty_path, "device", "of_node")
            try:
                target = os.path.realpath(of_link)
            except OSError:
                continue
            base = os.path.basename(target).lower()  # e.g. "serial@7e201800"
            if "@" not in base:
                continue
            mmio = base.split("@", 1)[1]
            dev_path = f"/dev/{os.path.basename(tty_path)}"
            mapping.setdefault(mmio, dev_path)

        for wanted in mmio_hex_addresses:
            hit = mapping.get(wanted.lower())
            if hit and os.path.exists(hit):
                return hit
        return None

    @staticmethod
    def _iter_candidate_ports() -> list[str]:
        """Enumerate serial device paths to probe, Pi-UART first.

        On the DeckHand PCB the BMS lives on Pi UART4 (GPIO 8/9). Because
        the /dev/ttyAMA<N> index depends on device-tree overlay load
        order, we resolve UART4 by MMIO address (7e201800 on BCM2711,
        matches the same UART controller on BCM2712/RP1 legacy layout)
        and put that tty first. We then fall back to all remaining ttyAMA
        devices, /dev/serial0, and finally USB serial adapters, so legacy
        setups (BMS on ttyAMA0 / USB-RS485) still auto-detect. Combines
        pyserial's ``list_ports.comports()`` with a direct glob over /dev
        so we don't miss devices that pyserial's enumeration drops (some
        adapters lack udev info inside containers).
        """
        import glob

        seen: list[str] = []

        def _add(path: str) -> None:
            if not path or path in seen:
                return
            if any(path.startswith(prefix) for prefix in _AUTO_SCAN_SKIP_PREFIXES):
                return
            seen.append(path)

        # Prefer the actual UART4 hardware, wherever the kernel put it.
        uart4 = BatteryMonitor._find_pi_uart_by_mmio(("7e201800",))
        if uart4:
            if BatteryMonitor._uart4_logged != uart4:
                logger.info("BMS auto-scan: resolved UART4 hardware to %s", uart4)
                BatteryMonitor._uart4_logged = uart4
            _add(uart4)
        elif BatteryMonitor._uart4_logged != "__missing__":
            logger.info(
                "BMS auto-scan: UART4 MMIO (0x7e201800) not exposed as any "
                "ttyAMA* — check that 'dtoverlay=uart4' is set in "
                "/boot/config.txt on the host and the Pi has been rebooted."
            )
            BatteryMonitor._uart4_logged = "__missing__"

        for path in ("/dev/serial0",):
            if os.path.exists(path):
                _add(path)
        for pattern in ("/dev/ttyAMA*",):
            for path in sorted(glob.glob(pattern)):
                _add(path)

        try:
            from serial.tools import list_ports

            for info in list_ports.comports():
                _add(info.device)
        except Exception as exc:
            logger.debug("list_ports.comports() failed: %s", exc)

        for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
            for path in sorted(glob.glob(pattern)):
                _add(path)

        return seen

    def _disconnect(self) -> None:
        reader = self._reader
        self._reader = None
        self._reader_device = None
        if reader is None:
            return
        try:
            reader.disconnect()
        except Exception:
            pass
        with self._lock:
            self._connected = False

    def _handle_error(self, exc: Exception) -> None:
        msg = str(exc) or exc.__class__.__name__
        with self._lock:
            if self._last_error != msg:
                logger.warning("Battery monitor error: %s", msg)
            self._last_error = msg
            self._connected = False
        self._disconnect()

    def _process_snapshot(self, snapshot: dict[str, Any], cfg: dict[str, Any]) -> None:
        snapshot["updated_at"] = time.time()
        board = snapshot.get("board_number")
        if board is None:
            board = int(cfg.get("board_number", 1))
            snapshot["board_number"] = board

        tracker = self._get_tracker(cfg)
        if tracker is not None:
            try:
                if tracker.observe(board, snapshot):
                    tracker.save()
                tracker.enrich_snapshot(snapshot)
            except Exception as exc:
                logger.warning("Charge tracker update failed: %s", exc)

        with self._lock:
            self._last_snapshot = snapshot
            self._last_error = None
            self._connected = True

        self._update_alarm(snapshot, cfg)

        if cfg.get("csv_logging_enabled", True):
            self._append_csv(snapshot, cfg)

    def _update_alarm(self, snapshot: dict[str, Any], cfg: dict[str, Any]) -> None:
        summary = snapshot.get("summary") or {}
        voltage = summary.get("total_voltage_v")
        if voltage is None:
            return
        try:
            v = float(voltage)
        except (TypeError, ValueError):
            return

        low = float(cfg.get("low_voltage", 13.0))
        clear = float(cfg.get("clear_voltage", max(low + 0.1, low)))
        # Guard against an inverted configuration.
        if clear < low:
            clear = low

        with self._lock:
            active = self._alarm_active

        new_active = active
        if v < low and not active:
            new_active = True
        elif v >= clear and active:
            new_active = False

        if new_active == active:
            return

        with self._lock:
            self._alarm_active = new_active

        setter = getattr(self._hw, "set_battery_alarm", None)
        if not callable(setter):
            logger.warning(
                "Hardware controller lacks set_battery_alarm(); battery alarm not driven"
            )
            return
        try:
            setter(new_active)
            logger.warning(
                "Battery low-voltage alarm %s (V=%.2f, thresholds low=%.2f clear=%.2f)",
                "RAISED" if new_active else "cleared",
                v,
                low,
                clear,
            )
        except Exception as exc:
            logger.error("Failed to drive battery alarm: %s", exc)

    # ── Charge tracker ───────────────────────────────────────────────────

    def _get_tracker(self, cfg: dict[str, Any]) -> Any:
        from doris_battery.charge_tracker import FullChargeTracker

        path = Path(cfg.get("charge_state_path", DEFAULT_CONFIG["charge_state_path"]))
        threshold = float(cfg.get("full_charge_soc_percent", 98.0))
        if (
            self._charge_tracker is None
            or self._tracker_path != path
            or self._tracker_threshold != threshold
        ):
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                self._charge_tracker = FullChargeTracker(path, full_charge_soc_percent=threshold)
                self._tracker_path = path
                self._tracker_threshold = threshold
            except Exception as exc:
                logger.warning("Could not initialize charge tracker at %s: %s", path, exc)
                self._charge_tracker = None
        return self._charge_tracker

    # ── CSV logging ──────────────────────────────────────────────────────

    def _append_csv(self, snapshot: dict[str, Any], cfg: dict[str, Any]) -> None:
        if self._csv_init_failed:
            return
        from doris_battery.logger import flatten_snapshot, is_resting

        log_dir = Path(cfg.get("log_dir", DEFAULT_CONFIG["log_dir"]))
        threshold = float(cfg.get("rest_current_threshold_a", 0.5))

        try:
            log_dir.mkdir(parents=True, exist_ok=True)
        except Exception as exc:
            logger.warning("Battery CSV: could not create %s: %s", log_dir, exc)
            self._csv_init_failed = True
            return

        if self._csv_path is None or self._csv_path.parent != log_dir:
            try:
                self._init_csv(log_dir)
            except Exception as exc:
                logger.warning("Battery CSV: init failed: %s", exc)
                self._csv_init_failed = True
                return

        # Try to add a wall-clock timestamp to the filename once time is synced.
        if not self._csv_timestamped:
            self._maybe_timestamp_csv()

        try:
            resting = is_resting(snapshot, threshold)
        except Exception:
            resting = False

        try:
            row = flatten_snapshot(snapshot, resting=resting)
        except Exception as exc:
            logger.warning("Battery CSV: flatten failed: %s", exc)
            return

        self._write_row(row)

    def _init_csv(self, log_dir: Path) -> None:
        """Pick the next sequence number and create an empty per-power-up file."""
        next_seq = self._next_sequence(log_dir)
        path = log_dir / f"battery_{next_seq:04d}.csv"
        # Touch the file so subsequent scans count it. The header is written on
        # the first row write (when we know the full field set).
        path.touch(exist_ok=True)
        self._csv_path = path
        self._csv_sequence = next_seq
        self._csv_fieldnames = []
        self._csv_timestamped = False
        logger.info("Battery CSV logging to %s", path)

    @staticmethod
    def _next_sequence(log_dir: Path) -> int:
        highest = 0
        try:
            for entry in log_dir.iterdir():
                if not entry.is_file():
                    continue
                match = _SEQUENCE_RE.match(entry.name)
                if not match:
                    continue
                try:
                    n = int(match.group(1))
                except ValueError:
                    continue
                if n > highest:
                    highest = n
        except FileNotFoundError:
            pass
        return highest + 1

    def _maybe_timestamp_csv(self) -> None:
        if self._csv_path is None or self._csv_sequence is None:
            return
        try:
            synced = self._time_synced_fn()
        except Exception:
            synced = None
        if not synced:
            return
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        new_path = self._csv_path.parent / f"battery_{self._csv_sequence:04d}_{stamp}.csv"
        if new_path == self._csv_path:
            self._csv_timestamped = True
            return
        try:
            os.replace(self._csv_path, new_path)
        except Exception as exc:
            logger.warning(
                "Battery CSV: rename %s -> %s failed: %s",
                self._csv_path.name,
                new_path.name,
                exc,
            )
            # Don't keep retrying every poll on persistent errors.
            self._csv_timestamped = True
            return
        logger.info("Battery CSV renamed to %s (time synced)", new_path.name)
        self._csv_path = new_path
        self._csv_timestamped = True

    def _write_row(self, row: dict[str, Any]) -> None:
        import csv as _csv

        path = self._csv_path
        if path is None:
            return

        existing_fieldnames = list(self._csv_fieldnames)
        # On first write to an existing file (e.g. after a crash mid-run) pick
        # up its header so we don't double-write columns.
        if not existing_fieldnames:
            try:
                if path.exists() and path.stat().st_size > 0:
                    with path.open(newline="", encoding="utf-8") as handle:
                        reader = _csv.DictReader(handle)
                        existing_fieldnames = list(reader.fieldnames or [])
            except Exception:
                existing_fieldnames = []

        fieldnames = list(existing_fieldnames)
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)

        write_header = not path.exists() or path.stat().st_size == 0
        # If the header on disk grew (new cells appeared mid-run), rewrite it.
        rewrite_header = (
            not write_header
            and existing_fieldnames
            and fieldnames != existing_fieldnames
        )

        try:
            if rewrite_header:
                self._rewrite_with_new_header(path, fieldnames)
                write_header = False
            with path.open("a", newline="", encoding="utf-8") as handle:
                writer = _csv.DictWriter(handle, fieldnames=fieldnames, extrasaction="ignore")
                if write_header:
                    writer.writeheader()
                writer.writerow(row)
            self._csv_fieldnames = fieldnames
        except Exception as exc:
            logger.warning("Battery CSV: write failed: %s", exc)

    @staticmethod
    def _rewrite_with_new_header(path: Path, fieldnames: list[str]) -> None:
        import csv as _csv

        tmp = path.with_suffix(path.suffix + ".tmp")
        with path.open(newline="", encoding="utf-8") as src, tmp.open(
            "w", newline="", encoding="utf-8"
        ) as dst:
            reader = _csv.DictReader(src)
            writer = _csv.DictWriter(dst, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for existing_row in reader:
                writer.writerow(existing_row)
        os.replace(tmp, path)
