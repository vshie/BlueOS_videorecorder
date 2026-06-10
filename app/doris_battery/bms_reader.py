"""Read all available data from a Daly BMS over RS485.

Ported from brianhBR/Doris-Battery. The Sinowealth code path is preserved
verbatim; the DropCam extension only uses the default A5 path.
"""

from __future__ import annotations

import logging
import struct
import time
from copy import deepcopy
from typing import Any

from dalybms import DalyBMS

logger = logging.getLogger(__name__)


def normalize_mode(mode: str | None) -> str | None:
    """Map Daly BMS mode names to dashboard labels."""
    if mode == "stationary":
        return "idle"
    return mode


def normalize_mosfet_status(mosfet: dict[str, Any]) -> dict[str, Any]:
    data = dict(mosfet)
    data["mode"] = normalize_mode(data.get("mode"))
    return data


def snapshot_from_bms(
    bms: DalyBMS,
    board_number: int | None = None,
    pack_name: str | None = None,
) -> dict[str, Any]:
    """Build a normalized snapshot from a connected DalyBMS instance."""
    status = bms.get_status()
    if not status:
        raise RuntimeError("Failed to read BMS status")

    soc = bms.get_soc() or {}
    cell_range = bms.get_cell_voltage_range() or {}
    temp_range = bms.get_temperature_range() or {}
    mosfet = normalize_mosfet_status(bms.get_mosfet_status() or {})
    cell_voltages = bms.get_cell_voltages()
    if not isinstance(cell_voltages, dict):
        cell_voltages = {}
    temperatures = bms.get_temperatures()
    if not isinstance(temperatures, dict):
        temperatures = {}
    errors = bms.get_errors() or []
    balancing = get_balancing_status(bms) or {}

    highest_v = cell_range.get("highest_voltage", 0.0)
    lowest_v = cell_range.get("lowest_voltage", 0.0)

    snapshot = {
        "soc": soc,
        "cell_voltage_range": cell_range,
        "temperature_range": temp_range,
        "mosfet_status": mosfet,
        "status": status,
        "cell_voltages": cell_voltages,
        "temperatures": temperatures,
        "balancing_status": balancing,
        "errors": errors,
        "summary": {
            "total_voltage_v": soc.get("total_voltage"),
            "current_a": soc.get("current"),
            "soc_percent": soc.get("soc_percent"),
            "mode": mosfet.get("mode"),
            "cell_delta_mv": round((highest_v - lowest_v) * 1000, 1),
            "highest_cell": cell_range.get("highest_cell"),
            "lowest_cell": cell_range.get("lowest_cell"),
            "cycles": status.get("cycles"),
            "capacity_ah": mosfet.get("capacity_ah"),
            "charger_running": status.get("charger_running"),
            "load_running": status.get("load_running"),
        },
    }
    if board_number is not None:
        snapshot["board_number"] = board_number
    if pack_name:
        snapshot["pack_name"] = pack_name
    return snapshot


def quick_snapshot_from_bms(
    bms: DalyBMS,
    board_number: int,
    pack_name: str | None = None,
    previous: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Read SOC for fast polling; reuse prior detail fields when available."""
    soc = bms.get_soc() or {}
    if not soc:
        raise RuntimeError("Failed to read BMS SOC")

    raw_mosfet = None
    for _ in range(3):
        try:
            raw_mosfet = bms.get_mosfet_status()
        except (IndexError, struct.error, TypeError):
            raw_mosfet = None
        if raw_mosfet:
            break
        time.sleep(0.05)
    if not raw_mosfet:
        raw_mosfet = (previous or {}).get("mosfet_status") or {}
    mosfet = normalize_mosfet_status(raw_mosfet)
    snapshot = deepcopy(previous) if previous else {
        "cell_voltage_range": {},
        "temperature_range": {},
        "status": {},
        "cell_voltages": {},
        "temperatures": {},
        "balancing_status": {},
        "errors": [],
        "summary": {},
    }

    snapshot["soc"] = soc
    snapshot["mosfet_status"] = mosfet
    summary = dict(snapshot.get("summary") or {})
    summary.update(
        {
            "total_voltage_v": soc.get("total_voltage"),
            "current_a": soc.get("current"),
            "soc_percent": soc.get("soc_percent"),
            "mode": mosfet.get("mode"),
            "capacity_ah": mosfet.get("capacity_ah"),
        }
    )
    snapshot["summary"] = summary
    snapshot["board_number"] = board_number
    if pack_name:
        snapshot["pack_name"] = pack_name
    return snapshot


def get_balancing_status(bms: DalyBMS) -> dict[int, bool]:
    """Parse cell balancing bitmask (command 0x97)."""
    response_data = bms._read_request("97")
    if not response_data or not bms.status or not isinstance(response_data, (bytes, bytearray)):
        return {}

    bits = bin(int.from_bytes(response_data, byteorder="big"))[2:].zfill(48)
    cells: dict[int, bool] = {}
    for cell in range(1, bms.status["cells"] + 1):
        cells[cell] = bool(int(bits[cell * -1]))
    return cells


class DorisBMSReader:
    """Wrapper around python-daly-bms with balancing status support (single default board)."""

    def __init__(
        self,
        device: str,
        address: int = 4,
        board_number: int = 1,
        request_retries: int = 5,
        baudrate: int = 9600,
        serial_timeout: float = 0.5,
        sinowealth: bool = False,
        pack_name: str | None = None,
    ) -> None:
        self._sinowealth = sinowealth
        self._device = device
        self.board_number = board_number
        self.pack_name = pack_name
        if sinowealth:
            from dalybms import SinowealthBMS

            self._bms: Any = SinowealthBMS(request_retries=request_retries, logger=logger)
        else:
            from doris_battery.a5_bus import BoardDalyBMS

            self._bms = BoardDalyBMS(
                board_number,
                request_retries=request_retries,
                baudrate=baudrate,
                serial_timeout=serial_timeout,
            )

    def connect(self) -> None:
        self._bms.connect(self._device)

    def disconnect(self) -> None:
        try:
            self._bms.disconnect()
        except Exception:
            pass

    def read_snapshot(self) -> dict[str, Any]:
        if self._sinowealth:
            return self._read_sinowealth_snapshot()
        return snapshot_from_bms(self._bms, self.board_number, self.pack_name)

    def read_soc_broadcast(self, duration: float = 3.0, solicit: bool = True) -> dict[str, Any] | None:
        """Listen for an auto-reported / solicited 0x90 SOC frame.

        Returns a raw SOC dict (total_voltage/current/soc_percent) or None.
        Only valid on the A5 (non-Sinowealth) path.
        """
        if self._sinowealth:
            return None
        reader = getattr(self._bms, "read_soc_broadcast", None)
        if not callable(reader):
            return None
        return reader(duration=duration, solicit=solicit)

    def snapshot_from_soc(self, soc: dict[str, Any]) -> dict[str, Any]:
        """Build a minimal snapshot from a broadcast SOC reading.

        Detail fields (cells, temps, mosfet, errors) are left empty because
        the pack only surfaced its SOC frame; voltage/current/SOC populate the
        summary the rest of the extension consumes."""
        snapshot: dict[str, Any] = {
            "soc": soc,
            "cell_voltage_range": {},
            "temperature_range": {},
            "mosfet_status": {},
            "status": {},
            "cell_voltages": {},
            "temperatures": {},
            "balancing_status": {},
            "errors": [],
            "summary": {
                "total_voltage_v": soc.get("total_voltage"),
                "current_a": soc.get("current"),
                "soc_percent": soc.get("soc_percent"),
            },
            "board_number": self.board_number,
        }
        if self.pack_name:
            snapshot["pack_name"] = self.pack_name
        return snapshot

    def _read_sinowealth_snapshot(self) -> dict[str, Any]:
        soc = self._bms.get_soc() or {}
        cell_voltages = self._bms.get_cell_voltages() or {}
        temperatures = self._bms.get_temperatures() or {}
        errors = self._bms.get_errors() or []

        if cell_voltages:
            values = list(cell_voltages.values())
            highest_v = max(values)
            lowest_v = min(values)
            highest_cell = max(cell_voltages, key=cell_voltages.get)
            lowest_cell = min(cell_voltages, key=cell_voltages.get)
        else:
            highest_v = lowest_v = 0.0
            highest_cell = lowest_cell = None

        snapshot = {
            "soc": soc,
            "cell_voltage_range": {
                "highest_voltage": highest_v,
                "highest_cell": highest_cell,
                "lowest_voltage": lowest_v,
                "lowest_cell": lowest_cell,
            },
            "temperature_range": {},
            "mosfet_status": {},
            "status": {"cells": len(cell_voltages), "temperature_sensors": len(temperatures)},
            "cell_voltages": cell_voltages,
            "temperatures": temperatures,
            "balancing_status": {},
            "errors": errors,
            "summary": {
                "total_voltage_v": soc.get("total_voltage"),
                "current_a": soc.get("current"),
                "soc_percent": soc.get("soc_percent"),
                "mode": None,
                "cell_delta_mv": round((highest_v - lowest_v) * 1000, 1),
                "highest_cell": highest_cell,
                "lowest_cell": lowest_cell,
                "cycles": None,
                "capacity_ah": None,
                "charger_running": None,
                "load_running": None,
            },
        }
        if self.pack_name:
            snapshot["pack_name"] = self.pack_name
        return snapshot
