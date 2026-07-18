"""CSV row helpers for BMS snapshots.

Ported from brianhBR/Doris-Battery. The DropCam extension only uses
`is_resting()` and `flatten_snapshot()`; the per-day CsvLogger is preserved
but unused (the monitor in app/battery.py writes its own per-power-up file).
"""

from __future__ import annotations

import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def is_resting(snapshot: dict[str, Any], current_threshold_a: float) -> bool:
    """True when the pack appears idle (no significant charge/discharge)."""
    summary = snapshot.get("summary", {})
    current = summary.get("current_a")
    if current is None:
        return False

    if abs(current) >= current_threshold_a:
        return False

    if summary.get("charger_running"):
        return False
    if summary.get("load_running"):
        return False

    mode = (snapshot.get("mosfet_status") or {}).get("mode")
    if mode in ("charging", "discharging"):
        return False

    return True


def flatten_snapshot(snapshot: dict[str, Any], resting: bool) -> dict[str, Any]:
    """Convert a nested snapshot into flat CSV columns."""
    summary = snapshot.get("summary", {})
    cell_voltages = snapshot.get("cell_voltages") or {}
    balancing = snapshot.get("balancing_status") or {}
    balancing_cells = [str(cell) for cell, active in sorted(balancing.items()) if active]
    board_number = snapshot.get("board_number", 1)
    pack_name = snapshot.get("pack_name") or f"board{board_number}"

    row: dict[str, Any] = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "board_number": board_number,
        "pack_name": pack_name,
        "total_voltage_v": summary.get("total_voltage_v"),
        "current_a": summary.get("current_a"),
        "soc_percent": summary.get("soc_percent"),
        "awake_uptime": summary.get("awake_uptime"),
        "seconds_awake": summary.get("seconds_awake"),
        # Legacy aliases (same values as awake uptime).
        "since_full_charge": summary.get("awake_uptime")
            or summary.get("since_full_charge"),
        "seconds_since_full_charge": summary.get("seconds_awake")
            if summary.get("seconds_awake") is not None
            else summary.get("seconds_since_full_charge"),
        "mode": summary.get("mode"),
        "cell_delta_mv": summary.get("cell_delta_mv"),
        "highest_cell": summary.get("highest_cell"),
        "lowest_cell": summary.get("lowest_cell"),
        "highest_cell_v": (snapshot.get("cell_voltage_range") or {}).get("highest_voltage"),
        "lowest_cell_v": (snapshot.get("cell_voltage_range") or {}).get("lowest_voltage"),
        "cycles": summary.get("cycles"),
        "capacity_ah": summary.get("capacity_ah"),
        "charger_running": summary.get("charger_running"),
        "load_running": summary.get("load_running"),
        "max_temp_c": (snapshot.get("temperature_range") or {}).get("highest_temperature"),
        "min_temp_c": (snapshot.get("temperature_range") or {}).get("lowest_temperature"),
        "charging_mosfet": (snapshot.get("mosfet_status") or {}).get("charging_mosfet"),
        "discharging_mosfet": (snapshot.get("mosfet_status") or {}).get("discharging_mosfet"),
        "is_resting": resting,
        "balancing_cells": ",".join(balancing_cells) if balancing_cells else "",
        "errors": "; ".join(snapshot.get("errors") or []),
    }

    for cell_id, voltage in sorted(cell_voltages.items(), key=lambda item: int(item[0])):
        row[f"cell_{cell_id}_v"] = voltage

    return row


def write_json_snapshot(snapshot: dict[str, Any], log_dir: Path) -> Path:
    """Write a one-off JSON snapshot for debugging."""
    log_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    pack = snapshot.get("pack_name") or f"board{snapshot.get('board_number', 1)}"
    path = log_dir / f"snapshot_{pack}_{stamp}.json"
    path.write_text(json.dumps(snapshot, indent=2), encoding="utf-8")
    return path
