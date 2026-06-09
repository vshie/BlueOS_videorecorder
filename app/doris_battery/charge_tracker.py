"""Track time since last full charge per pack.

Ported verbatim from brianhBR/Doris-Battery.
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any


def is_charging(snapshot: dict[str, Any]) -> bool:
    summary = snapshot.get("summary") or {}
    mosfet = snapshot.get("mosfet_status") or {}
    current = summary.get("current_a")
    if summary.get("charger_running"):
        return True
    if mosfet.get("mode") == "charging" or mosfet.get("charging_mosfet"):
        return True
    if current is not None and float(current) > 0.5:
        return True
    return False


def format_duration(seconds: float | None) -> str:
    if seconds is None or seconds < 0:
        return "—"
    total = int(seconds)
    days = total // 86400
    hours = (total % 86400) // 3600
    minutes = (total % 3600) // 60
    if days:
        return f"{days}d {hours}h"
    if hours:
        return f"{hours}h {minutes}m"
    if minutes:
        return f"{minutes}m"
    return "<1m"


class FullChargeTracker:
    """Record when a pack reaches full SOC while charging."""

    def __init__(self, state_path: Path, full_charge_soc_percent: float = 98.0) -> None:
        self.state_path = state_path
        self.full_charge_soc_percent = full_charge_soc_percent
        self._state: dict[str, dict[str, Any]] = self._load()

    def _load(self) -> dict[str, dict[str, Any]]:
        if not self.state_path.exists():
            return {}
        try:
            with self.state_path.open(encoding="utf-8") as handle:
                data = json.load(handle)
            return data if isinstance(data, dict) else {}
        except (OSError, json.JSONDecodeError):
            return {}

    def save(self) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        with self.state_path.open("w", encoding="utf-8") as handle:
            json.dump(self._state, handle, indent=2)

    def observe(self, board: int, snapshot: dict[str, Any]) -> bool:
        """Update state from a snapshot. Returns True if state changed."""
        summary = snapshot.get("summary") or {}
        soc = summary.get("soc_percent")
        if soc is None:
            return False

        key = str(board)
        entry = dict(self._state.get(key) or {})
        was_at_full = bool(entry.get("was_at_full"))
        at_full = float(soc) >= self.full_charge_soc_percent
        charging = is_charging(snapshot)
        changed = False

        if at_full:
            if charging and not was_at_full:
                entry["last_full_charge_at"] = time.time()
                changed = True
            entry["was_at_full"] = True
        else:
            if was_at_full:
                changed = True
            entry["was_at_full"] = False

        if entry != self._state.get(key):
            self._state[key] = entry
            return True
        return changed

    def enrich_snapshot(self, snapshot: dict[str, Any]) -> None:
        board = snapshot.get("board_number")
        if board is None:
            return
        entry = self._state.get(str(board)) or {}
        last_at = entry.get("last_full_charge_at")
        seconds_since = None
        if isinstance(last_at, (int, float)):
            seconds_since = max(0.0, time.time() - float(last_at))
        summary = dict(snapshot.get("summary") or {})
        summary["last_full_charge_at"] = last_at
        summary["seconds_since_full_charge"] = seconds_since
        summary["since_full_charge"] = format_duration(seconds_since)
        snapshot["summary"] = summary
