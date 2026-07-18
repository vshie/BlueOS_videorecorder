"""Track awake uptime gated by battery SOC.

Counter resets when SOC hits 100%, stays at zero until SOC drops to
99% or below, then accumulates only while the monitor is running
(offline gaps are not credited across process restarts).
"""

from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

# Hard thresholds per product request (not the legacy full_charge_soc knob).
RESET_SOC_PERCENT = 100.0
START_SOC_PERCENT = 99.0


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


class AwakeUptimeTracker:
    """Accumulate system-awake time between full-charge resets.

    State machine (per pack board):
      * SOC >= 100% → reset counter to 0, hold (not counting)
      * SOC <= 99%  → start/continue counting
      * 99% < SOC < 100% after a reset → keep holding at 0
      * Boot with SOC <= 99% and no prior hold → start counting
        (system is already awake mid-deployment)
    """

    def __init__(self, state_path: Path, **_ignored: Any) -> None:
        self.state_path = state_path
        self._state: dict[str, dict[str, Any]] = self._load()
        # last_tick is process-local; on load we resume accumulated_s
        # without crediting the offline gap.
        self._last_tick: dict[str, float] = {}

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

    def _entry(self, key: str) -> dict[str, Any]:
        entry = dict(self._state.get(key) or {})
        entry.setdefault("accumulated_s", 0.0)
        entry.setdefault("counting", False)
        entry.setdefault("holding", False)
        return entry

    def observe(self, board: int, snapshot: dict[str, Any]) -> bool:
        """Update awake uptime from a snapshot. Returns True if state changed."""
        summary = snapshot.get("summary") or {}
        soc = summary.get("soc_percent")
        if soc is None:
            return False

        key = str(board)
        entry = self._entry(key)
        now = time.time()
        soc_f = float(soc)
        changed = False

        # Credit elapsed awake time since the previous tick (process uptime
        # only — last_tick is not persisted, so reboots do not add offline
        # time).
        if entry.get("counting"):
            last = self._last_tick.get(key)
            if last is not None and now > last:
                entry["accumulated_s"] = float(entry["accumulated_s"]) + (now - last)
                changed = True
            self._last_tick[key] = now

        if soc_f >= RESET_SOC_PERCENT:
            if float(entry["accumulated_s"]) != 0.0 or entry.get("counting") or not entry.get("holding"):
                changed = True
            entry["accumulated_s"] = 0.0
            entry["counting"] = False
            entry["holding"] = True
            self._last_tick.pop(key, None)
        elif soc_f <= START_SOC_PERCENT:
            if entry.get("holding") or not entry.get("counting"):
                # Leave full-charge hold, or first observation mid-dive.
                if not entry.get("counting"):
                    changed = True
                entry["holding"] = False
                entry["counting"] = True
                self._last_tick[key] = now
            else:
                entry["counting"] = True
                self._last_tick.setdefault(key, now)
        else:
            # 99% < SOC < 100%: hold at zero after a reset; otherwise keep
            # whatever counting state we already had.
            if entry.get("holding"):
                entry["counting"] = False
                self._last_tick.pop(key, None)
            elif entry.get("counting"):
                self._last_tick.setdefault(key, now)

        if entry != self._state.get(key):
            self._state[key] = entry
            return True
        return changed

    def enrich_snapshot(self, snapshot: dict[str, Any]) -> None:
        board = snapshot.get("board_number")
        if board is None:
            return
        key = str(board)
        entry = self._entry(key)
        seconds = float(entry.get("accumulated_s") or 0.0)
        # Include sub-tick elapsed so the UI advances between observes.
        if entry.get("counting"):
            last = self._last_tick.get(key)
            if last is not None:
                seconds += max(0.0, time.time() - last)
        summary = dict(snapshot.get("summary") or {})
        summary["seconds_awake"] = seconds
        summary["awake_uptime"] = format_duration(seconds)
        summary["awake_counting"] = bool(entry.get("counting"))
        # Legacy keys so older UI/log readers keep working until rolled.
        summary["seconds_since_full_charge"] = seconds
        summary["since_full_charge"] = summary["awake_uptime"]
        snapshot["summary"] = summary


# Back-compat alias — battery.py historically imported FullChargeTracker.
FullChargeTracker = AwakeUptimeTracker
