"""
Idempotent DeckHand host bring-up: I2C-based PCB detection, /boot/firmware/config.txt
verification/patching, and automatic reboot when the host config needs changes.

Runs on every extension start. Non-destructive on non-DeckHand hardware:

  1. Probe I2C bus 1 for the DeckHand signature (PCA9685 @ 0x40 AND
     MCP7940N @ 0x6F both responding). If either device is missing we
     conclude this isn't a DeckHand PCB — no host changes are made.

  2. On a confirmed DeckHand, read the active /boot/firmware/config.txt
     (Bookworm) or /boot/config.txt (Bullseye) and check that the first
     ``[pi4]`` section contains our required DeckHand overrides:
        dtoverlay=uart4,ctsrts   (RTS4 on GPIO 11 for kernel TIOCSRS485)
        dtoverlay=uart3-off      (free GPIO 4 for PCA9685 ~OE)
        dtoverlay=spi0-led-off   (free GPIO 10 which would otherwise be SPI0_MOSI)
        dtoverlay=spi1-3cs-off   (free GPIO 20 for rotation-sensor input)
        gpio=11=a4,pn            (force GPIO 11 to ALT4 RTS4)
     Each line carries a `# custom - DeckHand:` marker so BlueOS's
     ``blueos_startup_update`` reconciler leaves it alone.

  3. If any lines are missing or wrong, patch the file (in the first
     [pi4] section, where the reconciler looks) and reboot the host.

All host-side file writes and the reboot itself go through BlueOS's
commander HTTP API at ``http://localhost/commander/v1.0/``. Our
container uses ``NetworkMode: host`` so ``localhost`` == the Pi's
loopback and no additional bind mounts / capabilities are required.

Boot-loop protection: we count how many boots in a row we've applied
changes. If we've patched N times without the config sticking, we
stop trying and just log — the operator can then look at what's
undoing our changes.
"""

from __future__ import annotations

import base64
import json
import logging
import os
import re
import time
from pathlib import Path
from typing import Any

try:
    import requests  # noqa: F401  (used lazily to keep dev-machine imports light)
except ImportError:
    requests = None  # type: ignore

logger = logging.getLogger(__name__)

# BlueOS core services are reachable from any host-networked container on
# localhost. Nginx on port 80 forwards `/commander/` to the commander
# service on port 9100, so this URL works whether the container has
# its own network stack or shares the host's.
COMMANDER_BASE = "http://localhost/commander/v1.0"

# DeckHand signature: PCA9685 (servo/PWM driver) and MCP7940N (RTC),
# both on I2C bus 1. Neither is present on Navigator flight-controller
# HATs, so this pair is a reliable "am I on the right board?" test.
I2C_BUS = 1
DECKHAND_I2C_ADDRS = (0x40, 0x6F)

# Persistent state on the extension's storage volume — used to detect
# reboot loops (we've patched the config N times but it's not sticking).
STATE_FILE = Path("/app/videorecordings/deckhand_host_setup_state.json")
MAX_PATCH_ATTEMPTS = 3

# Lines we want in the first [pi4] section. Each entry is
#   (regex-that-matches-any-existing-variant, canonical-line-to-write).
# The regex is deliberately permissive so we recognise both our own
# previous writes and Navigator/reconciler-added variants.
WANTED_OVERLAYS: list[tuple[str, str]] = [
    (r"^dtoverlay=uart4(?:,\S+)?(?:\s+#.*)?$",
     "dtoverlay=uart4,ctsrts  # custom - DeckHand: RTS4 on GPIO 11 for kernel TIOCSRS485 DE"),
    (r"^dtoverlay=uart3(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=uart3-off  # custom - DeckHand: GPIO 4 needed for PCA9685 ~OE"),
    (r"^dtoverlay=spi0-led(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=spi0-led-off  # custom - DeckHand: GPIO 10 must not go to SPI0_MOSI"),
    (r"^dtoverlay=spi1-3cs(?:-off)?(?:\s+#.*)?$",
     "dtoverlay=spi1-3cs-off  # custom - DeckHand: GPIO 20 needed as rotation-sensor input"),
]

# Reconciler always adds `gpio=11,24,25=op,pu,dh` to force those pins on
# for Navigator hardware. We can't remove it (reconciler would re-add on
# next boot), but Pi firmware processes gpio= lines top-to-bottom and
# later per-pin settings override earlier list entries — so we sneak our
# override in immediately after the list line, forcing GPIO 11 into ALT4
# RTS4 for kernel-driven RS-485 DE. GPIO 24/25 keep their required
# op,pu,dh state.
GPIO_1124_REGEX = re.compile(r"^gpio=[^=]*\b11\b[^=]*=op,pu,dh(?:\s+#.*)?$")
GPIO_1124_CANONICAL = "gpio=11,24,25=op,pu,dh"
GPIO_11_OVERRIDE = (
    "gpio=11=a4,pn  # custom - DeckHand: force GPIO 11 to ALT4 (RTS4) instead of op,pu,dh"
)

# Public status dict — read by main.py's /telemetry and /host_setup routes.
_STATUS: dict[str, Any] = {
    "ran": False,
    "is_deckhand": None,        # True / False / None (couldn't tell)
    "config_valid": None,       # True / False / None
    "config_path": None,        # e.g. "/boot/firmware/config.txt"
    "commander_reachable": None,
    "problems": [],             # list[str] of missing / wrong config lines
    "changes_applied": False,   # did we just write the config?
    "reboot_required": False,   # if changes_applied and reboot not yet triggered
    "reboot_triggered": False,  # did we call the commander shutdown endpoint?
    "attempts_this_state": 0,   # from STATE_FILE (loop protection)
    "last_error": None,
    "detail": "not yet run",
}


def get_status() -> dict[str, Any]:
    """Return a snapshot of the last setup pass — cheap, safe from any thread."""
    return dict(_STATUS)


# ----- internal helpers --------------------------------------------------

def _http_post(path: str, params: dict[str, Any], timeout: float = 10.0) -> Any:
    """POST to a commander endpoint, return parsed JSON or raise."""
    if requests is None:
        raise RuntimeError("python-requests not installed; can't reach commander")
    url = f"{COMMANDER_BASE}{path}"
    resp = requests.post(url, params=params, timeout=timeout)
    if resp.status_code != 200:
        raise RuntimeError(f"commander {path} -> HTTP {resp.status_code}: {resp.text[:200]}")
    try:
        return resp.json()
    except json.JSONDecodeError:
        return resp.text


def _run_host_command(cmd: str, timeout: float = 10.0) -> tuple[int, str, str]:
    """Execute ``cmd`` as root on the BlueOS host via the commander HTTP API.

    Returns ``(return_code, stdout, stderr)``. Commander stringifies stdout
    and stderr with ``repr()``, so bytes look like ``"b'hello\\n'"`` and
    strings look like ``"'hello\\n'"`` — either way, ``ast.literal_eval``
    turns the outer wrapper back into the raw bytes/str, and we decode
    bytes as UTF-8 (replacing anything invalid) so callers get plain text.
    """
    import ast

    data = _http_post(
        "/command/host",
        {"command": cmd, "i_know_what_i_am_doing": "true"},
        timeout=timeout,
    )
    if not isinstance(data, dict):
        raise RuntimeError(f"unexpected commander response: {data!r}")

    def _unwrap(s: Any) -> str:
        if not isinstance(s, str):
            return str(s)
        try:
            value = ast.literal_eval(s)
        except (ValueError, SyntaxError):
            return s
        if isinstance(value, bytes):
            return value.decode("utf-8", errors="replace")
        return str(value)

    return (
        int(data.get("return_code", -1)),
        _unwrap(data.get("stdout", "")),
        _unwrap(data.get("stderr", "")),
    )


def _load_state() -> dict[str, Any]:
    """Load the persistent boot-loop counter. Missing file / bad JSON = fresh state."""
    try:
        return json.loads(STATE_FILE.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return {"last_signature": None, "attempts": 0}


def _save_state(state: dict[str, Any]) -> None:
    try:
        STATE_FILE.parent.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(state))
    except OSError as exc:
        logger.warning("host_setup: couldn't persist state to %s: %s", STATE_FILE, exc)


# ----- DeckHand detection -----------------------------------------------

def _detect_deckhand() -> bool | None:
    """Return True if BOTH PCA9685 (0x40) and MCP7940N (0x6F) respond on
    I2C bus 1, False if the bus works but the devices are missing, or
    None if we can't even open /dev/i2c-1.

    Called with GPIO 2/3 potentially still corrupted by autopilot_manager's
    boot-time compass probe on the Pi 4 — try a defensive pinmux recovery
    first so this function works both from the extension boot path (where
    ``main.py`` has already restored ALT0) and standalone.
    """
    try:
        from bcm_pinmux import set_alt
        set_alt(2, "a0")
        set_alt(3, "a0")
    except Exception:
        # Pi 5 / dev laptop / no /dev/mem access — nothing to recover.
        pass
    try:
        import smbus2
    except ImportError:
        logger.warning("host_setup: smbus2 not installed; can't scan I2C")
        return None
    try:
        bus = smbus2.SMBus(I2C_BUS)
    except (FileNotFoundError, PermissionError) as exc:
        logger.info("host_setup: /dev/i2c-%d unavailable (%s) — I2C not enabled?",
                    I2C_BUS, exc)
        return None
    try:
        found: list[int] = []
        for addr in DECKHAND_I2C_ADDRS:
            try:
                # The PCA9685 doesn't ACK a bare `read_byte` (which sends
                # START+ADDR+READ+STOP with no register offset) — it wants
                # a register write first. Use ``read_byte_data(addr, 0)``
                # instead, which reads register 0 (MODE1 on PCA9685,
                # RTCSEC on MCP7940N — both valid on their target devices).
                bus.read_byte_data(addr, 0x00)
                found.append(addr)
            except OSError:
                pass
        is_deckhand = set(found) == set(DECKHAND_I2C_ADDRS)
        logger.info(
            "host_setup: I2C bus %d scan responded at %s — is_deckhand=%s",
            I2C_BUS,
            [f"0x{a:02x}" for a in found] or "nothing",
            is_deckhand,
        )
        return is_deckhand
    finally:
        try:
            bus.close()
        except Exception:
            pass


# ----- config.txt read / write / verify ---------------------------------

def _resolve_config_path() -> str:
    """Return the active /boot config.txt path on the host.

    Bookworm-based BlueOS 1.5.x puts the real file at
    ``/boot/firmware/config.txt`` (with a stub at ``/boot/config.txt``
    that says "moved"). Bullseye-based 1.4.x keeps it at
    ``/boot/config.txt``. Prefer the Bookworm path if it exists.
    """
    rc, out, err = _run_host_command(
        "if [ -f /boot/firmware/config.txt ]; then "
        "echo /boot/firmware/config.txt; "
        "else echo /boot/config.txt; fi"
    )
    if rc != 0:
        raise RuntimeError(f"couldn't resolve config.txt path (rc={rc}, err={err[:200]})")
    path = out.strip().splitlines()[-1] if out.strip() else "/boot/config.txt"
    return path


def _read_config(path: str) -> str:
    rc, out, err = _run_host_command(f"sudo cat {path}")
    if rc != 0:
        raise RuntimeError(f"couldn't read {path}: rc={rc}, stderr={err[:200]}")
    return out


def _first_pi4_section_bounds(lines: list[str]) -> tuple[int, int] | None:
    """Return (start_line_index, end_line_index_exclusive) for the first
    [pi4] block. blueos_startup_update.py bounds the section at the first
    blank line or the next [tag] header — we mirror that so our writes
    land in exactly the same range it inspects.
    """
    start = next(
        (i for i, l in enumerate(lines) if re.match(r"^\[pi4\]\s*$", l)),
        None,
    )
    if start is None:
        return None
    end = len(lines)
    for i in range(start + 1, len(lines)):
        if lines[i] == "" or re.match(r"^\[.+\]\s*$", lines[i]):
            end = i
            break
    return start, end


def _needed_changes(config_text: str) -> list[str]:
    """Return a list of DeckHand overrides that are missing / wrong in
    the first [pi4] section. Empty list = config is already correct.
    """
    lines = config_text.splitlines()
    bounds = _first_pi4_section_bounds(lines)
    if bounds is None:
        return ["[pi4] section missing"]
    start, end = bounds
    section = lines[start + 1 : end]

    problems: list[str] = []
    for _regex, canonical in WANTED_OVERLAYS:
        if canonical not in section:
            problems.append(canonical)

    # GPIO 11 override must come *immediately after* the reconciler's
    # gpio=11,24,25=op,pu,dh line — otherwise firmware ordering means
    # the list-form line wins and GPIO 11 ends up as OUTPUT instead of
    # ALT4 RTS4.
    idx_1124 = next(
        (i for i, l in enumerate(section) if GPIO_1124_REGEX.match(l)),
        None,
    )
    if idx_1124 is None:
        problems.append(f"{GPIO_1124_CANONICAL} + {GPIO_11_OVERRIDE}")
    else:
        after = section[idx_1124 + 1] if idx_1124 + 1 < len(section) else ""
        if after.strip() != GPIO_11_OVERRIDE.strip():
            problems.append(GPIO_11_OVERRIDE)
    return problems


def _rewrite_config(config_text: str) -> str:
    """Return a new config.txt with DeckHand overrides applied in the
    first [pi4] section. Idempotent — running twice on the same input
    yields byte-identical output.
    """
    lines = config_text.splitlines()
    bounds = _first_pi4_section_bounds(lines)
    if bounds is None:
        # No [pi4] block at all — append one at the end.
        lines.extend(["", "[pi4]"])
        bounds = (len(lines) - 1, len(lines))
    start, end = bounds
    section = lines[start + 1 : end]

    # Overlay lines: replace any existing variant in place, or if none is
    # present insert at the first stable anchor (before the first gpio=
    # line, else at end of section). Doing this per-line means we never
    # strip-and-reinsert, so line order is stable across reruns.
    for pattern, canonical in WANTED_OVERLAYS:
        pat = re.compile(pattern)
        matched_idx = next((i for i, l in enumerate(section) if pat.match(l)), None)
        if matched_idx is not None:
            section[matched_idx] = canonical
            continue
        insert_at = next(
            (i for i, l in enumerate(section) if l.startswith("gpio=")),
            len(section),
        )
        section.insert(insert_at, canonical)

    # GPIO 11 override: must sit immediately after the reconciler's
    # list-form ``gpio=11,24,25=op,pu,dh`` line so Pi firmware's
    # top-to-bottom gpio= processing lets the per-pin override win.
    # If a prior run already put it there, leave it alone.
    idx_1124 = next(
        (i for i, l in enumerate(section) if GPIO_1124_REGEX.match(l)),
        None,
    )
    if idx_1124 is None:
        section.append(GPIO_1124_CANONICAL)
        section.append(GPIO_11_OVERRIDE)
    else:
        after = section[idx_1124 + 1] if idx_1124 + 1 < len(section) else None
        if after is None or after.strip() != GPIO_11_OVERRIDE.strip():
            # Drop any stale copy elsewhere in the section first, so we
            # don't end up with two copies of the override.
            section = [l for l in section if l.strip() != GPIO_11_OVERRIDE.strip()]
            # Re-find idx_1124 (positions may have shifted if we filtered).
            idx_1124 = next(
                (i for i, l in enumerate(section) if GPIO_1124_REGEX.match(l)),
                None,
            )
            assert idx_1124 is not None
            section.insert(idx_1124 + 1, GPIO_11_OVERRIDE)

    return "\n".join(lines[: start + 1] + section + lines[end:]) + "\n"


def _write_config(path: str, new_content: str) -> None:
    """Atomically replace the host config.txt with new_content via commander."""
    ts = time.strftime("%Y%m%d-%H%M%S")
    # Backup first — cheap and lets a human diff later if anything goes wrong.
    rc, _, err = _run_host_command(f"sudo cp {path} {path}.bak-deckhand-{ts}")
    if rc != 0:
        raise RuntimeError(f"config backup failed: {err[:200]}")
    # Base64-encode to sidestep shell quoting entirely.
    b64 = base64.b64encode(new_content.encode("utf-8")).decode("ascii")
    cmd = f"echo '{b64}' | base64 -d | sudo tee {path} > /dev/null"
    rc, _, err = _run_host_command(cmd, timeout=15.0)
    if rc != 0:
        raise RuntimeError(f"config write failed: {err[:200]}")


# ----- reboot ------------------------------------------------------------

def _trigger_reboot() -> None:
    """Ask the commander to reboot the host (schedules `sudo reboot` in
    the background 5 s from now — this HTTP call returns first)."""
    _http_post(
        "/shutdown",
        {"shutdown_type": "reboot", "i_know_what_i_am_doing": "true"},
        timeout=5.0,
    )


# ----- top-level orchestrator -------------------------------------------

def run_startup_setup(auto_reboot: bool = False) -> dict[str, Any]:
    """One-shot startup pass. See module docstring for the full flow.

    ``auto_reboot`` defaults to False — when config changes are needed the
    extension patches the file and sets ``reboot_required=True`` in the
    status dict so the frontend can show a banner with an explicit
    "Reboot now" button (POST /host_setup/reboot). We deliberately do
    not reboot behind the operator's back: they may be in the middle of
    a dive, saving a recording, or debugging over SSH.
    """
    _STATUS["ran"] = True
    _STATUS["last_error"] = None

    # 1) DeckHand detection is a hard prerequisite for any host mutation.
    is_deckhand = _detect_deckhand()
    _STATUS["is_deckhand"] = is_deckhand
    if is_deckhand is None:
        _STATUS["detail"] = (
            "I2C bus 1 unavailable — can't detect DeckHand PCB; skipping host config"
        )
        logger.warning("host_setup: %s", _STATUS["detail"])
        return get_status()
    if is_deckhand is False:
        _STATUS["detail"] = (
            "not a DeckHand PCB (PCA9685@0x40 or MCP7940N@0x6F missing); "
            "leaving host config alone"
        )
        logger.info("host_setup: %s", _STATUS["detail"])
        return get_status()

    # 2) Reach the commander so we can read/write /boot/firmware/config.txt.
    try:
        rc, _, _ = _run_host_command("uname -r", timeout=5.0)
        _STATUS["commander_reachable"] = rc == 0
    except Exception as exc:
        _STATUS["commander_reachable"] = False
        _STATUS["last_error"] = f"commander unreachable: {exc}"
        _STATUS["detail"] = (
            "DeckHand detected but BlueOS commander HTTP API is unreachable; "
            "host config can't be verified/patched from the extension"
        )
        logger.warning("host_setup: %s (%s)", _STATUS["detail"], exc)
        return get_status()

    try:
        config_path = _resolve_config_path()
        _STATUS["config_path"] = config_path
        config_text = _read_config(config_path)
        problems = _needed_changes(config_text)
        _STATUS["problems"] = problems

        if not problems:
            _STATUS["config_valid"] = True
            _STATUS["detail"] = f"DeckHand host config OK ({config_path})"
            logger.info("host_setup: %s", _STATUS["detail"])
            # Clear the loop counter — we've reached a good steady state.
            _save_state({"last_signature": None, "attempts": 0})
            return get_status()

        # 3) Config needs changes. Loop protection: if we've applied the
        # same set of changes MAX_PATCH_ATTEMPTS times without them sticking,
        # something is undoing our writes — stop rebooting and just log.
        signature = "\n".join(sorted(problems))
        state = _load_state()
        if state.get("last_signature") == signature:
            state["attempts"] = int(state.get("attempts", 0)) + 1
        else:
            state = {"last_signature": signature, "attempts": 1}
        _STATUS["attempts_this_state"] = state["attempts"]
        _save_state(state)

        if state["attempts"] > MAX_PATCH_ATTEMPTS:
            _STATUS["config_valid"] = False
            _STATUS["detail"] = (
                f"config still wrong after {state['attempts']} patch attempts — "
                "giving up to avoid a reboot loop; check what's rewriting "
                f"{config_path}"
            )
            logger.error("host_setup: %s", _STATUS["detail"])
            return get_status()

        logger.warning(
            "host_setup: DeckHand config missing %d line(s): %s — patching",
            len(problems),
            ", ".join(problems),
        )
        new_content = _rewrite_config(config_text)
        _write_config(config_path, new_content)
        _STATUS["changes_applied"] = True
        _STATUS["config_valid"] = True   # verified after reboot
        _STATUS["reboot_required"] = True

        if auto_reboot:
            logger.warning(
                "host_setup: rebooting host in ~5 s to apply DeckHand config"
            )
            try:
                _trigger_reboot()
                _STATUS["reboot_triggered"] = True
                _STATUS["detail"] = (
                    f"applied {len(problems)} config change(s) to {config_path}; "
                    "host reboot triggered"
                )
            except Exception as exc:
                _STATUS["last_error"] = f"reboot trigger failed: {exc}"
                _STATUS["detail"] = (
                    f"config patched but reboot trigger failed ({exc}); "
                    "please reboot the Pi manually"
                )
                logger.error("host_setup: %s", _STATUS["detail"])
        else:
            _STATUS["detail"] = (
                f"applied {len(problems)} config change(s) to {config_path}; "
                "MANUAL REBOOT REQUIRED"
            )
            logger.warning("host_setup: %s", _STATUS["detail"])

        return get_status()

    except Exception as exc:
        logger.exception("host_setup: unexpected error: %s", exc)
        _STATUS["last_error"] = str(exc)
        _STATUS["detail"] = f"error while patching config: {exc}"
        return get_status()
