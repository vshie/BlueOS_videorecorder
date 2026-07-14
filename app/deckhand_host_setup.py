"""
Idempotent DeckHand host bring-up: I2C-based PCB detection, /boot/firmware/config.txt
verification/patching, and automatic reboot when the host config needs changes.

Runs on every extension start. Non-destructive on non-DeckHand hardware:

  1. Probe I2C bus 1 for the DeckHand signature (PCA9685 @ 0x40 AND
     MCP7940N @ 0x6F both responding). If either device is missing we
     conclude this isn't a DeckHand PCB — no host changes are made.

  2. On a confirmed DeckHand, read the active /boot/firmware/config.txt
     (Bookworm) or /boot/config.txt (Bullseye) and check that the first
     ``[pi4]`` section contains our required DeckHand overrides. The
     Rev-A firmware assumes the J104 silkscreen has been relabelled so
     GPIO 10 = rotation sensor input and GPIO 20 = WS2812 LED (SPI1
     MOSI); the required overrides encode that pinout:
        dtoverlay=uart4          (TXD4/RXD4 only — DO NOT add ,ctsrts
                                  because that would claim GPIO 10 for
                                  CTS4 and steal the rotation-sensor pin)
        dtoverlay=uart3-off      (free GPIO 4 for PCA9685 ~OE)
        dtoverlay=spi0-led-off   (belt-and-braces — keep SPI0 off GPIO 10
                                  even if a downstream dtparam=spi=on
                                  gets added by another agent)
     These lines are DELIBERATELY written WITHOUT the usual
     ``# custom - ...`` inline marker: the RPi firmware ``dtoverlay=``
     parser silently rejects lines whose value has trailing whitespace
     followed by ``#`` (empirically verified 2026-07-14: with the marker
     UART4 was not enumerated at boot; without it, UART4 came up
     immediately). The BlueOS reconciler doesn't strip these three
     overlays anyway — none of them match any of its known
     conflicting-configuration patterns for ``[pi4]``.

     Two other pieces of state we DON'T persist in config.txt on purpose:
       * ``dtoverlay=spi1-1cs`` — the reconciler auto-adds ``spi1-3cs``
         which also enables /dev/spidev1.0 on GPIO 20 (WS2812 MOSI),
         so we ride on the reconciler's line and skip our own.
       * ``gpio=11=a4,pn`` — the reconciler ALWAYS insists on
         ``gpio=11,24,25=op,pu,dh`` and unless our override carries
         ``# custom`` (which corrupts the parser for gpio= lines too)
         it just gets stripped every boot. Instead, ``bcm_pinmux`` in
         ``main.py`` forces GPIO 11 -> ALT4 RTS4 directly via /dev/mem
         at extension startup — reliable and doesn't touch config.txt.

  3. If any lines are missing or wrong, patch the file (in the first
     [pi4] section, where the reconciler looks) and reboot the host.

  4. Even when the file is textually correct, the Pi firmware's boot-time
     overlay processing can silently drop an overlay (SPI1 vs. onboard
     audio contention on BCM2711 is a known example). So we also verify
     that each overlay's expected runtime device — /dev/i2c-1,
     /dev/spidev1.0, and the UART4 tty (resolved by MMIO 7e201800) —
     actually exists. If any is missing we first try ``sudo dtoverlay
     <name>`` on the host to load it live (this reliably fixes spi1-1cs
     without a reboot), and only escalate ``reboot_required=True`` when
     that runtime recovery fails too.

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
import glob
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
# previous writes and Navigator/reconciler-added variants (including
# older `# custom - DeckHand: ...` variants that we now know break
# the RPi firmware overlay parser — we rewrite those to the plain
# form on next apply).
#
# CANONICAL LINES CARRY NO INLINE COMMENT: with an inline `# custom`
# marker, the RPi firmware ``dtoverlay=`` parser treats the whole
# trailing string as part of the overlay name and silently skips the
# line at boot. The BlueOS reconciler doesn't strip any of these three
# lines anyway (none of them match its `[pi4]` conflict patterns for
# Navigator hardware), so we don't need marker protection.
#
# uart4 is DELIBERATELY plain (no ``,ctsrts``) — the DeckHand's rotation
# sensor lives on GPIO 10, which ``ctsrts`` would steal for CTS4.
# uart3-off frees GPIO 4/5 for PCA9685 ~OE and I2C respectively.
# spi0-led-off is belt-and-braces: it neutralises the ``spi0-led``
# overlay if some other agent ever adds it (would clobber our LED pin
# via GPIO 10). SPI0 itself is disabled by not having ``dtparam=spi=on``
# in our overrides — the reconciler adds it but the SPI0 driver just
# fails to bind because GPIO 9 (SPI0_MISO) is already taken by UART4
# (RXD4), which is benign.
WANTED_OVERLAYS: list[tuple[str, str]] = [
    # The regex intentionally matches BOTH `dtoverlay=uart4` and any
    # ``dtoverlay=uart4  # custom - ...`` from previous versions of the
    # extension, so a re-apply from an old install cleans up the broken
    # form in place.
    (r"^dtoverlay=uart4(?:,\S+)?(?:\s+#.*)?$",         "dtoverlay=uart4"),
    (r"^dtoverlay=uart3(?:-off)?(?:\s+#.*)?$",         "dtoverlay=uart3-off"),
    (r"^dtoverlay=spi0-led(?:-off)?(?:\s+#.*)?$",      "dtoverlay=spi0-led-off"),
]

# Runtime devices we expect to exist on a booted DeckHand host. Each entry
# is ``(path_glob_or_probe_key, human_label, overlay_to_reload)`` — the
# glob is the pattern we check for on the extension side (via /dev), the
# overlay name is what we ask the host to reload at runtime if the device
# is missing but its overlay is textually present in config.txt. We use
# ``uart4:mmio`` for the UART4 tty because its /dev/ttyAMA<N> index isn't
# fixed (see BatteryMonitor._find_pi_uart_by_mmio) — we resolve it by
# matching the sysfs of_node MMIO address 7e201800 instead.
EXPECTED_DEVICES: list[tuple[str, str, str | None]] = [
    ("/dev/i2c-1",       "I2C bus 1 (PCA9685 + MCP7940N RTC)",   "i2c1"),
    # /dev/spidev1.0 is provided by the reconciler's ``dtoverlay=spi1-3cs``
    # (auto-added by BlueOS). We used to add our own ``spi1-1cs``, but
    # the reconciler wins that fight anyway and both variants expose
    # /dev/spidev1.0 on MOSI=GPIO 20, which is all the WS2812 backend
    # actually cares about. If /dev/spidev1.0 is somehow missing we can
    # still runtime-recover with ``dtoverlay spi1-1cs``.
    ("/dev/spidev1.0",   "SPI1 device 0 (WS2812 RGB LED)",       "spi1-1cs"),
    ("uart4:mmio",       "UART4 tty (Daly BMS RS-485)",          None),
]

# Some overlays (uart4 in particular) can't be safely (re)loaded at
# runtime — swapping the pin muxing for GPIO 8/9 out from under a
# running kernel driver risks dropping into a bad state and needs a
# reboot to take effect from config.txt cleanly.
NO_RUNTIME_RELOAD = {"uart4"}

# Public status dict — read by main.py's /telemetry and /host_setup routes.
_STATUS: dict[str, Any] = {
    "ran": False,
    "is_deckhand": None,        # True / False / None (couldn't tell)
    "config_valid": None,       # True / False / None
    "config_path": None,        # e.g. "/boot/firmware/config.txt"
    "commander_reachable": None,
    "problems": [],             # list[str] of missing config lines or runtime devices
    "missing_devices": [],      # list[str] of expected /dev entries not present
    "runtime_recovered": [],    # list[str] of overlays we loaded live to fix a gap
    "changes_applied": False,   # did we just write the config?
    "reboot_required": False,   # config or runtime device gap needs a reboot
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
    boot-time compass probe on the Pi 4 — we defensively restore the
    ALT0 pinmux, then retry the scan a few times with backoff because
    autopilot_manager's compass probe can also transiently hold the bus
    for the first second or two after our container starts, causing an
    initial false-negative (empirically confirmed 2026-07-14: PCA9685
    and RTC both respond a few seconds later on the same boot).
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

    # Retry the whole open+scan up to 5 times with exponential backoff.
    # A single attempt is ~10 ms of bus activity, so worst-case total
    # delay is ~3.1 s — acceptable at boot and cheap on the happy path
    # where the first attempt succeeds.
    delays = [0.0, 0.2, 0.4, 0.8, 1.6]
    last_found: list[int] = []
    for attempt, delay in enumerate(delays, 1):
        if delay:
            time.sleep(delay)
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
            last_found = found
            if set(found) == set(DECKHAND_I2C_ADDRS):
                if attempt > 1:
                    logger.info(
                        "host_setup: I2C bus %d scan succeeded on attempt %d "
                        "(devices responded at %s) — is_deckhand=True",
                        I2C_BUS, attempt, [f"0x{a:02x}" for a in found],
                    )
                else:
                    logger.info(
                        "host_setup: I2C bus %d scan responded at %s — is_deckhand=True",
                        I2C_BUS, [f"0x{a:02x}" for a in found],
                    )
                return True
        finally:
            try:
                bus.close()
            except Exception:
                pass

    logger.info(
        "host_setup: I2C bus %d scan after %d retries responded at %s "
        "(needed %s) — is_deckhand=False",
        I2C_BUS, len(delays),
        [f"0x{a:02x}" for a in last_found] or "nothing",
        [f"0x{a:02x}" for a in DECKHAND_I2C_ADDRS],
    )
    return False


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

    We also flag any *broken* forms of our overlays — i.e. the same
    overlay name with a trailing ``#`` inline comment, which the RPi
    firmware silently rejects at boot. On next apply we'll rewrite
    those lines to the plain form.
    """
    lines = config_text.splitlines()
    bounds = _first_pi4_section_bounds(lines)
    if bounds is None:
        return ["[pi4] section missing"]
    start, end = bounds
    section = lines[start + 1 : end]

    problems: list[str] = []
    for regex, canonical in WANTED_OVERLAYS:
        pat = re.compile(regex)
        matched = [l for l in section if pat.match(l)]
        if not matched:
            problems.append(f"missing: {canonical}")
        elif canonical not in matched:
            # Line is present but in a form the firmware parser rejects
            # (typically the legacy ``... # custom - ...`` variant we
            # used to write). Needs to be rewritten to the plain form.
            problems.append(f"needs rewrite -> {canonical}")
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

    # NOTE: We do NOT persist a ``gpio=11=a4,pn`` override here anymore.
    # The BlueOS reconciler always fights that line (it insists on
    # ``gpio=11,24,25=op,pu,dh``), and the ``# custom`` marker we'd
    # need to survive the reconciler also corrupts the firmware parser
    # for that line. Instead, ``bcm_pinmux.set_alt(11, "a4")`` in
    # ``main.py`` forces GPIO 11 to ALT4 RTS4 directly via /dev/mem at
    # extension startup — reliable and doesn't need config.txt.

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


# ----- runtime device presence + on-the-fly overlay recovery ------------

def _find_uart4_tty() -> str | None:
    """Resolve UART4's /dev/ttyAMA<N> by MMIO address (7e201800 on BCM2711).

    The ttyAMA index isn't stable across kernels/overlay-load-order, so we
    walk /sys/class/tty/ttyAMA*/device/of_node and match the target address
    the same way BatteryMonitor does. Returns None if UART4 isn't wired
    through to any tty yet (usually means dtoverlay=uart4 didn't take).
    """
    for tty_dir in sorted(glob.glob("/sys/class/tty/ttyAMA*")):
        of_link = os.path.join(tty_dir, "device", "of_node")
        try:
            target = os.path.realpath(of_link)
        except OSError:
            continue
        base = os.path.basename(target).lower()
        if "@" in base and base.split("@", 1)[1] == "7e201800":
            return "/dev/" + os.path.basename(tty_dir)
    return None


def _device_present(probe_key: str) -> bool:
    """Return True if the runtime device identified by ``probe_key`` exists.

    ``probe_key`` is either a filesystem path (checked with os.path.exists)
    or a special sentinel starting with ``uart4:`` (resolved via MMIO
    scan). Kept small and dependency-free so it works both inside the
    container and on the host during debugging.
    """
    if probe_key.startswith("uart4:"):
        return _find_uart4_tty() is not None
    return os.path.exists(probe_key)


def _check_runtime_devices() -> list[tuple[str, str, str | None]]:
    """Return the subset of EXPECTED_DEVICES that aren't currently exposed
    to /dev (or, for UART4, aren't wired to any /dev/ttyAMA<N>).

    Empty list = every runtime device our overlays are supposed to
    create is actually present. Non-empty means either (a) the config
    file is textually correct but boot-time overlay processing silently
    failed, or (b) something on the host is un-mapping the pin after
    boot. Either way we escalate to the operator.
    """
    missing: list[tuple[str, str, str | None]] = []
    for probe, label, overlay in EXPECTED_DEVICES:
        if not _device_present(probe):
            missing.append((probe, label, overlay))
    return missing


def _try_runtime_dtoverlay_load(overlay: str) -> bool:
    """Attempt ``sudo dtoverlay <overlay>`` on the host via commander.

    Returns True if the command succeeded (rc == 0). We use this as a
    best-effort recovery when the boot-time overlay silently didn't
    take — for spi1-1cs this reliably creates /dev/spidev1.0 with no
    reboot, which is a much better operator experience than a forced
    reboot cycle.
    """
    try:
        rc, out, err = _run_host_command(
            f"sudo dtoverlay {overlay}", timeout=10.0
        )
        if rc == 0:
            logger.info(
                "host_setup: runtime dtoverlay %s applied successfully", overlay
            )
            return True
        logger.warning(
            "host_setup: runtime dtoverlay %s failed rc=%d stderr=%s",
            overlay, rc, err[:200].strip(),
        )
    except Exception as exc:
        logger.warning(
            "host_setup: runtime dtoverlay %s errored: %s", overlay, exc
        )
    return False


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

            # Textual config is right; now verify the runtime devices those
            # overlays are supposed to have created actually exist. Boot-time
            # overlay processing on the Pi firmware can silently fail (e.g.
            # SPI1 vs. onboard audio contention on BCM2711), which our
            # text-only diff can't see. Try to recover in-band first via
            # ``dtoverlay <name>`` on the host, and only escalate to a
            # reboot notice if that fails too.
            missing = _check_runtime_devices()
            _STATUS["missing_devices"] = [label for _, label, _ in missing]

            recovered_via_runtime: list[str] = []
            still_missing: list[tuple[str, str, str | None]] = []
            for probe, label, overlay in missing:
                if overlay and overlay not in NO_RUNTIME_RELOAD and \
                        _try_runtime_dtoverlay_load(overlay):
                    if _device_present(probe):
                        recovered_via_runtime.append(label)
                        continue
                still_missing.append((probe, label, overlay))

            _STATUS["runtime_recovered"] = recovered_via_runtime
            _STATUS["missing_devices"] = [l for _, l, _ in still_missing]

            if not still_missing:
                if recovered_via_runtime:
                    _STATUS["detail"] = (
                        f"DeckHand host config OK ({config_path}); "
                        f"loaded {len(recovered_via_runtime)} missing overlay(s) "
                        f"at runtime: {', '.join(recovered_via_runtime)}"
                    )
                    logger.info("host_setup: %s", _STATUS["detail"])
                else:
                    _STATUS["detail"] = f"DeckHand host config OK ({config_path})"
                    logger.info("host_setup: %s", _STATUS["detail"])
                _save_state({"last_signature": None, "attempts": 0})
                return get_status()

            # A device is still missing after runtime attempts — surface it
            # so the frontend banner can prompt the operator to reboot. The
            # config file is fine on disk; only a reboot has any chance of
            # fixing the boot-time overlay-processing quirk.
            _STATUS["reboot_required"] = True
            _STATUS["problems"] = [
                f"{label} missing at runtime (expected {probe})"
                for probe, label, _ in still_missing
            ]
            _STATUS["detail"] = (
                f"DeckHand host config OK on disk but "
                f"{len(still_missing)} expected runtime device(s) missing "
                f"({', '.join(l for _, l, _ in still_missing)}); "
                "reboot required to retry boot-time overlay processing"
            )
            logger.warning("host_setup: %s", _STATUS["detail"])
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
