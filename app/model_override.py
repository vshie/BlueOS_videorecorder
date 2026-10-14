"""
Install the DropCam 3D model as BlueOS's vehicle model override.

BlueOS's frontend (Digital Twin on the home page, Vehicle Setup, PWM setup,
frame selector, accel calibration) loads ``/userdata/modeloverrides/ALL.glb``
in place of its built-in vehicle model whenever that file exists
(``core/frontend/src/components/vehiclesetup/viewers/modelHelper.ts``).
nginx serves ``/userdata/`` from host ``/usr/blueos/userdata`` with caching
off, so the next page load picks the file up.

The same model is used for 4k Cam and USB camera units — it's one housing.

Install paths, in order:
  1. BlueOS 1.5+ customization API: ``POST /customization/v1.0/models``
     (multipart, ``.glb`` only, 20 MB max — our model is < 1 MB).
  2. BlueOS 1.4.x has no such API: run ``curl`` on the host through the
     commander to pull the file from our own HTTP server into
     ``/usr/blueos/userdata/modeloverrides``.

We never overwrite an ``ALL.glb`` we didn't put there: an unknown hash is
treated as the user's own model and left alone. The hash we last installed
is kept in the extension config so a newer bundled model can replace an
older one of ours.
"""
from __future__ import annotations

import hashlib
import logging
import os
import threading
import time
from typing import Any, Callable

logger = logging.getLogger(__name__)

MODEL_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static", "models", "dropcam.glb")
OVERRIDE_NAME = "ALL.glb"
OVERRIDE_URL = f"http://localhost/userdata/modeloverrides/{OVERRIDE_NAME}"
CUSTOMIZATION_API = "http://localhost/customization/v1.0/models"
HOST_DIR = "/usr/blueos/userdata/modeloverrides"
# Our own Flask server, as the host sees it (the extension uses host networking).
SELF_MODEL_URL = "http://127.0.0.1:5423/static/models/dropcam.glb"

_lock = threading.Lock()
_status: dict[str, Any] = {"state": "pending", "detail": "", "method": None}


def get_status() -> dict[str, Any]:
    with _lock:
        return dict(_status)


def _set_status(state: str, detail: str = "", method: str | None = None) -> None:
    with _lock:
        _status.update(state=state, detail=detail, method=method)


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def bundled_model() -> tuple[bytes, str]:
    with open(MODEL_FILE, "rb") as f:
        data = f.read()
    return data, _sha256(data)


def _fetch_current() -> bytes | None:
    """Return the installed ALL.glb, None if absent. Raises if BlueOS is unreachable."""
    import requests

    resp = requests.get(OVERRIDE_URL, timeout=10)
    if resp.status_code == 404:
        return None
    resp.raise_for_status()
    return resp.content


def _install_via_api(data: bytes) -> bool:
    import requests

    try:
        resp = requests.post(
            CUSTOMIZATION_API,
            params={"name": OVERRIDE_NAME},
            files={"file": (OVERRIDE_NAME, data, "model/gltf-binary")},
            timeout=30,
        )
    except requests.RequestException as exc:
        logger.debug("model_override: customization API unreachable: %s", exc)
        return False
    if resp.ok:
        return True
    logger.debug("model_override: customization API -> HTTP %s: %s", resp.status_code, resp.text[:200])
    return False


def _install_via_commander() -> bool:
    from deckhand_host_setup import _run_host_command

    tmp = f"{HOST_DIR}/.{OVERRIDE_NAME}.tmp"
    cmd = (
        f"mkdir -p {HOST_DIR} && "
        f"curl -fsS --max-time 60 {SELF_MODEL_URL} -o {tmp} && "
        f"mv -f {tmp} {HOST_DIR}/{OVERRIDE_NAME}"
    )
    rc, _, err = _run_host_command(cmd, timeout=90.0)
    if rc != 0:
        logger.warning("model_override: host install failed (rc=%s): %s", rc, err.strip()[:200])
    return rc == 0


def _delete_installed() -> bool:
    import requests

    try:
        resp = requests.delete(f"{CUSTOMIZATION_API}/{OVERRIDE_NAME}", timeout=10)
        if resp.status_code in (204, 404):
            return True
    except requests.RequestException:
        pass
    from deckhand_host_setup import _run_host_command

    rc, _, _ = _run_host_command(f"rm -f {HOST_DIR}/{OVERRIDE_NAME}")
    return rc == 0


def ensure(enabled: bool, last_hash: str | None, save_hash: Callable[[str | None], None]) -> dict[str, Any]:
    """Make BlueOS's ALL.glb match the setting. Raises if BlueOS is unreachable."""
    data, ours = bundled_model()
    current = _fetch_current()
    current_hash = _sha256(current) if current is not None else None
    mine = current_hash is not None and current_hash in (ours, last_hash)

    if not enabled:
        if current is None:
            _set_status("disabled", "BlueOS shows its default vehicle model.")
        elif mine:
            if _delete_installed():
                save_hash(None)
                _set_status("disabled", "DropCam model removed; BlueOS shows its default vehicle model.")
            else:
                _set_status("error", "Could not remove the DropCam model from BlueOS.")
        else:
            _set_status("disabled", "A custom ALL.glb is installed in BlueOS (not ours, left alone).")
        return get_status()

    if current_hash == ours:
        if last_hash != ours:
            save_hash(ours)
        _set_status("installed", "DropCam model is BlueOS's vehicle model.")
        return get_status()
    if current is not None and not mine:
        _set_status(
            "custom_present",
            "BlueOS already has a custom ALL.glb model override; leaving it in place. "
            "Delete it in BlueOS (Settings > 3D model overrides, or File Browser "
            "userdata/modeloverrides) to use the DropCam model.",
        )
        return get_status()

    method = "api" if _install_via_api(data) else ("host" if _install_via_commander() else None)
    if method is None:
        _set_status("error", "Could not copy the model into BlueOS userdata.")
        return get_status()
    installed = _fetch_current()
    if installed is None or _sha256(installed) != ours:
        _set_status("error", "Model copy did not verify (hash mismatch).", method)
        return get_status()
    save_hash(ours)
    logger.info("model_override: installed DropCam model as BlueOS %s (via %s)", OVERRIDE_NAME, method)
    _set_status("installed", "DropCam model is BlueOS's vehicle model.", method)
    return get_status()


def ensure_in_background(
    get_settings: Callable[[], tuple[bool, str | None]],
    save_hash: Callable[[str | None], None],
    attempts: int = 20,
) -> threading.Thread:
    """Run ``ensure`` off-thread, retrying while BlueOS's web server isn't up yet."""

    def _run() -> None:
        delay = 5.0
        for attempt in range(1, attempts + 1):
            try:
                enabled, last_hash = get_settings()
                ensure(enabled, last_hash, save_hash)
                return
            except Exception as exc:
                _set_status("pending", f"Waiting for BlueOS ({exc.__class__.__name__}), attempt {attempt}/{attempts}")
                logger.debug("model_override: attempt %d failed: %s", attempt, exc)
            time.sleep(delay)
            delay = min(delay * 1.5, 60.0)
        _set_status("error", "BlueOS web server not reachable; model not installed.")

    t = threading.Thread(target=_run, name="model-override", daemon=True)
    t.start()
    return t
