"""
USB external storage detection, mounting, and health monitoring for DropCam.

Scans for removable block devices, mounts the first usable partition, and
exposes state for the rest of the application.  A background probe thread
periodically checks for newly-inserted USB drives when the system is idle.
"""

import glob
import logging
import os
import subprocess
import threading
import time

logger = logging.getLogger(__name__)

USB_MOUNT_POINT = "/mnt/usb"
USB_MIN_FREE_GB = 20
DROPCAM_DIR = "DropCam"
PROBE_INTERVAL_S = 30

_lock = threading.Lock()
_mounted = False
_device = None          # e.g. "/dev/sda1"
_probe_thread = None
_stop_probe = threading.Event()


# ── Detection ────────────────────────────────────────────────────────────

def _scan_usb_devices():
    """Return a list of partition device paths on removable block devices.

    Only returns actual partitions (e.g. /dev/sda1).  Whole-disk devices
    without a partition table are skipped on purpose: mounting them blocks
    the kernel filesystem probe and can hang `mount` indefinitely.
    """
    partitions = []
    for block in sorted(glob.glob("/sys/block/sd*")):
        try:
            with open(os.path.join(block, "removable"), "r") as f:
                if f.read().strip() != "1":
                    continue
        except Exception:
            continue
        dev_name = os.path.basename(block)
        found_any = False
        for part in sorted(glob.glob(os.path.join(block, dev_name + "[0-9]*"))):
            part_name = os.path.basename(part)
            dev_path = f"/dev/{part_name}"
            if os.path.exists(dev_path):
                partitions.append(dev_path)
                found_any = True
        if not found_any:
            logger.debug(
                f"USB block {dev_name} has no partitions; skipping whole-disk mount "
                "(raw device without a partition table cannot be mounted safely)"
            )
    return partitions


# ── Mount / unmount ──────────────────────────────────────────────────────

def is_mounted():
    """Check whether USB_MOUNT_POINT is an active mount."""
    try:
        with open("/proc/mounts", "r") as f:
            for line in f:
                if USB_MOUNT_POINT in line.split():
                    return True
    except Exception:
        pass
    return False


def try_mount():
    """Detect and mount the first usable USB partition.  Returns True on success."""
    global _mounted, _device

    with _lock:
        if _mounted and is_mounted():
            return True

        partitions = _scan_usb_devices()
        if not partitions:
            _mounted = False
            _device = None
            return False

        os.makedirs(USB_MOUNT_POINT, exist_ok=True)

        if is_mounted():
            _mounted = True
            _device = _device or partitions[0]
            return True

        for dev in partitions:
            try:
                result = subprocess.run(
                    ["mount", "-o", "rw", dev, USB_MOUNT_POINT],
                    capture_output=True, timeout=10,
                )
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"mount {dev} timed out after 10s; device may be unresponsive "
                    "or have a corrupt/unrecognized filesystem. Skipping."
                )
                continue
            except Exception as e:
                logger.warning(f"mount {dev} raised: {e}; skipping")
                continue
            if result.returncode == 0:
                _mounted = True
                _device = dev
                logger.info(f"USB mounted: {dev} -> {USB_MOUNT_POINT}")
                return True
            logger.debug(f"mount {dev} failed: {result.stderr.decode(errors='replace').strip()}")

        _mounted = False
        _device = None
        return False


def unmount():
    """Unmount USB storage if mounted."""
    global _mounted, _device
    with _lock:
        if is_mounted():
            try:
                subprocess.run(
                    ["umount", USB_MOUNT_POINT], capture_output=True, timeout=10,
                )
                logger.info("USB unmounted")
            except subprocess.TimeoutExpired:
                logger.warning("umount timed out after 10s; leaving state stale")
            except Exception as e:
                logger.warning(f"umount raised: {e}")
        _mounted = False
        _device = None


# ── Health / space ───────────────────────────────────────────────────────

def is_healthy():
    """Fast health check: can we stat the mount point?"""
    if not _mounted:
        return False
    try:
        os.statvfs(USB_MOUNT_POINT)
        return True
    except Exception:
        return False


def get_free_mb():
    """Return free space in MB on the USB mount, or None if unavailable."""
    if not _mounted:
        return None
    try:
        st = os.statvfs(USB_MOUNT_POINT)
        return round((st.f_bavail * st.f_frsize) / (1024 * 1024), 1)
    except Exception:
        return None


def is_usable():
    """USB is mounted and has at least USB_MIN_FREE_GB free."""
    free = get_free_mb()
    if free is None:
        return False
    return free >= USB_MIN_FREE_GB * 1024


def get_recording_dir(subfolder_name):
    """Return the full path for a recording subfolder on USB, creating it."""
    base = os.path.join(USB_MOUNT_POINT, DROPCAM_DIR, subfolder_name)
    os.makedirs(base, exist_ok=True)
    return base


def get_status():
    """Return a status dict for the API."""
    mounted = _mounted and is_mounted()
    free = get_free_mb() if mounted else None
    return {
        "mounted": mounted,
        "device": _device,
        "free_mb": free,
        "usable": is_usable() if mounted else False,
        "mount_point": USB_MOUNT_POINT,
    }


# ── Background probe ────────────────────────────────────────────────────

def _probe_loop():
    """Periodically scan and mount USB when idle."""
    while not _stop_probe.is_set():
        if not (_mounted and is_mounted()):
            try:
                try_mount()
            except Exception as e:
                logger.debug(f"USB probe error: {e}")
        _stop_probe.wait(PROBE_INTERVAL_S)


def start_probe():
    """Start the background USB probe thread."""
    global _probe_thread
    if _probe_thread and _probe_thread.is_alive():
        return
    _stop_probe.clear()
    _probe_thread = threading.Thread(target=_probe_loop, daemon=True, name="usb-probe")
    _probe_thread.start()
    logger.info("USB probe thread started")


def stop_probe():
    """Stop the background probe thread."""
    _stop_probe.set()
    if _probe_thread and _probe_thread.is_alive():
        _probe_thread.join(timeout=5)
