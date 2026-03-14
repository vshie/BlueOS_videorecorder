"""
System telemetry for DropCam: Pi CPU temperature, voltage, clock speed, and time sync.
Replaces the MAVLink-based telemetry from the original extension.
"""

import logging
import os
import subprocess
import time

logger = logging.getLogger(__name__)


def get_cpu_temperature():
    """Read Pi CPU temperature in degrees C from sysfs."""
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            return round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        pass
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_temp"], timeout=2, stderr=subprocess.DEVNULL
        ).decode()
        return float(out.split("=")[1].split("'")[0])
    except Exception as e:
        logger.debug(f"CPU temp read failed: {e}")
    return None


def get_cpu_voltage():
    """Read Pi core voltage via vcgencmd. Returns volts as float or None."""
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_volts", "core"], timeout=2, stderr=subprocess.DEVNULL
        ).decode()
        return float(out.split("=")[1].strip().rstrip("V"))
    except Exception as e:
        logger.debug(f"CPU voltage read failed: {e}")
    return None


def get_cpu_clock_mhz():
    """Read Pi ARM clock speed in MHz via vcgencmd."""
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_clock", "arm"], timeout=2, stderr=subprocess.DEVNULL
        ).decode()
        freq_hz = int(out.split("=")[1].strip())
        return round(freq_hz / 1_000_000, 0)
    except Exception as e:
        logger.debug(f"CPU clock read failed: {e}")
    return None


def is_time_synced():
    """Check if system clock has been synchronized via NTP or browser.
    Returns True if synced, False if not, None if unknown."""
    try:
        out = subprocess.check_output(
            ["timedatectl", "show", "--property=NTPSynchronized"],
            timeout=2, stderr=subprocess.DEVNULL,
        ).decode().strip()
        return out == "NTPSynchronized=yes"
    except Exception:
        pass
    try:
        out = subprocess.check_output(
            ["timedatectl", "status"], timeout=2, stderr=subprocess.DEVNULL
        ).decode()
        for line in out.splitlines():
            if "synchronized" in line.lower():
                return "yes" in line.lower()
    except Exception as e:
        logger.debug(f"Time sync check failed: {e}")
    return None


def get_disk_free_mb(path="/app/videorecordings"):
    """Return free disk space at path in MB."""
    try:
        stat = os.statvfs(path)
        return round((stat.f_bavail * stat.f_frsize) / (1024 * 1024), 1)
    except Exception as e:
        logger.debug(f"Disk free check failed: {e}")
    return None


def get_all_telemetry(servo_position=None, light_brightness=None,
                      recipe_name=None, recording_ok=None):
    """Collect all available system telemetry into a dict."""
    return {
        "cpu_temp_c": get_cpu_temperature(),
        "cpu_voltage_v": get_cpu_voltage(),
        "cpu_clock_mhz": get_cpu_clock_mhz(),
        "time_synced": is_time_synced(),
        "disk_free_mb": get_disk_free_mb(),
        "servo_position_us": servo_position,
        "light_brightness_pct": light_brightness,
        "recipe_name": recipe_name,
        "recording_ok": recording_ok,
    }
