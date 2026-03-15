"""
System telemetry for DropCam: Pi CPU temperature, voltage, clock speed, time sync,
and disk space. Uses sysfs fallbacks for Docker container environments where
vcgencmd and timedatectl may not be available.
"""

import logging
import os
import subprocess
import time
from datetime import datetime

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
    """Read Pi core voltage. Tries vcgencmd first, then sysfs fallback."""
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_volts", "core"], timeout=2, stderr=subprocess.DEVNULL
        ).decode()
        return float(out.split("=")[1].strip().rstrip("V"))
    except Exception:
        pass
    try:
        with open("/sys/devices/platform/soc/soc:firmware/get_throttled", "r") as f:
            throttled = int(f.read().strip(), 16)
        if throttled & 0x1:
            return "Under-voltage"
        return "OK"
    except Exception as e:
        logger.debug(f"CPU voltage read failed: {e}")
    return None


def get_cpu_clock_mhz():
    """Read Pi ARM clock speed in MHz. Tries vcgencmd, then sysfs cpufreq."""
    try:
        out = subprocess.check_output(
            ["vcgencmd", "measure_clock", "arm"], timeout=2, stderr=subprocess.DEVNULL
        ).decode()
        freq_hz = int(out.split("=")[1].strip())
        return round(freq_hz / 1_000_000, 0)
    except Exception:
        pass
    try:
        with open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "r") as f:
            freq_khz = int(f.read().strip())
        return round(freq_khz / 1000, 0)
    except Exception as e:
        logger.debug(f"CPU clock read failed: {e}")
    return None


def is_time_synced():
    """Check if system clock has been synchronized via NTP or browser.
    Returns True if synced, False if not, None if unknown.

    The Pi has no RTC, so time is wrong on boot. It becomes synced when:
    - BlueOS reaches the internet (NTP), or
    - A user loads the BlueOS web interface (browser time sync).
    """
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
    except Exception:
        pass
    if os.path.exists("/run/systemd/timesync/synchronized"):
        return True
    now = datetime.now()
    if now.year < 2024:
        return False
    return None


def get_disk_free_mb(path="/app/videorecordings"):
    """Return free disk space at path in MB."""
    try:
        stat = os.statvfs(path)
        return round((stat.f_bavail * stat.f_frsize) / (1024 * 1024), 1)
    except Exception as e:
        logger.debug(f"Disk free check failed: {e}")
    return None


def get_system_time():
    """Return the current system time as a formatted string."""
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def get_all_telemetry(servo_position=None, light_brightness=None,
                      recipe_name=None, recording_ok=None):
    """Collect all available system telemetry into a dict."""
    return {
        "cpu_temp_c": get_cpu_temperature(),
        "cpu_voltage_v": get_cpu_voltage(),
        "cpu_clock_mhz": get_cpu_clock_mhz(),
        "time_synced": is_time_synced(),
        "system_time": get_system_time(),
        "disk_free_mb": get_disk_free_mb(),
        "servo_position_us": servo_position,
        "light_brightness_pct": light_brightness,
        "recipe_name": recipe_name,
        "recording_ok": recording_ok,
    }
