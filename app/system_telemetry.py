"""
System telemetry for DropCam: Pi CPU temperature, voltage, clock speed, time sync,
and disk space. Uses sysfs fallbacks for Docker container environments where
vcgencmd and timedatectl may not be available.
"""

import ctypes
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


def _check_adjtimex():
    """Check kernel NTP discipline status via adjtimex(2).

    The container shares the host kernel, so this works even in Docker.
    Return values: 0-4 = clock synchronized, 5 = TIME_ERROR (unsynchronized).
    """
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        # struct timex is ~200 bytes; modes=0 (read-only) is the first field.
        buf = (ctypes.c_char * 256)()
        ctypes.memset(buf, 0, 256)
        result = libc.adjtimex(buf)
        if result == 5:          # TIME_ERROR — kernel clock not synced
            return False
        if 0 <= result <= 4:     # TIME_OK / TIME_INS / TIME_DEL / TIME_OOP / TIME_WAIT
            return True
    except Exception as e:
        logger.debug(f"adjtimex check failed: {e}")
    return None


def is_time_synced():
    """Check if system clock has been synchronized via NTP or browser.
    Returns True if synced, False if not, None if unknown.

    The Pi has no RTC, so time is wrong on boot. It becomes synced when:
    - BlueOS reaches the internet (NTP), or
    - A user loads the BlueOS web interface (browser time sync).
    """
    # Preferred: ask the kernel directly (works inside Docker)
    result = _check_adjtimex()
    if result is not None:
        return result

    # Fallback: timedatectl (works on host, not in Docker)
    try:
        out = subprocess.check_output(
            ["timedatectl", "show", "--property=NTPSynchronized"],
            timeout=2, stderr=subprocess.DEVNULL,
        ).decode().strip()
        return out == "NTPSynchronized=yes"
    except Exception:
        pass

    if os.path.exists("/run/systemd/timesync/synchronized"):
        return True

    # Last resort: Pi has no RTC, so a current year means time was set somehow
    if datetime.now().year >= 2025:
        return True
    if datetime.now().year < 2024:
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
