"""Minimal BCM2711 (Pi 4) pinmux helper.

Some BlueOS services running on the host — most notably ``autopilot_manager``
which reprobes the primary I2C bus for compass/IMU auto-detection at boot —
reconfigure pins we care about out of their alt-function mode. The most
common casualty is GPIO 2 (SDA1): it gets driven as a plain OUTPUT LOW,
which shorts the I2C bus and makes the PCA9685 (0x40) and MCP7940N RTC
(0x6F) both return ``[Errno 5] Input/output error``.

Since the extension container doesn't ship ``raspi-gpio`` and we need to
recover deterministically on Pi 4 (BCM2711), this module pokes the GPFSEL
function-select registers directly via ``/dev/mem``. It's ~10 lines of
memory-mapped I/O and no external dependency.

Pi 5 uses the RP1 chip, which has a completely different pinctrl layout,
so this helper is deliberately Pi 4-only. On Pi 5 (or if /dev/mem isn't
mappable — dev laptop, unprivileged container), :func:`set_alt` returns
False and callers should treat that as "kernel/host handles pinmux
already, no action needed".
"""

from __future__ import annotations

import logging
import mmap
import os
import struct

logger = logging.getLogger(__name__)

# BCM2711 (Pi 4) GPIO peripheral base — see the BCM2711 ARM peripherals
# datasheet, section 5. GPFSEL0..5 sit at offsets 0x00..0x14.
BCM2711_GPIO_BASE = 0xFE200000
GPIO_REG_SIZE = 0x1000  # one 4 KiB page covers GPFSEL0..GPPUPPDN3

# fsel encoding (BCM2835 GPIO chapter, but identical on 2711):
#   000 = input, 001 = output, 100 = alt0, 101 = alt1, 110 = alt2,
#   111 = alt3, 011 = alt4, 010 = alt5
_ALT_TO_FSEL = {
    "in":  0b000,
    "out": 0b001,
    "a0":  0b100,
    "a1":  0b101,
    "a2":  0b110,
    "a3":  0b111,
    "a4":  0b011,
    "a5":  0b010,
}


def _is_bcm2711() -> bool:
    """Best-effort check that we're on a Pi 4 (BCM2711) — /dev/mem access
    on other SoCs would be pointing at the wrong physical address."""
    try:
        with open("/proc/device-tree/compatible", "rb") as fh:
            data = fh.read()
    except OSError:
        return False
    return b"bcm2711" in data or b"raspberrypi,4" in data


def set_alt(gpio: int, alt: str) -> bool:
    """Set BCM GPIO ``gpio`` to function ``alt`` (``in``/``out``/``a0``..``a5``).

    Returns True on success, False if we can't touch /dev/mem or aren't
    on a BCM2711. Failure is silent-by-design — call sites treat this as
    a defensive recovery hook, not a mandatory step.
    """
    if not 0 <= gpio <= 57:
        raise ValueError(f"invalid BCM GPIO {gpio}")
    fsel = _ALT_TO_FSEL.get(alt.lower())
    if fsel is None:
        raise ValueError(f"invalid alt {alt!r}; want one of {list(_ALT_TO_FSEL)}")
    if not _is_bcm2711():
        logger.debug("bcm_pinmux: not on BCM2711, skipping GPIO %d -> %s", gpio, alt)
        return False
    reg_index = gpio // 10   # GPFSEL0 covers 0..9, GPFSEL1 covers 10..19, etc.
    bit_shift = (gpio % 10) * 3
    mask = 0b111 << bit_shift
    value = (fsel & 0b111) << bit_shift
    try:
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    except OSError as exc:
        logger.warning("bcm_pinmux: cannot open /dev/mem (%s); can't reclaim GPIO %d", exc, gpio)
        return False
    try:
        mm = mmap.mmap(fd, GPIO_REG_SIZE, offset=BCM2711_GPIO_BASE, flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ | mmap.PROT_WRITE)
    except OSError as exc:
        os.close(fd)
        logger.warning("bcm_pinmux: mmap failed (%s); can't reclaim GPIO %d", exc, gpio)
        return False
    try:
        off = reg_index * 4
        cur = struct.unpack_from("<I", mm, off)[0]
        new = (cur & ~mask) | value
        if cur != new:
            struct.pack_into("<I", mm, off, new)
    finally:
        mm.close()
        os.close(fd)
    logger.info("bcm_pinmux: set GPIO %d -> %s (fsel=0b%s)", gpio, alt, format(fsel, "03b"))
    return True
