"""Kernel-driven RS-485 half-duplex direction control via ``TIOCSRS485``.

When the PL011 (amba-pl011) driver is new enough (Linux ≥ 5.14) *and* the
UART's RTS pin is wired to the transceiver's DE/~RE line, the kernel can
flip DE with hardware-precision timing — the RTS pin goes HIGH just before
the first TX bit and LOW just after the last stop bit shifts out. That's
the same auto-direction behaviour an FT232 gives to the BlueRobotics
BLUART USB adapter, only routed through the SoC's UART controller instead
of a USB serial chip.

On the DeckHand PCB this means:
    * ``dtoverlay=uart4,ctsrts`` in ``/boot/firmware/config.txt``
      routes GPIO 11 to the PL011 RTS4 alt-function.
    * The SN65HVD75 DE/~RE pin (tied together on the board) is wired
      to GPIO 11, so RTS4 physically drives DE.
    * A ``TIOCSRS485`` ioctl on the open ``/dev/ttyAMA*`` fd tells the
      kernel to hold RTS HIGH during TX (the DeckHand's active-high DE
      convention) and LOW otherwise, with sub-microsecond timing.

Older kernels (Linux 5.10 that ships in BlueOS 1.4.x / Bullseye) accept
``TIOCGRS485`` but return ``ENOTTY`` on ``TIOCSRS485``. In that case
``try_enable_rs485()`` returns False and the caller should fall back to
its software DE-toggle path (``a5_bus.BoardDalyBMS.set_direction_control``).
"""

from __future__ import annotations

import fcntl
import logging
import os
import struct
from typing import Any

logger = logging.getLogger(__name__)

# ioctl numbers from <asm-generic/ioctls.h>. Same values on all Linux archs
# we care about (armhf, arm64, x86_64), so hard-coding is fine.
TIOCGRS485 = 0x542E
TIOCSRS485 = 0x542F

# struct serial_rs485 layout from <linux/serial.h>:
#     __u32 flags;
#     __u32 delay_rts_before_send;   /* ms */
#     __u32 delay_rts_after_send;    /* ms */
#     __u32 padding[5];              /* reserved */
# Total 32 bytes.
_STRUCT_FMT = "IIIIIIII"
_STRUCT_SIZE = struct.calcsize(_STRUCT_FMT)  # 32

# Flag bits from <linux/serial.h>.
SER_RS485_ENABLED        = 1 << 0
SER_RS485_RTS_ON_SEND    = 1 << 1  # RTS pin level while TX'ing
SER_RS485_RTS_AFTER_SEND = 1 << 2  # RTS pin level after TX

# DeckHand DE/~RE polarity: HIGH while driving, LOW when receiving.
DECKHAND_FLAGS = SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND


def _serial_fd(port: Any) -> int | None:
    """Extract a file descriptor from a pyserial Serial (or int fd)."""
    if isinstance(port, int):
        return port
    fd = getattr(port, "fd", None)
    if isinstance(fd, int) and fd >= 0:
        return fd
    fileno = getattr(port, "fileno", None)
    if callable(fileno):
        try:
            f = fileno()
            if isinstance(f, int) and f >= 0:
                return f
        except (OSError, ValueError):
            return None
    return None


def try_enable_rs485(
    port: Any,
    *,
    flags: int = DECKHAND_FLAGS,
    delay_before_send_ms: int = 0,
    delay_after_send_ms: int = 0,
) -> bool:
    """Try to switch the PL011 (or other) tty into hardware RS-485 mode.

    ``port`` may be a pyserial ``Serial`` instance or a raw fd. Returns
    ``True`` if the kernel accepted the ioctl (i.e. the driver will now
    drive RTS for us), ``False`` on any failure — the caller should keep
    using its software DE toggle in that case.
    """
    fd = _serial_fd(port)
    if fd is None:
        logger.debug("rs485_kernel: no fd on %r; skipping ioctl", port)
        return False
    cfg = struct.pack(
        _STRUCT_FMT,
        flags,
        max(0, int(delay_before_send_ms)),
        max(0, int(delay_after_send_ms)),
        0, 0, 0, 0, 0,
    )
    try:
        fcntl.ioctl(fd, TIOCSRS485, cfg)
    except OSError as exc:
        logger.debug("rs485_kernel: TIOCSRS485 failed on fd=%d: %s", fd, exc)
        return False
    # Read back so we log what the driver actually accepted (some drivers
    # silently mask bits they don't support).
    try:
        buf = bytearray(_STRUCT_SIZE)
        fcntl.ioctl(fd, TIOCGRS485, buf)
        got_flags, got_before, got_after = struct.unpack(_STRUCT_FMT, bytes(buf))[:3]
        logger.info(
            "rs485_kernel: TIOCSRS485 accepted (flags=0x%x, before=%dms, after=%dms)",
            got_flags, got_before, got_after,
        )
    except OSError:
        logger.info("rs485_kernel: TIOCSRS485 accepted (readback failed)")
    return True


def is_kernel_rs485_supported(port: Any) -> bool:
    """Best-effort check without leaving RS-485 mode enabled.

    Returns True iff the driver accepts a set-then-clear ioctl pair.
    Useful when the caller wants to decide up front whether to install
    a software DE toggle, without leaving RTS in an unknown state.
    """
    fd = _serial_fd(port)
    if fd is None:
        return False
    try:
        cfg = struct.pack(_STRUCT_FMT, SER_RS485_ENABLED, 0, 0, 0, 0, 0, 0, 0)
        fcntl.ioctl(fd, TIOCSRS485, cfg)
        # Turn it back off so we don't leave the driver in an unexpected state.
        fcntl.ioctl(fd, TIOCSRS485, struct.pack(_STRUCT_FMT, 0, 0, 0, 0, 0, 0, 0, 0))
    except OSError:
        return False
    return True


def disable_rs485(port: Any) -> None:
    """Clear RS-485 mode on the tty. Safe to call unconditionally."""
    fd = _serial_fd(port)
    if fd is None:
        return
    try:
        fcntl.ioctl(fd, TIOCSRS485, struct.pack(_STRUCT_FMT, 0, 0, 0, 0, 0, 0, 0, 0))
    except OSError:
        pass
