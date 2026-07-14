"""
USB-camera microphone helpers for the DropCam pipeline.

The BlueOS mavlink-camera-manager owns the video half of the USB
composite device (v4l2 / /dev/video*) but leaves the ALSA capture side
completely alone, so we can open the mic (``hw:Camera,0``) in parallel
with the RTSP video stream and mux both into the same MP4.

Two quirks force the two small utilities in this module:

  1. **The Camera card boots with ``Mic Capture Switch = off``** even
     though ``Mic Capture Volume`` is at 256/256 (0 dB). Both
     ``arecord`` and ``alsasrc`` happily open the device and stream
     zero samples for the entire recording — you get a valid MP4
     that plays but has silent audio. :func:`ensure_mic_unmuted`
     issues the equivalent of ``amixer -c <N> sset Mic cap`` at
     extension boot and again just before recording, defensively.

  2. **The Camera card index isn't deterministic** — depending on
     which USB port the camera is plugged into (and whether any
     other USB audio device is present) it may land as ``card 1``,
     ``card 2``, or something else. :func:`find_capture_device`
     walks ``/proc/asound/cards`` looking for the Camera line and
     returns both the numeric card index (for amixer) and the
     ``hw:<name>,<device>`` handle (for GStreamer).

Excluded from RadCam mode: the RadCam is a bare external RTSP
camera with no local mic, and its recording pipeline is video-only
by construction.
"""

from __future__ import annotations

import logging
import re
import shutil
import subprocess
from pathlib import Path
from typing import Optional

logger = logging.getLogger(__name__)

# The BlueRobotics H264 USB Camera enumerates its ALSA name as
# "Camera" (from the descriptor "H264 USB Camera"). If a future
# camera model uses a different name, add it here.
KNOWN_CAMERA_ALSA_NAMES = ("Camera",)


class CaptureDevice:
    """Resolved capture device — pass to :func:`ensure_mic_unmuted` and
    build the GStreamer alsasrc `device=` string from :attr:`gst_device`.
    """

    def __init__(self, card_index: int, alsa_name: str, human_name: str):
        self.card_index = card_index
        self.alsa_name = alsa_name
        self.human_name = human_name

    @property
    def gst_device(self) -> str:
        """ALSA handle for ``alsasrc device=...`` — uses the by-name
        form which is stable across USB reconnects, unlike ``hw:<N>,0``.
        """
        return f"hw:{self.alsa_name},0"

    def __repr__(self) -> str:
        return (
            f"CaptureDevice(card={self.card_index}, "
            f"alsa_name={self.alsa_name!r}, human={self.human_name!r})"
        )


def find_capture_device() -> Optional[CaptureDevice]:
    """Walk ``/proc/asound/cards`` looking for the USB camera's ALSA card.
    Returns None if it isn't plugged in (or if /proc/asound isn't
    populated — e.g. running on a dev laptop with no sound stack).
    """
    cards_path = Path("/proc/asound/cards")
    if not cards_path.exists():
        logger.debug("audio: /proc/asound/cards missing; no ALSA")
        return None
    try:
        text = cards_path.read_text()
    except OSError as exc:
        logger.debug("audio: can't read %s: %s", cards_path, exc)
        return None

    # Format:
    #   0 [Headphones     ]: bcm2835_headphon - bcm2835 Headphones
    #                        bcm2835 Headphones
    #   1 [Camera         ]: USB-Audio - H264 USB Camera
    #                        HD USB Camera at usb-xhci-hcd.1-1.1, high speed
    # Each card is two lines; we anchor on the first line only.
    pat = re.compile(r"^\s*(\d+)\s+\[([^\]]+)\]:\s*(.+)$", re.MULTILINE)
    for m in pat.finditer(text):
        card_index = int(m.group(1))
        alsa_name = m.group(2).strip()
        human = m.group(3).strip()
        if any(known.lower() in alsa_name.lower() for known in KNOWN_CAMERA_ALSA_NAMES) or \
           any(known.lower() in human.lower() for known in KNOWN_CAMERA_ALSA_NAMES):
            logger.info(
                "audio: found capture device — card=%d alsa_name=%r human=%r",
                card_index, alsa_name, human,
            )
            return CaptureDevice(card_index, alsa_name, human)
    logger.info("audio: no USB camera capture card found in /proc/asound/cards")
    return None


def ensure_mic_unmuted(device: Optional[CaptureDevice] = None) -> bool:
    """Turn on the mic's capture switch. Safe to call repeatedly.

    Returns True if a device was found *and* amixer succeeded, False
    otherwise. Never raises — if the mic isn't detected or amixer is
    missing we just log and let the caller proceed (recording will still
    work, it'll just be silent).
    """
    if device is None:
        device = find_capture_device()
    if device is None:
        return False
    amixer = shutil.which("amixer")
    if amixer is None:
        logger.warning("audio: amixer not installed; can't unmute mic")
        return False
    # 'cap' enables the Capture Switch. Some USB cameras also expose a
    # separate 'Auto Gain Control' switch — we deliberately don't touch
    # it (leaving whatever the vendor default is).
    for control in ("Mic", "Capture"):
        try:
            result = subprocess.run(
                [amixer, "-c", str(device.card_index), "sset", control, "cap"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=3,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            logger.debug("audio: amixer sset %s cap failed: %s", control, exc)
            continue
        if result.returncode == 0:
            logger.info(
                "audio: unmuted %s on card %d (%s)",
                control, device.card_index, device.alsa_name,
            )
            return True
        # Missing control isn't an error — the camera just doesn't have that name.
        stderr = result.stderr.decode(errors="replace").strip()
        logger.debug(
            "audio: amixer -c %d sset %s cap => rc=%d, %s",
            device.card_index, control, result.returncode, stderr[:120],
        )
    logger.warning(
        "audio: no known capture control ('Mic' or 'Capture') on card %d — "
        "the mic may be permanently muted at the ALSA level",
        device.card_index,
    )
    return False


def build_alsa_source_snippet(
    device: CaptureDevice,
    *,
    rate_hz: int = 44100,
    channels: int = 1,
    bitrate_bps: int = 96000,
    mux_pad: str = "m.audio_0",
) -> str:
    """Return a GStreamer sub-pipeline that captures + encodes audio and
    plugs it into a named ``mp4mux`` pad. Combined with a video branch
    that plugs into ``m.video_0`` on the same ``mp4mux name=m …``
    element, this produces a fragmented A/V MP4.

    ``voaacenc`` (VisualOn AAC) is used instead of ``avenc_aac`` because
    it's more permissive about transient underruns from ALSA (which
    happen occasionally on USB cameras when the video URB queue backs
    up). Both encoders live in ``gstreamer1.0-plugins-bad`` /
    ``gstreamer1.0-libav`` which are already in the Dockerfile.

    ``do-timestamp=true`` on alsasrc makes GStreamer stamp each buffer
    with the pipeline running-time rather than the ALSA hardware clock,
    which keeps audio aligned with the RTP-timestamped video in mp4mux
    (the alternative bubbles through as ~200 ms of drift per hour).
    """
    return (
        f"alsasrc device={device.gst_device} do-timestamp=true "
        f"! audio/x-raw,format=S16LE,rate={rate_hz},channels={channels} "
        "! audioconvert ! audioresample "
        f"! voaacenc bitrate={bitrate_bps} ! aacparse "
        f"! queue ! {mux_pad}"
    )
