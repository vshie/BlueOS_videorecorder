"""
Continuous low-resolution JPEG preview from the active camera source.

A single long-running ffmpeg child pipes MJPEG to stdout; a Python
reader thread splits it into individual JPEG frames and keeps the most
recent complete frame in a memory buffer.  Flask serves that buffer
atomically — there is no on-disk race window where a half-written
file could be returned to the browser.

The preview process is paused while a recording is active so it does
not contend with the recorder for the camera stream — both the USB
v4l2 device and the RTSP RadCam can only safely be read by one
ffmpeg/GStreamer instance at a time.

Usage:
    mgr = PreviewManager(video_device, rtsp_endpoint)
    mgr.start("usb")          # or "radcam"
    frame = mgr.get_latest_frame()    # bytes or None
    mgr.set_rotation(90)
    mgr.stop()                # before a recording starts
    mgr.start("usb")          # again after the recording stops
    mgr.shutdown()            # at app exit
"""

import logging
import subprocess
import threading
import time

logger = logging.getLogger(__name__)


# Defaults are chosen for a Pi 4 / 5 with USB camera or 4K RadCam over RTSP.
# The pipeline already has to decode whatever the source delivers, so the
# CPU cost scales with the input resolution far more than with the
# preview output resolution / fps.
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 5               # gentle on CPU; smooth enough for a preview
DEFAULT_JPEG_Q = 5            # ffmpeg q:v scale 1..31; 5 ≈ visually fine

_RESTART_BACKOFF_S = 2.0      # wait between auto-restart attempts
_READ_CHUNK_BYTES = 16 * 1024  # ffmpeg-stdout read size

_JPEG_SOI = b"\xff\xd8"   # start-of-image marker
_JPEG_EOI = b"\xff\xd9"   # end-of-image marker


def _rotation_filter(degrees):
    """Return an ffmpeg -vf filter fragment for the given rotation, or ''."""
    deg = int(degrees) % 360
    if deg == 0:
        return ""
    if deg == 90:
        return "transpose=1"
    if deg == 180:
        return "hflip,vflip"
    if deg == 270:
        return "transpose=2"
    return ""


class PreviewManager:
    def __init__(self, video_device, rtsp_endpoint,
                 width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT,
                 fps=DEFAULT_FPS, jpeg_quality=DEFAULT_JPEG_Q):
        self.video_device = video_device
        self.rtsp_endpoint = rtsp_endpoint
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality

        # DropCam now previews the BlueOS camera-manager RTSP stream rather
        # than the raw v4l2 device.  Set via set_blueos_endpoint(); when present
        # the "usb" mode builds an RTSP pipeline instead of a v4l2 one.
        self.blueos_endpoint = None

        self._lock = threading.RLock()
        self._proc = None
        self._mode = None            # "usb" | "radcam" | None
        self._rotation = 0
        self._enabled = False        # caller wants preview running
        self._stop = threading.Event()
        self._watcher_thread = None
        self._stderr_thread = None
        self._frame_thread = None

        # Atomic-swap buffer holding the most recent complete JPEG frame.
        # Mutated only by the frame-reader thread; readers grab a reference
        # under _frame_lock.  bytes objects are immutable so callers can
        # release the lock and still serve the frame safely.
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._latest_frame_time = 0.0

    # ── ffmpeg command builders ─────────────────────────────────────────

    def _build_filter(self):
        parts = [f"scale={self.width}:{self.height}:force_original_aspect_ratio=decrease"]
        rot = _rotation_filter(self._rotation)
        if rot:
            parts.append(rot)
        return ",".join(parts)

    def _build_cmd(self):
        # Output an MJPEG stream to stdout — the reader thread splits it
        # into discrete JPEGs.  We never touch disk, eliminating the
        # half-written-file race that produced flicker / colour artifacts
        # with the old image2 -update muxer.
        common_out = [
            "-vf", self._build_filter(),
            "-r", str(self.fps),
            "-q:v", str(self.jpeg_quality),
            "-f", "mjpeg",
            "pipe:1",
        ]
        if self._mode == "radcam":
            return [
                "ffmpeg", "-nostdin", "-loglevel", "error",
                "-rtsp_transport", "tcp",
                "-i", self.rtsp_endpoint,
                *common_out,
            ]
        # DropCam ("usb"): prefer the BlueOS camera-manager RTSP stream.
        if self.blueos_endpoint:
            return [
                "ffmpeg", "-nostdin", "-loglevel", "error",
                "-rtsp_transport", "tcp",
                "-i", self.blueos_endpoint,
                *common_out,
            ]
        # Legacy fallback: direct v4l2 capture (pre-BlueOS-managed camera).
        return [
            "ffmpeg", "-nostdin", "-loglevel", "error",
            "-f", "v4l2", "-input_format", "h264",
            "-video_size", "1920x1080",
            "-i", self.video_device,
            *common_out,
        ]

    # ── lifecycle ───────────────────────────────────────────────────────

    def start(self, mode):
        """Start (or restart) the preview pipeline.  mode = 'usb' or 'radcam'."""
        if mode not in ("usb", "radcam"):
            raise ValueError(f"invalid preview mode {mode!r}")
        with self._lock:
            self._mode = mode
            self._enabled = True
            self._stop.clear()
            self._kill_proc_locked()
            self._spawn_locked()
            self._ensure_watcher_locked()

    def set_rotation(self, degrees):
        """Update the preview rotation; restarts the pipeline if running."""
        deg = int(degrees) % 360
        with self._lock:
            if deg == self._rotation:
                return
            self._rotation = deg
            if self._enabled and self._mode is not None:
                self._kill_proc_locked()
                self._spawn_locked()

    def set_blueos_endpoint(self, url):
        """Set the BlueOS camera-manager RTSP URL used by the 'usb' mode.

        Restarts the preview if it is currently running the 'usb' source so the
        new endpoint takes effect immediately.
        """
        with self._lock:
            if url == self.blueos_endpoint:
                return
            self.blueos_endpoint = url
            if self._enabled and self._mode == "usb":
                self._kill_proc_locked()
                self._spawn_locked()

    def set_mode(self, mode):
        """Switch source (e.g. when RadCam is detected after boot)."""
        with self._lock:
            if mode == self._mode:
                return
            if self._enabled:
                self.start(mode)
            else:
                self._mode = mode

    def stop(self):
        """Pause the preview pipeline (e.g. before a recording starts).

        Synchronous: returns once ffmpeg has actually exited so the camera
        is free for the recorder to claim.
        """
        with self._lock:
            self._enabled = False
            self._kill_proc_locked()

    def shutdown(self):
        """Fully stop the preview and watcher thread."""
        with self._lock:
            self._enabled = False
            self._stop.set()
            self._kill_proc_locked()
        t = self._watcher_thread
        if t and t.is_alive():
            t.join(timeout=3)

    def is_running(self):
        with self._lock:
            return bool(self._proc and self._proc.poll() is None)

    def get_latest_frame(self):
        """Return (bytes, age_seconds) for the latest complete JPEG, or
        ``(None, None)`` if no frame has been received yet."""
        with self._frame_lock:
            frame = self._latest_frame
            ts = self._latest_frame_time
        if not frame:
            return None, None
        return frame, max(0.0, time.monotonic() - ts)

    # ── internals ───────────────────────────────────────────────────────

    def _spawn_locked(self):
        if self._proc and self._proc.poll() is None:
            return
        cmd = self._build_cmd()
        logger.info(
            f"Starting preview ({self._mode}, "
            f"{self.width}x{self.height}@{self.fps}fps, rot={self._rotation}°): "
            f"{' '.join(cmd)}"
        )
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                start_new_session=True,
                bufsize=0,
            )
        except Exception as e:
            logger.error(f"Failed to launch preview ffmpeg: {e}")
            self._proc = None
            return
        self._stderr_thread = threading.Thread(
            target=self._drain_stderr, args=(self._proc,),
            daemon=True, name="preview-stderr",
        )
        self._stderr_thread.start()
        self._frame_thread = threading.Thread(
            target=self._read_frames, args=(self._proc,),
            daemon=True, name="preview-frames",
        )
        self._frame_thread.start()

    def _read_frames(self, proc):
        """Parse MJPEG from ffmpeg stdout into the latest-frame buffer.

        ffmpeg -f mjpeg pipe:1 emits concatenated JPEG frames.  Each frame
        starts with the SOI marker (0xFFD8) and ends with EOI (0xFFD9).
        We accumulate stdout into a buffer and emit a frame to the
        in-memory slot every time we see a complete SOI..EOI pair.  Only
        whole frames ever land in self._latest_frame, so /snapshot will
        never serve a partial JPEG.
        """
        buf = bytearray()
        try:
            while True:
                chunk = proc.stdout.read(_READ_CHUNK_BYTES)
                if not chunk:
                    return
                buf.extend(chunk)
                while True:
                    soi = buf.find(_JPEG_SOI)
                    if soi < 0:
                        # No SOI in buffer.  Discard noise but keep the last
                        # byte in case it's the leading 0xFF of a marker that
                        # spans the next read.
                        if len(buf) > 1:
                            del buf[:-1]
                        break
                    eoi = buf.find(_JPEG_EOI, soi + 2)
                    if eoi < 0:
                        # Frame still arriving — drop bytes before SOI.
                        if soi > 0:
                            del buf[:soi]
                        break
                    frame = bytes(buf[soi:eoi + 2])
                    del buf[:eoi + 2]
                    with self._frame_lock:
                        self._latest_frame = frame
                        self._latest_frame_time = time.monotonic()
        except Exception as e:
            logger.debug(f"preview frame reader exiting: {e}")

    def _drain_stderr(self, proc):
        """Forward ffmpeg stderr lines into the logger so failures are visible."""
        try:
            for raw in iter(proc.stderr.readline, b""):
                line = raw.decode("utf-8", errors="replace").rstrip()
                if line:
                    logger.warning(f"preview ffmpeg: {line}")
        except Exception:
            pass

    def _kill_proc_locked(self):
        proc = self._proc
        self._proc = None
        if not proc:
            return
        if proc.poll() is not None:
            return
        try:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                logger.warning("Preview ffmpeg did not exit on SIGTERM, killing")
                proc.kill()
                proc.wait(timeout=2)
        except Exception as e:
            logger.warning(f"Preview kill error: {e}")
        logger.info("Preview pipeline stopped")

    def _ensure_watcher_locked(self):
        if self._watcher_thread and self._watcher_thread.is_alive():
            return
        self._watcher_thread = threading.Thread(
            target=self._watch, daemon=True, name="preview-watch",
        )
        self._watcher_thread.start()

    def _watch(self):
        """Restart the pipeline if it dies while preview is enabled."""
        while not self._stop.is_set():
            with self._lock:
                if (self._enabled and
                    (not self._proc or self._proc.poll() is not None)):
                    rc = self._proc.poll() if self._proc else None
                    self._proc = None
                    logger.warning(
                        f"Preview ffmpeg exited (rc={rc}); "
                        f"retrying in {_RESTART_BACKOFF_S}s"
                    )
                    needs_restart = True
                else:
                    needs_restart = False
            if needs_restart:
                if self._stop.wait(_RESTART_BACKOFF_S):
                    return
                with self._lock:
                    if self._enabled:
                        self._spawn_locked()
            else:
                self._stop.wait(1.0)
