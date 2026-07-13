"""Daly A5 RS485 bus access for one or more board addresses.

Ported from brianhBR/Doris-Battery. We only use BoardDalyBMS here; the
DalyBus multi-board scanner is preserved verbatim in case a future
multi-pack setup needs it.
"""

from __future__ import annotations

import logging
import struct
import time
from dataclasses import dataclass
from typing import Any

import serial

DEFAULT_BAUD_RATE = 9600
DEFAULT_SERIAL_TIMEOUT = 0.5
DEFAULT_REQUEST_RETRIES = 3
from dalybms import DalyBMS

from doris_battery.bms_errors import parse_stage2_errors

logger = logging.getLogger(__name__)


def extract_broadcast_frames(buffer: bytearray) -> list[tuple[int, int, bytes]]:
    """Pull complete, checksum-valid A5 frames out of a raw byte buffer.

    Daly packs in auto-report mode emit unsolicited frames; this resynchronises
    on each frame start and returns ``(source, command, payload)`` tuples.
    Handles both UART-style frames (leading ``0xA5``, 13 bytes) and RS485-style
    frames (leading ``0x00 0xA5``, 14 bytes). Consumed bytes are removed from
    ``buffer``; a trailing partial frame is kept for the next read.
    """
    frames: list[tuple[int, int, bytes]] = []
    i = 0
    consumed = 0
    n = len(buffer)
    while i < n:
        b = buffer[i]
        if b == 0x00 and i + 1 < n and buffer[i + 1] == 0xA5:
            if i + 14 > n:
                break
            frame = buffer[i : i + 14]
            if frame[4] == 0x08 and (sum(frame[:13]) & 0xFF) == frame[13]:
                frames.append((frame[2], frame[3], bytes(frame[5:13])))
                i += 14
                consumed = i
                continue
            i += 1
            continue
        if b == 0xA5:
            if i + 13 > n:
                break
            frame = buffer[i : i + 13]
            if frame[3] == 0x08 and (sum(frame[:12]) & 0xFF) == frame[12]:
                frames.append((frame[1], frame[2], bytes(frame[4:12])))
                i += 13
                consumed = i
                continue
            i += 1
            continue
        i += 1
    if consumed:
        del buffer[:consumed]
    return frames


def parse_soc_payload(payload: bytes) -> dict[str, float] | None:
    """Decode an 0x90 (SOC/voltage/current) data payload."""
    if len(payload) != 8:
        return None
    try:
        total_voltage, _x, current, soc = struct.unpack(">hhhh", payload)
    except struct.error:
        return None
    return {
        "total_voltage": total_voltage / 10,
        "current": (current - 30000) / 10,  # negative = charging
        "soc_percent": soc / 10,
    }


def board_to_host(board_number: int) -> int:
    """Map BmsMonitor BoardNo (1-16) to A5 read host byte (0x40-0x4f)."""
    if not 1 <= board_number <= 16:
        raise ValueError("board_number must be 1-16")
    return 0x40 + board_number - 1


def board_to_write_host(board_number: int) -> int:
    """Map BoardNo to A5 write host byte (0x80-0x8f) for MOSFET / parameter commands."""
    if not 1 <= board_number <= 16:
        raise ValueError("board_number must be 1-16")
    return 0x80 + board_number - 1


@dataclass
class PackInfo:
    board: int
    host: int
    response_id: int
    total_voltage_v: float
    soc_percent: float


class BoardDalyBMS(DalyBMS):
    """DalyBMS that targets a specific board number on a shared RS485 bus.

    Supports half-duplex RS485 transceivers (SN65HVD75 on the DeckHand PCB)
    via ``set_direction_control()``: a caller supplies a ``toggle(transmit)``
    callback that drives the transceiver's DE/~RE pin, and this class flips
    it HIGH before each write, waits for the frame to drain out of the UART
    FIFO, then flips it back LOW to receive.
    """

    def __init__(
        self,
        board_number: int,
        request_retries: int = 2,
        baudrate: int = DEFAULT_BAUD_RATE,
        serial_timeout: float = DEFAULT_SERIAL_TIMEOUT,
    ) -> None:
        super().__init__(request_retries=request_retries, address=4, logger=logger)
        self.board_number = board_number
        self.host_address = board_to_host(board_number)
        self.baudrate = baudrate
        self.serial_timeout = serial_timeout
        # Half-duplex direction control (populated by set_direction_control()).
        self._de_toggle: Any = None
        self._de_release_delay_s: float = 0.001

    def set_direction_control(
        self,
        toggle: Any,
        release_delay_s: float = 0.001,
    ) -> None:
        """Register a driver-enable callback for half-duplex RS485.

        ``toggle(transmit: bool)`` is called with ``True`` immediately
        before ``serial.write()`` and ``False`` after the frame has been
        flushed to the wire and the extra release delay has elapsed.
        ``release_delay_s`` covers the pyserial ``flush()`` + UART shift-
        register drain time — 15 A5 bytes at 9600 baud take ~16 ms end to
        end, and ``flush()`` blocks until the last byte is on the wire, so
        a small extra pad (~1 ms) is enough to avoid clipping the trailing
        CRC. Pass ``None`` to disable direction control (auto-direction
        transceivers, USB adapters).
        """
        self._de_toggle = toggle
        self._de_release_delay_s = max(0.0, float(release_delay_s))

    def _write_frame(self, message_bytes: bytes) -> int:
        """Send a frame with half-duplex DE handling if configured.

        Returns the number of bytes written (same contract as
        pyserial.Serial.write). When a DE toggle is registered, drives
        the transceiver into transmit before the write, waits for
        ``flush()`` + the release delay to guarantee the last stop bit
        has left the shift register, then hands the bus back to the
        receiver.
        """
        toggle = self._de_toggle
        if toggle is None:
            return self.serial.write(message_bytes)
        toggle(True)
        try:
            written = self.serial.write(message_bytes)
            try:
                self.serial.flush()
            except Exception:
                pass
            if self._de_release_delay_s > 0:
                time.sleep(self._de_release_delay_s)
        finally:
            toggle(False)
        return written

    def connect(self, device: str) -> None:
        self.serial = serial.Serial(
            port=device,
            baudrate=self.baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self.serial_timeout,
            xonxoff=False,
            write_timeout=self.serial_timeout,
        )

    def disconnect(self) -> None:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        except Exception:
            pass

    def _format_message(self, command: str, extra: str = "") -> bytes:
        message = f"a5{self.host_address:02x}{command}08{extra}"
        message = message.ljust(24, "0")
        message_bytes = bytearray.fromhex(message)
        message_bytes += self._calc_crc(message_bytes)
        return message_bytes

    @staticmethod
    def _parse_response_frame(b: bytes, command_byte: int) -> bytes | None:
        """Extract the 8-byte data payload from an A5 UART or RS485 response frame."""
        if len(b) < 4:
            return None
        if b[0] == 0x00 and b[1] == 0xA5:
            if len(b) < 14 or b[3] != command_byte:
                return None
            payload = b[5:13]
            expected_crc = sum(b[:13]) & 0xFF
            crc = b[13]
        elif b[0] == 0xA5:
            if len(b) < 13 or b[2] != command_byte:
                return None
            payload = b[4:12]
            expected_crc = sum(b[:12]) & 0xFF
            crc = b[12]
        else:
            return None
        if expected_crc != crc:
            logger.debug("response crc mismatch: %02x != %02x", expected_crc, crc)
        return payload

    def _read(self, command, extra="", max_responses=1, return_list=False, write: bool = False):
        """Parse RS485 A5 frames (leading 0x00, 15-byte wire length)."""
        self.logger.debug("-- %s %s------------------------", command, "write " if write else "")
        if not self.serial.is_open:
            self.serial.open()
        saved_host = self.host_address
        if write:
            self.host_address = board_to_write_host(self.board_number)
        try:
            message_bytes = self._format_message(command, extra=extra)
        finally:
            self.host_address = saved_host

        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

        if not self._write_frame(message_bytes):
            self.logger.error("serial write failed for command %s", command)
            return False
        time.sleep(0.15 if write else 0.05)

        command_byte = int(command, 16)
        response_data = []
        for x in range(max_responses):
            b = self.serial.read(14 if x else 15)
            if len(b) == 0:
                self.logger.debug("%i empty response for command %s", x, command)
                break
            self.logger.debug("%i %s %s", x, b.hex(), len(b))
            payload = self._parse_response_frame(b, command_byte)
            if payload is None:
                self.logger.debug("invalid response frame for command %s", command)
                continue
            response_data.append(payload)
            if len(response_data) == max_responses:
                break

        if return_list or len(response_data) > 1:
            return response_data
        if len(response_data) == 1:
            return response_data[0]
        return False

    def _read_request(self, command, extra="", max_responses=1, return_list=False, write: bool = False):
        response_data = None
        tries = 0
        for tries in range(self.request_retries):
            response_data = self._read(
                command=command,
                extra=extra,
                max_responses=max_responses,
                return_list=return_list,
                write=write,
            )
            if response_data:
                break
            time.sleep(0.2)
        if not response_data:
            self.logger.debug("%s failed after %s tries", command, tries + 1)
            return False
        return response_data

    def get_errors(self, response_data=None):
        if not response_data:
            response_data = self._read_request("98")
        if not response_data:
            return []
        return parse_stage2_errors(response_data)

    def read_soc_broadcast(
        self,
        duration: float = 3.0,
        solicit: bool = True,
    ) -> dict[str, float] | None:
        """Capture a Daly SOC frame (command 0x90) over a short listen window.

        Works in both BMS regimes for packs that don't reliably answer full
        polled snapshots:

        * Auto-report: the pack streams unsolicited 0x90 frames, which we read.
        * Polled: when ``solicit`` is True we periodically send a 0x90 request
          for this board and read the reply.

        Returns the latest SOC dict (``total_voltage``/``current``/
        ``soc_percent``) seen during the window, or ``None`` if nothing valid
        arrived. Our own outgoing request frames (host addresses 0x40-0x8f)
        are ignored so they can't be mistaken for data.
        """
        if self.serial is None:
            raise RuntimeError("serial not connected")
        if not self.serial.is_open:
            self.serial.open()
        try:
            self.serial.reset_input_buffer()
        except Exception:
            pass

        latest: dict[str, float] | None = None
        buffer = bytearray()
        deadline = time.time() + max(duration, 0.2)
        next_solicit = 0.0
        while time.time() < deadline:
            now = time.time()
            if solicit and now >= next_solicit:
                try:
                    self._write_frame(self._format_message("90"))
                except Exception as exc:
                    self.logger.debug("solicit 0x90 failed: %s", exc)
                next_solicit = now + 0.7
            try:
                chunk = self.serial.read(64)
            except serial.SerialException as exc:
                # FTDI adapters can spuriously report readiness with no data.
                self.logger.debug("serial read hiccup, retrying: %s", exc)
                try:
                    self.serial.reset_input_buffer()
                except Exception:
                    pass
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            buffer.extend(chunk)
            for src, command, payload in extract_broadcast_frames(buffer):
                if command != 0x90 or 0x40 <= src <= 0x8F:
                    continue
                soc = parse_soc_payload(payload)
                if soc:
                    latest = soc
        return latest
