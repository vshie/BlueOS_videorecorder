"""Daly A5 RS485 bus access for one or more board addresses.

Ported from brianhBR/Doris-Battery. We only use BoardDalyBMS here; the
DalyBus multi-board scanner is preserved verbatim in case a future
multi-pack setup needs it.
"""

from __future__ import annotations

import logging
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
    """DalyBMS that targets a specific board number on a shared RS485 bus."""

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

        if not self.serial.write(message_bytes):
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
