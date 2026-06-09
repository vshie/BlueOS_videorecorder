"""Filter Daly BMS fault status to stage-2 (alarm) conditions only.

Ported verbatim from brianhBR/Doris-Battery.
"""

from __future__ import annotations

from dalybms.error_codes import ERROR_CODES

IGNORED_ERRORS = frozenset({"RESERVED"})


def is_stage2_fault(byte_index: int, bit_index: int, message: str) -> bool:
    """True for Daly stage-2 alarms; stage-1 entries are treated as warnings."""
    if message in IGNORED_ERRORS:
        return False

    lower = message.lower()
    if "warning" in lower:
        return False
    if any(token in lower for token in ("level two", "two stage", "alarm two")):
        return True
    if any(token in lower for token in ("level one", "one stage", "one alarm")):
        return False

    if byte_index <= 3:
        return bit_index % 2 == 1

    if byte_index == 4:
        return bit_index % 2 == 1

    return byte_index in (5, 6)


def parse_stage2_errors(response_data: bytes) -> list[str]:
    if not response_data or int.from_bytes(response_data, byteorder="big") == 0:
        return []

    errors: list[str] = []
    for byte_index, value in enumerate(response_data):
        if value == 0:
            continue
        bits = bin(value)[2:].zfill(8)
        for bit_index, bit in enumerate(reversed(bits)):
            if bit != "1":
                continue
            if byte_index >= len(ERROR_CODES):
                continue
            codes = ERROR_CODES[byte_index]
            if bit_index >= len(codes):
                continue
            message = codes[bit_index]
            if is_stage2_fault(byte_index, bit_index, message):
                errors.append(message)
    return errors


def filter_stage2_errors(errors: list[str] | None) -> list[str]:
    """Drop stage-1 warning strings when only the message text is available."""
    if not errors:
        return []
    cleaned: list[str] = []
    seen: set[str] = set()
    for item in errors:
        text = str(item).strip()
        if not text or text in IGNORED_ERRORS or text in seen:
            continue
        lower = text.lower()
        if "warning" in lower:
            continue
        if any(token in lower for token in ("level one", "one stage", "one alarm")):
            continue
        if not any(
            token in lower
            for token in ("level two", "two stage", "alarm two", "failure", "malfunction", "fault", "drop off")
        ):
            continue
        seen.add(text)
        cleaned.append(text)
    return cleaned
