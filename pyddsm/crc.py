"""CRC-8/MAXIM as required by the DDSM210 protocol.

Polynomial x^8 + x^5 + x^4 + 1 (0x31), init 0x00, reflected in/out, xorout 0x00.
"""

from __future__ import annotations

_TABLE: list[int] = []


def _build_table() -> None:
    poly = 0x8C  # 0x31 bit-reflected
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = (crc >> 1) ^ poly if crc & 1 else crc >> 1
        _TABLE.append(crc)


_build_table()


def crc8_maxim(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _TABLE[(crc ^ b) & 0xFF]
    return crc
