"""DDSM210 UART protocol: 10-byte frames, CRC-8/MAXIM in the last byte."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

from .crc import crc8_maxim

FRAME_LEN = 10
DEFAULT_ID = 0x01
BAUDRATE = 115200


class Mode(IntEnum):
    OPEN_LOOP = 0x00
    VELOCITY = 0x02
    POSITION = 0x03


class Feedback(IntEnum):
    SPEED = 0x01
    BUS_CURRENT = 0x02
    POSITION = 0x03


def _frame(payload: bytes) -> bytes:
    if len(payload) != FRAME_LEN - 1:
        raise ValueError(f"payload must be {FRAME_LEN - 1} bytes, got {len(payload)}")
    return payload + bytes([crc8_maxim(payload)])


def check_crc(frame: bytes) -> bool:
    return len(frame) == FRAME_LEN and crc8_maxim(frame[:-1]) == frame[-1]


def build_drive(
    motor_id: int,
    setpoint: int,
    *,
    feedback1: int = 0,
    feedback2: int = 0,
    accel: int = 0,
    brake: bool = False,
) -> bytes:
    """Velocity-loop: setpoint in 0.1 rpm, range -2100..2100 (signed 16-bit).
    Position-loop: setpoint 0..32767 (unsigned 16-bit, 0..360°).
    Open-loop:     setpoint -32767..32767 (signed 16-bit, raw PWM).

    feedback1/feedback2 tell the motor what to echo back in the reply:
      Feedback.SPEED (0x01), Feedback.BUS_CURRENT (0x02), Feedback.POSITION (0x03).
    """
    sp = int(setpoint) & 0xFFFF
    payload = bytes([
        motor_id & 0xFF,
        0x64,
        (sp >> 8) & 0xFF,
        sp & 0xFF,
        feedback1 & 0xFF,
        feedback2 & 0xFF,
        accel & 0xFF,
        0xFF if brake else 0x00,
        0x00,
    ])
    return _frame(payload)


def build_set_mode(motor_id: int, mode: Mode) -> bytes:
    payload = bytes([motor_id & 0xFF, 0xA0, int(mode), 0, 0, 0, 0, 0, 0])
    return _frame(payload)


def build_query_mode(motor_id: int) -> bytes:
    payload = bytes([motor_id & 0xFF, 0x75, 0, 0, 0, 0, 0, 0, 0])
    return _frame(payload)


def build_query_feedback(motor_id: int) -> bytes:
    payload = bytes([motor_id & 0xFF, 0x74, 0, 0, 0, 0, 0, 0, 0])
    return _frame(payload)


def build_query_id() -> bytes:
    payload = bytes([0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0])
    return _frame(payload)


def build_set_id(motor_id: int) -> bytes:
    payload = bytes([0xAA, 0x55, 0x53, motor_id & 0xFF, 0, 0, 0, 0, 0])
    return _frame(payload)


@dataclass
class DriveReply:
    motor_id: int
    feedback1: int  # signed 16-bit; interpretation depends on requested feedback type
    feedback2: int  # signed 16-bit; interpretation depends on requested feedback type
    accel: int
    temperature: int  # °C, signed 8-bit
    error_code: int

    @property
    def speed_rpm(self) -> float:
        """Actual speed in RPM. Valid when feedback1 slot = Feedback.SPEED (default)."""
        return self.feedback1 / 10.0

    @property
    def current_amps(self) -> float:
        """Bus current in amps (-8..8 A). Valid when feedback2 slot = Feedback.BUS_CURRENT (default)."""
        return self.feedback2 * 8.0 / 32767.0

    @property
    def overcurrent(self) -> bool:
        return bool(self.error_code & 0x02)

    @property
    def overtemperature(self) -> bool:
        return bool(self.error_code & 0x10)

    @property
    def fault_bit6(self) -> bool:
        """Bit 6 (0x40): undocumented in the Waveshare wiki. Observed when
        supply voltage is below the 11 V minimum; likely an undervoltage flag."""
        return bool(self.error_code & 0x40)


@dataclass
class FeedbackReply:
    motor_id: int
    mileage_laps: int  # signed 32-bit; resets to 0 on power-on
    position: int      # 0..65535 → 0..360°
    error_code: int

    @property
    def position_degrees(self) -> float:
        """Current shaft position in degrees (0..360°)."""
        return self.position * 360.0 / 65535.0

    @property
    def overcurrent(self) -> bool:
        return bool(self.error_code & 0x02)

    @property
    def overtemperature(self) -> bool:
        return bool(self.error_code & 0x10)


def parse_drive_reply(frame: bytes) -> DriveReply:
    _validate(frame, 0x64)
    fb1 = _i16(frame[2], frame[3])
    fb2 = _i16(frame[4], frame[5])
    return DriveReply(
        motor_id=frame[0],
        feedback1=fb1,
        feedback2=fb2,
        accel=frame[6],
        temperature=_i8(frame[7]),
        error_code=frame[8],
    )


def parse_feedback_reply(frame: bytes) -> FeedbackReply:
    _validate(frame, 0x74)
    mileage = int.from_bytes(frame[2:6], "big", signed=True)
    position = (frame[6] << 8) | frame[7]
    return FeedbackReply(
        motor_id=frame[0],
        mileage_laps=mileage,
        position=position,
        error_code=frame[8],
    )


def parse_mode_reply(frame: bytes) -> Mode:
    _validate(frame, 0x75)
    return Mode(frame[2])


def parse_set_mode_reply(frame: bytes) -> Mode:
    _validate(frame, 0xA0)
    return Mode(frame[2])


def _validate(frame: bytes, expected_cmd: int) -> None:
    if len(frame) != FRAME_LEN:
        raise ValueError(f"expected {FRAME_LEN}-byte frame, got {len(frame)}")
    if not check_crc(frame):
        raise ValueError(f"bad CRC on frame {frame.hex()}")
    if frame[1] != expected_cmd:
        raise ValueError(f"expected cmd {expected_cmd:#04x}, got {frame[1]:#04x}")


def _i16(hi: int, lo: int) -> int:
    v = (hi << 8) | lo
    return v - 0x10000 if v & 0x8000 else v


def _i8(b: int) -> int:
    return b - 0x100 if b & 0x80 else b
