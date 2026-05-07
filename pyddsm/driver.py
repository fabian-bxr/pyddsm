"""High-level DDSM210 driver.

Single motor (backward-compatible):
    with DDSM210("COM3") as m:
        m.set_mode(Mode.VELOCITY)
        m.set_velocity_rpm(50)

Multiple motors on one shared bus:
    with DDSMBus("COM3") as bus:
        left  = bus.motor(1)
        right = bus.motor(2)
        left.set_mode(Mode.VELOCITY)
        right.set_mode(Mode.VELOCITY)
        left.set_velocity_rpm(50)
        right.set_velocity_rpm(-50)

Heartbeat watchdog (stops all motors if the host process hangs or crashes):
    with DDSMBus("COM3", heartbeat=1.0) as bus:
        m = bus.motor(1)
        m.set_mode(Mode.VELOCITY)
        while True:
            m.set_velocity_rpm(50)   # each call resets the watchdog
            time.sleep(0.1)
    # On exit the watchdog thread is stopped cleanly.
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Union

import serial

from . import protocol
from .protocol import (
    BAUDRATE,
    DEFAULT_ID,
    FRAME_LEN,
    DriveReply,
    FeedbackReply,
    Mode,
)


class DDSM210Error(RuntimeError):
    pass


class DDSMBus:
    """Manages a shared bus for one or more DDSM210 motors.

    All commands from every motor on the bus are serialised through a single
    serial port.  A configurable inter-command gap is enforced so each motor has time to reply
    before the next frame is transmitted.

    Heartbeat watchdog
    ------------------
    Pass ``heartbeat`` (seconds) to enable an automatic safety stop.  A
    background thread monitors the time since the last command; if it exceeds
    ``heartbeat`` seconds, a stop command is sent to every motor registered on
    this bus.

    The watchdog is fed automatically by every normal drive command, so no
    extra application code is needed beyond setting the interval.  To
    temporarily suspend the watchdog (e.g. during a long blocking operation)
    call ``feed_watchdog()`` manually.
    """

    DEFAULT_GAP: float = 0.004  # seconds between frames on the bus

    def __init__(
        self,
        port: str,
        baudrate: int = BAUDRATE,
        timeout: float = 0.1,
        debug: bool = False,
        inter_cmd_gap: float = DEFAULT_GAP,
        heartbeat: Optional[float] = None,
    ) -> None:
        self.debug = debug
        self.inter_cmd_gap = inter_cmd_gap
        self._last_tx: float = 0.0
        self._motors: dict[int, DDSM210] = {}
        self._ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        # Keep handshake lines low so the RS-485 adapter doesn't glitch the bus.
        try:
            self._ser.setRTS(False)
            self._ser.setDTR(False)
        except (OSError, AttributeError):
            pass

        self._watchdog_interval: Optional[float] = heartbeat
        self._watchdog_thread: Optional[threading.Thread] = None
        self._watchdog_stop = threading.Event()
        self._last_feed: float = time.monotonic()
        if heartbeat is not None:
            self._start_watchdog()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        self._stop_watchdog()
        self._ser.close()

    def __enter__(self) -> DDSMBus:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Motor factory
    # ------------------------------------------------------------------

    def motor(self, motor_id: int) -> DDSM210:
        """Return a DDSM210 bound to this bus, creating it on first call."""
        if motor_id not in self._motors:
            self._motors[motor_id] = DDSM210(self, motor_id=motor_id)
        return self._motors[motor_id]

    # ------------------------------------------------------------------
    # Watchdog
    # ------------------------------------------------------------------

    def feed_watchdog(self) -> None:
        """Reset the watchdog timer without sending a command.

        Useful when the application is doing work that takes longer than the
        heartbeat interval but the motors should keep running.
        """
        self._last_feed = time.monotonic()

    def _start_watchdog(self) -> None:
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop, daemon=True, name="ddsm-watchdog"
        )
        self._watchdog_thread.start()

    def _stop_watchdog(self) -> None:
        self._watchdog_stop.set()
        if self._watchdog_thread is not None:
            self._watchdog_thread.join(timeout=2.0)
            self._watchdog_thread = None

    def _watchdog_loop(self) -> None:
        assert self._watchdog_interval is not None
        # Poll at half the heartbeat interval for responsiveness.
        poll = self._watchdog_interval / 2
        while not self._watchdog_stop.wait(timeout=poll):
            if time.monotonic() - self._last_feed > self._watchdog_interval:
                if self.debug:
                    print("WDT: heartbeat expired — stopping all motors")
                self._emergency_stop_all()

    def _emergency_stop_all(self) -> None:
        for motor_id in list(self._motors):
            try:
                frame = protocol.build_drive(motor_id, 0)
                self._txrx(frame, expect_reply=False)
            except Exception:
                pass  # best-effort; don't let one failure block the others

    # ------------------------------------------------------------------
    # Transport
    # ------------------------------------------------------------------

    def _txrx(self, frame: bytes, *, expect_reply: bool = True) -> Optional[bytes]:
        # Enforce inter-command gap.
        wait = self.inter_cmd_gap - (time.monotonic() - self._last_tx)
        if wait > 0:
            time.sleep(wait)

        self._ser.reset_input_buffer()
        self._ser.write(frame)
        self._ser.flush()
        self._last_tx = time.monotonic()
        self._last_feed = self._last_tx  # every TX feeds the watchdog

        if self.debug:
            print(f"TX: {frame.hex()}")

        if not expect_reply:
            return None

        reply = self._ser.read(FRAME_LEN)
        if self.debug:
            print(f"RX: {reply.hex()} ({len(reply)} bytes)")

        if len(reply) != FRAME_LEN:
            raise DDSM210Error(
                f"timeout: expected {FRAME_LEN} bytes, got {len(reply)} ({reply.hex()})"
            )
        if reply == b"\xff" * FRAME_LEN or reply == b"\x00" * FRAME_LEN:
            raise DDSM210Error(
                f"reply is all {reply[0]:#04x} bytes — line idle/floating. "
                "Check: motor powered (11-22 V), TX/RX not swapped, common GND, correct port."
            )
        if not protocol.check_crc(reply):
            raise DDSM210Error(f"bad CRC in reply: {reply.hex()}")
        return reply


class DDSM210:
    """Driver for a single DDSM210 motor.

    Can be used standalone (creates its own serial port) or attached to a
    shared DDSMBus for multi-motor setups.
    """

    def __init__(
        self,
        port: Union[str, DDSMBus] = "/dev/ttyUSB0",
        motor_id: int = DEFAULT_ID,
        timeout: float = 0.1,
        baudrate: int = BAUDRATE,
        debug: bool = False,
        heartbeat: Optional[float] = None,
    ) -> None:
        self.motor_id = motor_id
        if isinstance(port, DDSMBus):
            self._bus = port
            self._owns_bus = False
        else:
            self._bus = DDSMBus(
                port,
                baudrate=baudrate,
                timeout=timeout,
                debug=debug,
                heartbeat=heartbeat,
            )
            self._owns_bus = True

    @property
    def debug(self) -> bool:
        return self._bus.debug

    def close(self) -> None:
        """Close the underlying serial port (only when not sharing a bus)."""
        if self._owns_bus:
            self._bus.close()

    def __enter__(self) -> DDSM210:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _txrx(self, frame: bytes, *, expect_reply: bool = True) -> Optional[bytes]:
        return self._bus._txrx(frame, expect_reply=expect_reply)

    # ------------------------------------------------------------------
    # Mode
    # ------------------------------------------------------------------

    def set_mode(self, mode: Mode) -> Mode:
        reply = self._txrx(protocol.build_set_mode(self.motor_id, mode))
        assert reply is not None
        return protocol.parse_set_mode_reply(reply)

    def get_mode(self) -> Mode:
        reply = self._txrx(protocol.build_query_mode(self.motor_id))
        assert reply is not None
        return protocol.parse_mode_reply(reply)

    # ------------------------------------------------------------------
    # Feedback query (0x74)
    # ------------------------------------------------------------------

    def get_feedback(self) -> FeedbackReply:
        """Query mileage laps, absolute position (0..65535 = 0..360°), and error code."""
        reply = self._txrx(protocol.build_query_feedback(self.motor_id))
        assert reply is not None
        return protocol.parse_feedback_reply(reply)

    # ------------------------------------------------------------------
    # Drive commands
    # ------------------------------------------------------------------

    def drive(
        self,
        setpoint: int,
        *,
        feedback1: int = protocol.Feedback.SPEED,
        feedback2: int = protocol.Feedback.BUS_CURRENT,
        accel: int = 0,
        brake: bool = False,
    ) -> DriveReply:
        """Low-level drive command.  setpoint is raw (units depend on active mode).

        feedback1/feedback2 select what the motor echoes back:
          Feedback.SPEED (0x01), Feedback.BUS_CURRENT (0x02), Feedback.POSITION (0x03).
        """
        frame = protocol.build_drive(
            self.motor_id,
            setpoint,
            feedback1=feedback1,
            feedback2=feedback2,
            accel=accel,
            brake=brake,
        )
        reply = self._txrx(frame)
        assert reply is not None
        return protocol.parse_drive_reply(reply)

    def set_velocity_rpm(self, rpm: float, *, accel: int = 0) -> DriveReply:
        """Velocity-loop: target speed in RPM (-210..210).

        DriveReply.speed_rpm reflects actual speed; DriveReply.current_amps
        reflects bus current.
        """
        sp = max(-2100, min(2100, int(round(rpm * 10))))
        return self.drive(sp, accel=accel)

    def drive_open_loop(self, value: int) -> DriveReply:
        """Open-loop: raw PWM-like setpoint (-32767..32767).

        The motor must be in Mode.OPEN_LOOP.  Higher magnitude = more power.
        DriveReply.speed_rpm and DriveReply.current_amps are still returned.
        """
        sp = max(-32767, min(32767, int(value)))
        return self.drive(sp)

    def drive_to_position(self, setpoint: int, *, accel: int = 0) -> DriveReply:
        """Position-loop: move to setpoint (0..32767 = 0..360°).

        The motor must be in Mode.POSITION.  The motor takes the shortest path.
        DriveReply.feedback1 echoes the current position (same 0..32767 scale).
        DriveReply.current_amps reflects bus current.
        """
        sp = max(0, min(32767, int(setpoint)))
        return self.drive(
            sp,
            feedback1=protocol.Feedback.POSITION,
            feedback2=protocol.Feedback.BUS_CURRENT,
            accel=accel,
        )

    def drive_to_degrees(self, degrees: float, *, accel: int = 0) -> DriveReply:
        """Position-loop: move to target angle in degrees (0..360°).

        The motor must be in Mode.POSITION.
        """
        degrees = degrees % 360.0
        setpoint = int(round(degrees / 360.0 * 32767))
        return self.drive_to_position(setpoint, accel=accel)

    def stop(self) -> DriveReply:
        return self.drive(0)

    def brake(self) -> DriveReply:
        return self.drive(0, brake=True)

    # ------------------------------------------------------------------
    # ID management
    # ------------------------------------------------------------------

    def set_id(self, new_id: int) -> None:
        """Permanently set the motor ID (saved on power-off).

        Only one motor must be on the bus.  The command is sent 5 consecutive
        times as required by the DDSM210 protocol.  Can only be changed once
        per power cycle.
        """
        frame = protocol.build_set_id(new_id)
        for _ in range(5):
            self._bus._ser.reset_input_buffer()
            self._bus._ser.write(frame)
            self._bus._ser.flush()
            time.sleep(0.05)
        # Discard any reply bytes accumulated during the blind writes.
        self._bus._ser.reset_input_buffer()
        self.motor_id = new_id

    def query_id(self) -> int:
        """Query the ID of the single motor on the bus (broadcast command)."""
        reply = self._txrx(protocol.build_query_id())
        assert reply is not None
        return reply[0]
