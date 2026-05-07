# pyddsm

Python driver for the [Waveshare DDSM210](https://www.waveshare.com/wiki/DDSM210) direct-drive servo motor over UART.

## Installation

```bash
pip install git+https://github.com/fabian-bxr/pyddsm.git

# using uv:
uv add git+https://github.com/fabian-bxr/pyddsm.git
```

## Requirements

- Python 3.11+
- DDSM210 motor connected via a USB–UART adapter (115200 8N1)
- Supply voltage 11–22 V

## Quick start

```python
from pyddsm import DDSM210, Mode

with DDSM210("COM3") as m:          # Linux: "/dev/ttyUSB0"
    m.set_mode(Mode.VELOCITY)
    m.set_velocity_rpm(50)
```

## Modes

The DDSM210 supports three operating modes selected with `set_mode()`.

| Mode | Constant | Setpoint range |
|------|----------|---------------|
| Open-loop | `Mode.OPEN_LOOP` | −32767 .. 32767 (raw PWM) |
| Velocity loop | `Mode.VELOCITY` | −210 .. 210 RPM |
| Position loop | `Mode.POSITION` | 0 .. 360° |

## API

### Single motor

```python
from pyddsm import DDSM210, Mode

with DDSM210(port="COM3", motor_id=1) as m:
    m.set_mode(Mode.VELOCITY)

    # Velocity loop
    reply = m.set_velocity_rpm(50)
    print(reply.speed_rpm, reply.current_amps, reply.temperature)

    # Open loop
    m.set_mode(Mode.OPEN_LOOP)
    m.drive_open_loop(8000)

    # Position loop
    m.set_mode(Mode.POSITION)
    m.drive_to_degrees(90.0)
    m.drive_to_degrees(270.0)

    m.stop()
    m.brake()
```

### Multiple motors on one bus

```python
from pyddsm import DDSMBus, Mode

with DDSMBus("COM3") as bus:
    left  = bus.motor(1)
    right = bus.motor(2)

    left.set_mode(Mode.VELOCITY)
    right.set_mode(Mode.VELOCITY)

    while True:
        left.set_velocity_rpm(50)
        right.set_velocity_rpm(-50)
```

`DDSMBus` serialises all commands through a single serial port and enforces the 4 ms inter-command gap required by the DDSM210 protocol.

### Heartbeat watchdog

Pass `heartbeat` (seconds) to automatically stop all motors if the host process hangs or crashes.  Every drive command resets the timer; no extra code is needed.

```python
with DDSMBus("COM3", heartbeat=1.0) as bus:
    m = bus.motor(1)
    m.set_mode(Mode.VELOCITY)
    while True:
        m.set_velocity_rpm(50)   # resets the watchdog
        time.sleep(0.1)
```

To pause commands without triggering a stop:

```python
bus.feed_watchdog()
```

### Drive reply

All drive methods return a `DriveReply`:

| Attribute | Type | Description |
|-----------|------|-------------|
| `speed_rpm` | `float` | Actual speed in RPM (when `feedback1=Feedback.SPEED`) |
| `current_amps` | `float` | Bus current in amps, −8 .. 8 A (when `feedback2=Feedback.BUS_CURRENT`) |
| `temperature` | `int` | Controller temperature in °C |
| `error_code` | `int` | Raw error byte |
| `overcurrent` | `bool` | Bit 1 of error code |
| `overtemperature` | `bool` | Bit 4 of error code |
| `fault_bit6` | `bool` | Bit 6 — observed on undervoltage (undocumented) |

### Feedback query

```python
fb = m.get_feedback()
print(fb.mileage_laps)       # cumulative full rotations since power-on
print(fb.position_degrees)   # absolute shaft angle, 0..360°
```

### ID management

```python
# One motor on the bus only. Saved on power-off, once per power cycle.
with DDSM210("COM3") as m:
    m.set_id(2)
```

## License

MIT
