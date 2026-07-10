"""MyActuator Controller V3 / RMD Motion Protocol V4.2 CAN messages.

Wire format (single-motor):
  - Command TX arbitration ID: 0x140 + ID (1-32)
  - Reply RX arbitration ID:   0x240 + ID (1-32)
  - Special: CAN ID set/read uses fixed arbitration ID 0x300 (cmd 0x79)
  - DLC: always 8 bytes; DATA[0] = command byte
"""

from __future__ import annotations

import datetime
from typing import Dict, Literal, Optional, Type, TypeVar, cast

import can
from geometry import math_helpers

from drivers.can import enums, messages

MyActuatorCanMessageT = TypeVar("MyActuatorCanMessageT", bound="MyActuatorCanMessage")

# Single-motor command / reply ID ranges (IDs 0-32 → 0x140-0x160 / 0x240-0x260).
_CMD_ID_MIN = 0x140
_CMD_ID_MAX = 0x160
_REPLY_ID_MIN = 0x240
_REPLY_ID_MAX = 0x260
# Fixed arbitration IDs used by special commands.
_ARB_MULTI_MOTOR = 0x280
_ARB_CANID = 0x300


def _u16_le(data: bytes | bytearray | memoryview, offset: int) -> int:
    return int(data[offset]) | (int(data[offset + 1]) << 8)


def _i16_le(data: bytes | bytearray | memoryview, offset: int) -> int:
    value = _u16_le(data, offset)
    return value - 0x10000 if value > 0x7FFF else value


def _u32_le(data: bytes | bytearray | memoryview, offset: int) -> int:
    return (
        int(data[offset])
        | (int(data[offset + 1]) << 8)
        | (int(data[offset + 2]) << 16)
        | (int(data[offset + 3]) << 24)
    )


def _i32_le(data: bytes | bytearray | memoryview, offset: int) -> int:
    value = _u32_le(data, offset)
    return value - 0x100000000 if value > 0x7FFFFFFF else value


def _pack_u16_le(value: int) -> tuple[int, int]:
    value = int(value) & 0xFFFF
    return value & 0xFF, (value >> 8) & 0xFF


def _pack_i16_le(value: int) -> tuple[int, int]:
    value = max(-32768, min(32767, int(value)))
    if value < 0:
        value += 0x10000
    return value & 0xFF, (value >> 8) & 0xFF


def _pack_i32_le(value: int) -> tuple[int, int, int, int]:
    value = int(value)
    if value < 0:
        value += 0x100000000
    value &= 0xFFFFFFFF
    return (
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    )


def _pack_u32_le(value: int) -> tuple[int, int, int, int]:
    value = int(value) & 0xFFFFFFFF
    return (
        value & 0xFF,
        (value >> 8) & 0xFF,
        (value >> 16) & 0xFF,
        (value >> 24) & 0xFF,
    )


class MyActuatorCanMessage(messages.CanMessage):
    """Base class for MyActuator single-motor CAN messages."""

    cmd_id: int

    @property
    def arbitration_id(self) -> messages.MyActuatorArbitrationID:
        return cast(messages.MyActuatorArbitrationID, self._arbitration_id)

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        if not msg.data:
            return False
        arb = msg.arbitration_id
        if (
            _CMD_ID_MIN <= arb <= _CMD_ID_MAX
            or _REPLY_ID_MIN <= arb <= _REPLY_ID_MAX
            or arb in (_ARB_MULTI_MOTOR, _ARB_CANID)
        ):
            return msg.data[0] == cls.cmd_id
        return False

    @classmethod
    def from_can_message(
        cls: Type[MyActuatorCanMessageT], msg: can.Message
    ) -> MyActuatorCanMessageT:
        if not cls.matches(msg):
            raise ValueError(
                f"CAN message does not match the class desired {cls.__name__}!"
            )

        arb = msg.arbitration_id
        if _CMD_ID_MIN <= arb <= _CMD_ID_MAX:
            node_id = arb - _CMD_ID_MIN
        elif _REPLY_ID_MIN <= arb <= _REPLY_ID_MAX:
            node_id = arb - _REPLY_ID_MIN
        else:
            # Multi-motor (0x280) / CANID (0x300) — node id not in arbitration ID.
            node_id = 0

        message = cls(node_id=node_id)
        message._parse_can_msg_data(msg)
        return message

    def _gen_arbitration_id(self) -> messages.MyActuatorArbitrationID:
        return messages.MyActuatorArbitrationID(
            node_id=self.node_id, cmd_id=self.cmd_id
        )

    def _gen_can_msg_data(self) -> bytes:
        """Default: command byte followed by seven zeros. Override as needed."""
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """Default no-op for command-only messages without reply fields."""

    def _parse_motion_feedback(self, msg: can.Message) -> None:
        """Shared status-2 style feedback used by 0x9C and closed-loop replies."""
        self.temperature = int.from_bytes(bytes([msg.data[1]]), byteorder="little", signed=True)
        self.torque_current = _i16_le(msg.data, 2) * 0.01
        self.speed = _i16_le(msg.data, 4)
        self.angle = _i16_le(msg.data, 6)


########################################################################################
# PID PARAMETERS (0x30–0x32)
########################################################################################


class ReadPIDParametersMessage(MyActuatorCanMessage):
    """0x30 — Read current / speed / position loop KP/KI (uint8 each)."""

    cmd_id = 0x30

    current_kp: int
    current_ki: int
    speed_kp: int
    speed_ki: int
    position_kp: int
    position_ki: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.current_kp = msg.data[2]
        self.current_ki = msg.data[3]
        self.speed_kp = msg.data[4]
        self.speed_ki = msg.data[5]
        self.position_kp = msg.data[6]
        self.position_ki = msg.data[7]


class WritePIDParametersToRAMMessage(MyActuatorCanMessage):
    """0x31 — Write PID gains to RAM (lost on power-off)."""

    cmd_id = 0x31

    current_kp: int
    current_ki: int
    speed_kp: int
    speed_ki: int
    position_kp: int
    position_ki: int

    def _gen_can_msg_data(self) -> bytes:
        return bytes(
            [
                self.cmd_id,
                0,
                int(self.current_kp) & 0xFF,
                int(self.current_ki) & 0xFF,
                int(self.speed_kp) & 0xFF,
                int(self.speed_ki) & 0xFF,
                int(self.position_kp) & 0xFF,
                int(self.position_ki) & 0xFF,
            ]
        )

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        # Reply echoes the written gains (same layout as 0x30).
        self.current_kp = msg.data[2]
        self.current_ki = msg.data[3]
        self.speed_kp = msg.data[4]
        self.speed_ki = msg.data[5]
        self.position_kp = msg.data[6]
        self.position_ki = msg.data[7]


class WritePIDParametersToROMMessage(MyActuatorCanMessage):
    """0x32 — Write PID gains to ROM (persist across power-off)."""

    cmd_id = 0x32

    current_kp: int
    current_ki: int
    speed_kp: int
    speed_ki: int
    position_kp: int
    position_ki: int

    def _gen_can_msg_data(self) -> bytes:
        return bytes(
            [
                self.cmd_id,
                0,
                int(self.current_kp) & 0xFF,
                int(self.current_ki) & 0xFF,
                int(self.speed_kp) & 0xFF,
                int(self.speed_ki) & 0xFF,
                int(self.position_kp) & 0xFF,
                int(self.position_ki) & 0xFF,
            ]
        )

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.current_kp = msg.data[2]
        self.current_ki = msg.data[3]
        self.speed_kp = msg.data[4]
        self.speed_ki = msg.data[5]
        self.position_kp = msg.data[6]
        self.position_ki = msg.data[7]


# Suggested-agent aliases (same classes).
WritePIDToRAMMessage = WritePIDParametersToRAMMessage
WritePIDToROMMessage = WritePIDParametersToROMMessage


########################################################################################
# ACCELERATION (0x42–0x43)
########################################################################################


class ReadAccelerationCommand(MyActuatorCanMessage):
    """0x42 — Read acceleration parameter (dps²).

    Optional ``acceleration_type`` / ``index`` selects which profile value to read
    (position accel/decel, velocity accel/decel). Defaults to 0 (position accel).
    """

    cmd_id = 0x42

    # Request: profile index (same encoding as WriteAccelerationCommand).
    acceleration_type: enums.MyActuatorAccelerationType
    # Reply: acceleration value (dps²).
    acceleration_dps2: int
    # Alias used by suggested agent code.
    index: int
    accel_value: int

    def _gen_can_msg_data(self) -> bytes:
        index = 0
        if hasattr(self, "acceleration_type") and self.acceleration_type is not None:
            index = int(self.acceleration_type.value)
        elif hasattr(self, "index"):
            index = int(self.index)
        return bytes([self.cmd_id, index & 0xFF, 0, 0, 0, 0, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.index = msg.data[1]
        try:
            self.acceleration_type = enums.MyActuatorAccelerationType(msg.data[1])
        except ValueError:
            pass
        self.acceleration_dps2 = _i32_le(msg.data, 4)
        self.accel_value = self.acceleration_dps2


class WriteAccelerationCommand(MyActuatorCanMessage):
    """0x43 — Write acceleration/deceleration to RAM and ROM.

    Equivalent to myactuator_rmd ActuatorInterface.setAcceleration(value, mode).
    Value range is [100, 60000] dps² (1 dps²/LSB). Saved to ROM (persists).
    """

    cmd_id = 0x43

    # Profile type: position/velocity accel or decel.
    acceleration_type: enums.MyActuatorAccelerationType
    # Acceleration magnitude in dps² [100, 60000].
    acceleration_dps2: int

    def _gen_can_msg_data(self) -> bytes:
        # Support both acceleration_type enum and raw index/accel_value aliases.
        if hasattr(self, "acceleration_type") and self.acceleration_type is not None:
            mode = int(self.acceleration_type.value)
        else:
            mode = int(getattr(self, "index", 0))

        if hasattr(self, "acceleration_dps2"):
            accel = int(self.acceleration_dps2)
        else:
            accel = int(getattr(self, "accel_value", 0))
        # 0 is allowed by some firmwares to mean "default"; otherwise clamp.
        if accel != 0:
            accel = max(100, min(60000, accel))
        b0, b1, b2, b3 = _pack_u32_le(accel)
        return bytes([self.cmd_id, mode & 0xFF, 0, 0, b0, b1, b2, b3])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        try:
            self.acceleration_type = enums.MyActuatorAccelerationType(msg.data[1])
        except ValueError:
            pass
        self.acceleration_dps2 = _i32_le(msg.data, 4)


# Suggested-agent aliases.
ReadAccelerationMessage = ReadAccelerationCommand
WriteAccelerationMessage = WriteAccelerationCommand


########################################################################################
# ENCODER / ANGLE (0x60–0x64, 0x90, 0x92, 0x94)
########################################################################################


class ReadMultiTurnEncoderPositionMessage(MyActuatorCanMessage):
    """0x60 — Multi-turn encoder position (relative to zero offset), raw pulses."""

    cmd_id = 0x60
    position: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.position = _i32_le(msg.data, 4)


class ReadMultiTurnOriginalPositionMessage(MyActuatorCanMessage):
    """0x61 — Raw multi-turn encoder position (ignores zero offset)."""

    cmd_id = 0x61
    position: int
    # Alias for suggested agent naming.
    original_position: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.position = _i32_le(msg.data, 4)
        self.original_position = self.position


class ReadMultiTurnZeroOffsetMessage(MyActuatorCanMessage):
    """0x62 — Multi-turn encoder zero offset (pulses)."""

    cmd_id = 0x62
    offset: int
    position: int  # same value; alternate name used by rmd

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.offset = _i32_le(msg.data, 4)
        self.position = self.offset


class WriteEncoderMultiTurnValueToRomAsZeroMessage(MyActuatorCanMessage):
    """0x63 — Write a multi-turn encoder value to ROM as motor zero.

    ``encoder_offset`` is the int32 pulse value written as the new zero.
    Requires system reset (0x76) before the new zero takes effect.
    """

    cmd_id = 0x63
    encoder_offset: int

    def _gen_can_msg_data(self) -> bytes:
        offset = int(getattr(self, "encoder_offset", 0))
        b0, b1, b2, b3 = _pack_i32_le(offset)
        return bytes([self.cmd_id, 0, 0, 0, b0, b1, b2, b3])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.encoder_offset = _i32_le(msg.data, 4)


# Suggested-agent alias.
WriteEncoderMultiTurnToROMZeroMessage = WriteEncoderMultiTurnValueToRomAsZeroMessage


class WriteMotorZeroPositionMessage(MyActuatorCanMessage):
    """0x64 — Write the *current* multi-turn encoder position to ROM as zero.

    Note: After writing the new zero point, send 0x76 (system reset) to apply.
    The reply echoes the new encoder zero value (int32 pulses in DATA[4:8]).
    """

    cmd_id = 0x64
    encoder_zero: int

    def _gen_can_msg_data(self) -> bytes:
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.encoder_zero = _i32_le(msg.data, 4)


class ReadSingleTurnEncoderMessage(MyActuatorCanMessage):
    """0x90 — Single-turn encoder position, raw value, and offset (int16 each)."""

    cmd_id = 0x90
    position: int
    raw_position: int
    offset: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.position = _i16_le(msg.data, 2)
        self.raw_position = _i16_le(msg.data, 4)
        self.offset = _i16_le(msg.data, 6)


class ReadMultiTurnAngleMessage(MyActuatorCanMessage):
    """0x92 — Multi-turn absolute angle (degrees, 0.01°/LSB)."""

    cmd_id = 0x92
    angle: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.angle = _i32_le(msg.data, 4) * 0.01


class ReadSingleTurnAngleMessage(MyActuatorCanMessage):
    """0x94 — Single-turn angle (degrees, 0.01°/LSB, int16 at DATA[6:8])."""

    cmd_id = 0x94
    angle: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.angle = _i16_le(msg.data, 6) * 0.01


########################################################################################
# MOTOR STATUS (0x9A, 0x9C, 0x9D)
########################################################################################


class MyactuatorReadMotorStatus1Message(MyActuatorCanMessage):
    """0x9A — Motor temperature, brake state, bus voltage, and error flags."""

    cmd_id = 0x9A

    # Motor temperature (int8_t type, unit 1°C/LSB)
    temperature: int
    # Brake release command (1 = brake released, 0 = brake locked)
    brake_released: bool
    # Voltage (uint16_t type, unit 0.1V/LSB)
    voltage: float
    # Error flags (uint16_t type, bits represent different motor states)
    error_state: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.temperature = int.from_bytes(
            bytes([msg.data[1]]), byteorder="little", signed=True
        )
        self.brake_released = bool(msg.data[3])
        self.voltage = _u16_le(msg.data, 4) * 0.1
        self.error_state = _u16_le(msg.data, 6)


class ReadMotorStatus2Message(MyActuatorCanMessage):
    """0x9C — Temperature, torque current, speed, and shaft angle."""

    cmd_id = 0x9C

    temperature: int
    torque_current: float
    speed: int
    angle: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self._parse_motion_feedback(msg)


class ReadMotorStatus3Message(MyActuatorCanMessage):
    """0x9D — Temperature and three-phase currents (0.01 A/LSB)."""

    cmd_id = 0x9D

    temperature: int
    current_phase_a: float
    current_phase_b: float
    current_phase_c: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.temperature = int.from_bytes(
            bytes([msg.data[1]]), byteorder="little", signed=True
        )
        self.current_phase_a = _i16_le(msg.data, 2) * 0.01
        self.current_phase_b = _i16_le(msg.data, 4) * 0.01
        self.current_phase_c = _i16_le(msg.data, 6) * 0.01


########################################################################################
# MOTION CONTROL (0x80–0x81, 0xA1–0xA8)
########################################################################################


class MotorShutdownCommand(MyActuatorCanMessage):
    """0x80 — Turn off motor output and clear the running state."""

    cmd_id = 0x80


class MotorStopCommand(MyActuatorCanMessage):
    """0x81 — Stop the motor while maintaining closed-loop control mode."""

    cmd_id = 0x81


class TorqueControlCommand(MyActuatorCanMessage):
    """0xA1 — Torque / current closed-loop control.

    Request: ``torque_current`` in Amps (0.01 A/LSB on wire).
    Reply: status-2 style feedback (temperature, current, speed, angle).
    """

    cmd_id = 0xA1

    torque_current: float
    # Reply feedback fields (populated on parse).
    temperature: int
    speed: int
    angle: int

    def _gen_can_msg_data(self) -> bytes:
        torque_raw = int(round(float(self.torque_current) * 100))
        b0, b1 = _pack_i16_le(torque_raw)
        return bytes([self.cmd_id, 0, 0, 0, b0, b1, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self._parse_motion_feedback(msg)


class SpeedControlCommand(MyActuatorCanMessage):
    """0xA2 — Speed closed-loop control.

    Request: ``speed`` in deg/s (0.01 dps/LSB on wire).
    Reply: status-2 style feedback.
    """

    cmd_id = 0xA2

    speed: float
    # Reply feedback (speed field overwritten with measured shaft speed int).
    temperature: int
    torque_current: float
    angle: int

    def _gen_can_msg_data(self) -> bytes:
        speed_raw = int(round(float(self.speed) * 100))
        b0, b1, b2, b3 = _pack_i32_le(speed_raw)
        return bytes([self.cmd_id, 0, 0, 0, b0, b1, b2, b3])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        # Reply uses status-2 layout: int16 speed at 1 dps/LSB (not 0.01 dps).
        self.temperature = int.from_bytes(
            bytes([msg.data[1]]), byteorder="little", signed=True
        )
        self.torque_current = _i16_le(msg.data, 2) * 0.01
        self.speed = float(_i16_le(msg.data, 4))  # type: ignore[assignment]
        self.angle = _i16_le(msg.data, 6)


class PositionControlCommand(MyActuatorCanMessage):
    """0xA4 — Absolute multi-turn position closed-loop control.

    Request: ``position`` degrees (0.01°/LSB), ``max_speed`` dps (1 dps/LSB).
    Reply: status-2 style feedback.
    """

    cmd_id = 0xA4

    position: float
    max_speed: int
    # Reply feedback.
    temperature: int
    torque_current: float
    speed: int
    angle: int

    def _gen_can_msg_data(self) -> bytes:
        position_raw = int(round(float(self.position) * 100))
        s0, s1 = _pack_u16_le(int(self.max_speed))
        p0, p1, p2, p3 = _pack_i32_le(position_raw)
        return bytes([self.cmd_id, 0, s0, s1, p0, p1, p2, p3])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self._parse_motion_feedback(msg)


class SingleTurnPositionControlCommand(MyActuatorCanMessage):
    """0xA6 — Single-turn position closed-loop control.

    DATA[1] = spin direction, DATA[2:4] = max speed (uint16 dps),
    DATA[4:6] = angle (uint16, 0.01°/LSB), DATA[6:8] = 0.
    """

    cmd_id = 0xA6

    # 0 = default direction toward target (see PDF spinDirection table).
    spin_direction: int
    max_speed: int
    position: float  # single-turn degrees
    # Reply feedback.
    temperature: int
    torque_current: float
    speed: int
    angle: int

    def _gen_can_msg_data(self) -> bytes:
        spin = int(getattr(self, "spin_direction", 0)) & 0xFF
        s0, s1 = _pack_u16_le(int(self.max_speed))
        # Single-turn angle is uint16 0.01°/LSB on the wire.
        pos_raw = int(round(float(self.position) * 100)) & 0xFFFF
        p0, p1 = _pack_u16_le(pos_raw)
        return bytes([self.cmd_id, spin, s0, s1, p0, p1, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self._parse_motion_feedback(msg)


class IncrementalPositionControlCommand(MyActuatorCanMessage):
    """0xA8 — Incremental multi-turn position closed-loop control.

    Runs ``position_increment`` degrees from the current multi-turn position.
    """

    cmd_id = 0xA8

    max_speed: int
    position_increment: float
    # Reply feedback.
    temperature: int
    torque_current: float
    speed: int
    angle: int

    def _gen_can_msg_data(self) -> bytes:
        s0, s1 = _pack_u16_le(int(self.max_speed))
        position_raw = int(round(float(self.position_increment) * 100))
        p0, p1, p2, p3 = _pack_i32_le(position_raw)
        return bytes([self.cmd_id, 0, s0, s1, p0, p1, p2, p3])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self._parse_motion_feedback(msg)


########################################################################################
# SYSTEM / CONFIG (0x70–0x78, 0xB1–0xB6, 0x20, 0x79)
########################################################################################


class SystemOperatingModeAcquisitionCommand(MyActuatorCanMessage):
    """0x70 — Read the current operating / control mode."""

    cmd_id = 0x70
    operating_mode: enums.MyActuatorV3OperatingMode

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        operating_mode_byte = msg.data[7]
        self.operating_mode = enums.MyActuatorV3OperatingMode(operating_mode_byte)


class MotorPowerAcquisitionCommand(MyActuatorCanMessage):
    """0x71 — Read motor power (Watts, 0.1 W/LSB at DATA[6:8])."""

    cmd_id = 0x71
    power: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.power = _u16_le(msg.data, 6) * 0.1


class SystemResetCommand(MyActuatorCanMessage):
    """0x76 — Reset / reboot the motor controller."""

    cmd_id = 0x76


class SystemBrakeReleaseCommand(MyActuatorCanMessage):
    """0x77 — Release the holding brake."""

    cmd_id = 0x77


class SystemBrakeLockCommand(MyActuatorCanMessage):
    """0x78 — Lock the holding brake."""

    cmd_id = 0x78


class SystemRuntimeReadCommand(MyActuatorCanMessage):
    """0xB1 — System uptime in milliseconds (uint32 at DATA[4:8])."""

    cmd_id = 0xB1
    runtime_ms: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.runtime_ms = _u32_le(msg.data, 4)


class VersionAcquisitionCommand(MyActuatorCanMessage):
    """0xB2 — System software version date (YYYYMMDD as uint32)."""

    cmd_id = 0xB2
    version_date: int

    def version_datetime(self) -> datetime.datetime:
        """Parse version_date into a datetime.

        Common formats from firmware:
          - 8 digits: YYYYMMDD (e.g. 20220206)
          - 10 digits: YYYYMMDDNN build suffix (e.g. 2023041301 on X8)
        """
        s = str(self.version_date)
        if len(s) >= 8:
            return datetime.datetime.strptime(s[:8], "%Y%m%d")
        return datetime.datetime.strptime(s, "%Y%m%d")

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.version_date = _u32_le(msg.data, 4)


class CommunicationInterruptionProtectionCommand(MyActuatorCanMessage):
    """0xB3 — Communication interruption protection timeout (ms).

    0 disables the feature. After the timeout with no host frames, the motor
    stops for safety.
    """

    cmd_id = 0xB3
    timeout_ms: int

    def _gen_can_msg_data(self) -> bytes:
        b0, b1, b2, b3 = _pack_u32_le(int(self.timeout_ms))
        return bytes([self.cmd_id, 0, 0, 0, b0, b1, b2, b3])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.timeout_ms = _u32_le(msg.data, 4)


class CommunicationBaudRateSettingCommand(MyActuatorCanMessage):
    """0xB4 — Set CAN baud rate (takes effect after reset).

    Accepts either ``baud_rate`` (``MyActuatorCanBaudRate``) or ``baud_index`` int.
    """

    cmd_id = 0xB4
    baud_rate: enums.MyActuatorCanBaudRate
    baud_index: int

    def _gen_can_msg_data(self) -> bytes:
        if hasattr(self, "baud_rate") and self.baud_rate is not None:
            index = int(self.baud_rate.value)
        else:
            index = int(getattr(self, "baud_index", 1))
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, index & 0xFF])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.baud_index = msg.data[7]
        try:
            self.baud_rate = enums.MyActuatorCanBaudRate(msg.data[7])
        except ValueError:
            pass


class MotorModelReadingCommand(MyActuatorCanMessage):
    """0xB5 — Read motor model string (ASCII in DATA[1:8])."""

    cmd_id = 0xB5
    model: str

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.model = bytes(msg.data[1:8]).decode("ascii", errors="ignore").rstrip("\x00")


class ActiveReplyFunctionCommand(MyActuatorCanMessage):
    """0xB6 — Configure automatic/active reply for a target command.

    Layout (V4.2 Active Reply Function):
      DATA[1] = target command byte to auto-reply
      DATA[2] = enable (1) / disable (0)
      DATA[3:5] = interval in ms (uint16)
    """

    cmd_id = 0xB6
    target_cmd: int
    enable: bool
    interval_ms: int

    def _gen_can_msg_data(self) -> bytes:
        i0, i1 = _pack_u16_le(int(self.interval_ms))
        return bytes(
            [
                self.cmd_id,
                int(self.target_cmd) & 0xFF,
                1 if self.enable else 0,
                i0,
                i1,
                0,
                0,
                0,
            ]
        )

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.target_cmd = msg.data[1]
        self.enable = bool(msg.data[2])
        self.interval_ms = _u16_le(msg.data, 3)


class FunctionControlCommand(MyActuatorCanMessage):
    """0x20 — Function Control Command for controller configuration.

    See ``enums.MyActuatorFunctionControlIndex`` for function indices
    (clear multi-turn, CAN ID filter, angle limits, etc.).
    """

    cmd_id = 0x20

    function: enums.MyActuatorFunctionControlIndex
    function_value: int

    def _gen_can_msg_data(self) -> bytes:
        b0, b1, b2, b3 = _pack_i32_le(int(self.function_value))
        return bytes(
            [
                self.cmd_id,
                int(self.function.value) & 0xFF,
                0x00,
                0x00,
                b0,
                b1,
                b2,
                b3,
            ]
        )

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        try:
            self.function = enums.MyActuatorFunctionControlIndex(msg.data[1])
        except ValueError:
            pass
        self.function_value = _i32_le(msg.data, 4)


class CANIDCommand(MyActuatorCanMessage):
    """0x79 — Set/read CAN ID via fixed arbitration ID 0x300.

    Protocol:
      - Arbitration ID: 0x300 (not 0x140+ID)
      - DATA[2]: 0 = write, 1 = read
      - DATA[7]: CAN ID (1–32) for write
      - Read reply: DATA[6:8] = reply arbitration ID (0x240+ID) as uint16
    """

    cmd_id = 0x79

    read_write_flag: Literal["read", "write"]
    can_id: int
    # Parsed on read reply: full reply arbitration id (e.g. 0x241).
    reply_arbitration_id: int

    def _gen_arbitration_id(self) -> messages.MyActuatorArbitrationID:
        return messages.MyActuatorArbitrationID(
            node_id=self.node_id, cmd_id=self.cmd_id, custom_value=_ARB_CANID
        )

    def _gen_can_msg_data(self) -> bytes:
        read_write_flag_byte = 0x01 if self.read_write_flag == "read" else 0x00
        clipped_can_id = int(math_helpers.clip(int(self.can_id), 1, 32))
        return bytes([self.cmd_id, 0, read_write_flag_byte, 0, 0, 0, 0, clipped_can_id])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        # DATA[2] is the rw flag in the request; some firmwares echo it in DATA[1]
        # or DATA[2]. Prefer DATA[2] per protocol examples.
        flag_byte = msg.data[2] if len(msg.data) > 2 else msg.data[1]
        self.read_write_flag = "read" if bool(flag_byte) else "write"
        self.reply_arbitration_id = _u16_le(msg.data, 6)
        # Extract node id from reply arb id when present (0x240+ID).
        if _REPLY_ID_MIN <= self.reply_arbitration_id <= _REPLY_ID_MAX:
            self.can_id = self.reply_arbitration_id - _REPLY_ID_MIN
        else:
            self.can_id = msg.data[7]


########################################################################################
# Factory
########################################################################################


_CMD_CLASS_MAP: Dict[int, Type[MyActuatorCanMessage]] = {
    0x20: FunctionControlCommand,
    0x30: ReadPIDParametersMessage,
    0x31: WritePIDParametersToRAMMessage,
    0x32: WritePIDParametersToROMMessage,
    0x42: ReadAccelerationCommand,
    0x43: WriteAccelerationCommand,
    0x60: ReadMultiTurnEncoderPositionMessage,
    0x61: ReadMultiTurnOriginalPositionMessage,
    0x62: ReadMultiTurnZeroOffsetMessage,
    0x63: WriteEncoderMultiTurnValueToRomAsZeroMessage,
    0x64: WriteMotorZeroPositionMessage,
    0x70: SystemOperatingModeAcquisitionCommand,
    0x71: MotorPowerAcquisitionCommand,
    0x76: SystemResetCommand,
    0x77: SystemBrakeReleaseCommand,
    0x78: SystemBrakeLockCommand,
    0x79: CANIDCommand,
    0x80: MotorShutdownCommand,
    0x81: MotorStopCommand,
    0x90: ReadSingleTurnEncoderMessage,
    0x92: ReadMultiTurnAngleMessage,
    0x94: ReadSingleTurnAngleMessage,
    0x9A: MyactuatorReadMotorStatus1Message,
    0x9C: ReadMotorStatus2Message,
    0x9D: ReadMotorStatus3Message,
    0xA1: TorqueControlCommand,
    0xA2: SpeedControlCommand,
    0xA4: PositionControlCommand,
    0xA6: SingleTurnPositionControlCommand,
    0xA8: IncrementalPositionControlCommand,
    0xB1: SystemRuntimeReadCommand,
    0xB2: VersionAcquisitionCommand,
    0xB3: CommunicationInterruptionProtectionCommand,
    0xB4: CommunicationBaudRateSettingCommand,
    0xB5: MotorModelReadingCommand,
    0xB6: ActiveReplyFunctionCommand,
}


def create_myactuator_message(
    cmd_id: int, node_id: int = 1, **kwargs: object
) -> MyActuatorCanMessage:
    """Instantiate the message class for ``cmd_id`` (single-motor protocol)."""
    cls = _CMD_CLASS_MAP.get(cmd_id)
    if cls is None:
        raise ValueError(f"Unknown MyActuator command 0x{cmd_id:02X}")
    return cls(node_id=node_id, **kwargs)  # type: ignore[arg-type]


def message_class_for_cmd(cmd_id: int) -> Optional[Type[MyActuatorCanMessage]]:
    """Return the message class for a command byte, or None if unknown."""
    return _CMD_CLASS_MAP.get(cmd_id)

