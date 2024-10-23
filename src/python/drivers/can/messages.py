from __future__ import annotations

import struct
from typing import Any, Optional, Type, TypeVar

import can
import pydantic
from odrive import enums as odrive_enums  # type: ignore[import-untyped]

OdriveCanMessageT = TypeVar("OdriveCanMessageT", bound="OdriveCanMessage")


class _ArbitrationID(pydantic.BaseModel):
    node_id: int
    cmd_id: int

    @classmethod
    def from_can_message(cls, msg: can.Message) -> _ArbitrationID:
        return cls(
            node_id=(msg.arbitration_id >> 5), cmd_id=(msg.arbitration_id & 0b11111)
        )

    @property
    def value(self) -> int:
        return self.node_id << 5 | self.cmd_id


class OdriveCanMessage:
    cmd_id: int

    def __init__(self, node_id: int, **kwargs: Any):
        self._node_id = node_id
        self._arbitration_id = _ArbitrationID(node_id=node_id, cmd_id=self.cmd_id)

        for arg_name, value in kwargs.items():
            setattr(self, arg_name, value)

    @property
    def arbitration_id(self) -> _ArbitrationID:
        return self._arbitration_id

    @property
    def node_id(self) -> int:
        return self._node_id

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        arbitration_id = _ArbitrationID.from_can_message(msg)
        return cls.cmd_id == arbitration_id.cmd_id

    @classmethod
    def from_can_message(
        cls: Type[OdriveCanMessageT], msg: can.Message
    ) -> OdriveCanMessageT:
        arbitration_id = _ArbitrationID.from_can_message(msg)
        if not cls.matches(msg):
            raise ValueError(
                f"CAN message does not match the class desired {cls.__name__}!"
            )

        message = cls(node_id=arbitration_id.node_id)
        message._parse_can_msg_data(msg)
        return message

    def as_can_message(self, *, data: Optional[bytes] = None) -> can.Message:
        return can.Message(
            arbitration_id=self._arbitration_id.value,
            data=data or self._gen_can_msg_data(),
            is_extended_id=False,
        )

    def _gen_can_msg_data(self) -> bytes:
        raise NotImplementedError

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        raise NotImplementedError

    def __repr__(self) -> str:
        values_str = ", ".join(
            f"{key}: {value}" for key, value in self.__dict__.items()
        )
        return f"{self.__class__.__name__}\n ({values_str})"


##################################################################################################################
# CYCLIC MESSAGES ################################################################################################
##################################################################################################################


class HeartbeatMessage(OdriveCanMessage):
    cmd_id = 0x01

    # uint32, starts at 0th byte, axis.active_errors | axis.disarm_reason
    axis_error: int
    # uint8, starts at 4th byte, axis.current_state
    axis_state: int
    # uint8, starts at 5th byte, axis.procedure_result
    procedure_result: int
    # uint8, starts at 6th byte, axis.controller.trajectory_done (0: False, 1: True)
    trajectory_done_flag: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        msg_data_values = struct.unpack("<IBBB", bytes(msg.data[:7]))
        (
            self.axis_error,
            self.axis_state,
            self.procedure_result,
            self.trajectory_done_flag,
        ) = msg_data_values

    def __repr__(self) -> str:
        identification_str = f"{self.arbitration_id=}, {self.node_id=}"
        values_str = f"{self.axis_error=}, {self.procedure_result=}, {self.axis_state=}, {self.trajectory_done_flag=}"
        return f"{self.__class__.__name__} {identification_str}\n ({values_str})"


class EncoderEstimatesMessage(OdriveCanMessage):
    cmd_id = 0x09

    # Units in revolutions, axis.pos_vel_mapper.pos_rel or .pos_abs depending on
    # ODrive.Controller.Config.absolute_setpoints
    pos_estimate: float
    # Unit in revolutions/s, axis.pos_vel_mapper.vel
    vel_estimate: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.pos_estimate, self.vel_estimate = struct.unpack("<ff", bytes(msg.data))


class IqMessage(OdriveCanMessage):
    """A measure of the configured current.
    TODO: Quantify with a setting?
    """

    cmd_id = 0x14

    # Both in Amperes
    setpoint: float
    measured: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.setpoint, self.measured = struct.unpack("<ff", bytes(msg.data))


class ErrorMessage(OdriveCanMessage):
    """Odrive active errors."""

    cmd_id = 0x03

    # Both data points are uint32
    # NOTE: Not 100% sure this maps but lets just go with it
    active_errors: odrive_enums.ODriveError
    # TODO: Figure out what this should map to
    disarm_reason: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        active_err_int, self.disarm_reason = struct.unpack("<II", bytes(msg.data[:7]))
        self.active_errors = odrive_enums.ODriveError(active_err_int)


class TemperatureMessage(OdriveCanMessage):
    cmd_id = 0x15

    # Both in degrees C
    fet_temperature: float
    motor_temperature: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.fet_temperature, self.motor_temperature = struct.unpack(
            "<ff", bytes(msg.data)
        )


class BusVoltageCurrentMessage(OdriveCanMessage):
    cmd_id = 0x17

    # Data is in Volts
    voltage: float
    # Data in Amperes
    current: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.voltage, self.current = struct.unpack("<ff", bytes(msg.data))


class TorquesMessage(OdriveCanMessage):
    cmd_id = 0x1C

    # Both in Nm
    target: float
    estimate: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.target, self.estimate = struct.unpack("<ff", bytes(msg.data))


class VersionMessage(OdriveCanMessage):
    cmd_id = 0x00

    hw_version: str
    fw_version: str

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        msg_data_values = struct.unpack("<BBBBBBBB", bytes(msg.data[:8]))

        (
            _,
            hw_major,
            hw_minor,
            hw_variant,
            fw_major,
            fw_minor,
            fw_variant,
            _,
        ) = msg_data_values
        self.hw_version = f"{hw_major}.{hw_minor}.{hw_variant}"
        self.fw_version = f"{fw_major}.{fw_minor}.{fw_variant}"


##################################################################################################################
# COMMAND MESSAGES ###############################################################################################
##################################################################################################################


class SetAxisStateMessage(OdriveCanMessage):
    cmd_id = 0x07

    # The requested axis state, defined in the odrive enums.
    axis_state: odrive_enums.AxisState

    def _gen_can_msg_data(self) -> bytes:
        # 8 will set closed loop control mode
        return struct.pack("<I", self.axis_state.value)


class SetControllerMode(OdriveCanMessage):
    cmd_id = 0x0B

    # Control mode is a choice of voltage, torque, velocity and position control.
    control_mode: odrive_enums.ControlMode
    # Input mode is a bit more complicated (only relevant modes listed here). Typically we will use an input mode
    # of 1 unless we want to allow the motor to control itself vs just changing the setpoint.
    #   - Inactive (0): Disables input.
    #   - Passthrough (1): Allows setting the setpoint directly. Is useful in cases where we dont want the motor to
    #     decide if it should try to control the motor gradually up to a setpoint. Works for every type of control
    #   - Velocity Ramp (2): Ramps a velocity command from the current value to the target value. Uses the configured
    #     values to determine the velocity ramp rage (rev/s) and the inertia (Nm/(rev/s^2)).
    #   - Position Filter (3): 2nd order position tracking filter. Allows for cascade control of position by
    #     moderating velocity in order to reach a target position without overshooting it. Configured via the filter
    #     bandwidth and the inertia.
    #   - Trapezoidal Trajectory Planner (5): Trapezoidal trajectory planner for a position command by limiting the
    #     velocity, acceleration and deceleration. Only works with position control.
    #   - Torque Ramp (6): Ramps a torque command from the current value to the target value. Configured via the
    #     torque ramp rate.
    input_mode: odrive_enums.InputMode = odrive_enums.InputMode.PASSTHROUGH

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack("<II", self.control_mode.value, self.input_mode.value)


class SetPositionMessage(OdriveCanMessage):
    cmd_id = 0x0C

    # Position in revolutions.
    input_position: float
    # Velocity and torque feedforward values are set at 0.001 rev/s and 0.001 Nm respectively as a default. The input
    # velocity is calculated by input_vel = vel_ff / input_vel_scale. This is a configured value. The input torque is
    # calculated by input_torque = torque_ff / input_torque_scale, also a configured value.
    velocity_ff: int
    torque_ff: int

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack(
            "<fHH", self.input_position, self.velocity_ff, self.torque_ff
        )


class SetTorqueMessage(OdriveCanMessage):
    cmd_id = 0x0E

    # Torque in Nm
    input_torque: float

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack("<f", self.input_torque)


class SetVelocityMessage(OdriveCanMessage):
    cmd_id = 0x0D

    # float32 starts at 0 byte, unit: rev/s
    velocity: float
    # float32 starts at 4 byte, unit: Nm
    torque: float

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack("<ff", self.velocity, self.torque)


class EStop(OdriveCanMessage):
    """Commands the motor to immediately stop by disarming with Odrive command ESTOP_REQUESTED."""

    cmd_id = 0x02

    def _gen_can_msg_data(self) -> bytes:
        return b""


class Reboot(OdriveCanMessage):
    """Commands the motor to reboot itself."""

    cmd_id = 0x16

    # Action to take prior to a reboot.
    #   - Reboot (0): reboots the motor
    #   - Save configuration (1): Saves config prior to rebooting
    #   - Erase configuration (2): Erases config prior to rebooting
    #   - Enter DFU mode (3): Enters DFU mode
    action: int = 0

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack("<I", self.action)
