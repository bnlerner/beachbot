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

    # float32, starts at 0th byte, unit: revolutions, axis.pos_vel_mapper.pos_rel or .pos_abs depending
    # on ODrive.Controller.Config.absolute_setpoints
    pos_estimate: float
    # float32, starts at 4th byte, unit: rev/s, axis.pos_vel_mapper.vel
    vel_estimate: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.pos_estimate, self.vel_estimate = struct.unpack("<ff", bytes(msg.data))

    def __repr__(self) -> str:
        identification_str = f"{self.arbitration_id=}, {self.node_id=}"
        values_str = f"pos: {self.pos_estimate:.3f} [turns], vel: {self.vel_estimate:.3f} [turns/s]"
        return f"{self.__class__.__name__} {identification_str}\n ({values_str})"


class IqMessage(OdriveCanMessage):
    """A measure of the configured current.
    TODO: Quantify with a setting?
    """

    cmd_id = 0x14

    # float32, starts at 0th byte, unit: Amperes
    setpoint: float
    # float32, starts at 4th byte, unit: Amperes
    measured: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.setpoint, self.measured = struct.unpack("<ff", bytes(msg.data))


class ErrorMessage(OdriveCanMessage):
    cmd_id = 0x03

    active_errors: int  # starts at 0 byte, 4 byte, uint32
    disarm_reason: int  # starts at 4 byte, 4 byte, uint32

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        msg_data_values = struct.unpack("<II", bytes(msg.data[:7]))
        (
            self.axis_error,
            self.axis_state,
            self.procedure_result,
            self.trajectory_done_flag,
        ) = msg_data_values


class TemperatureMessage(OdriveCanMessage):
    cmd_id = 0x15

    fet_temperature: float
    motor_temperature: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.setpoint, self.measured = struct.unpack("<ff", bytes(msg.data))


class BusVoltageCurrentMessage(OdriveCanMessage):
    cmd_id = 0x17
    # TODO: add data args and parse function.


class TorquesMessage(OdriveCanMessage):
    cmd_id = 0x1C
    # TODO: add data args and parse function.


##################################################################################################################
# COMMAND MESSAGES ###############################################################################################
##################################################################################################################


class SetAxisStateMessage(OdriveCanMessage):
    cmd_id = 0x07

    # The requested axis state, defined in the odrive enums.
    axis_state: enums.AxisState

    def _gen_can_msg_data(self) -> bytes:
        # 8 will set closed loop control mode
        return struct.pack("<I", self.axis_state.value)


class SetVelocityMessage(OdriveCanMessage):
    cmd_id = 0x0D

    # float32 starts at 0 byte, unit: rev/s
    velocity: float
    # float32 starts at 4 byte, unit: Nm
    torque: float

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack("<ff", self.velocity, self.torque)
