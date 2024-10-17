from __future__ import annotations
import can
from typing import Optional, Any, TypeVar, Type
import struct
import pydantic

OdriveCanMessageT = TypeVar("OdriveCanMessageT", bound="OdriveCanMessage")


class _ArbitrationID(pydantic.BaseModel):
    node_id: int
    cmd_id: int

    @classmethod
    def from_can_message(cls, msg: can.Message) -> _ArbitrationID:
        return cls(
            node_id=(msg.arbitration_id >> 5),
            cmd_id=(msg.arbitration_id & 0b11111),
        )

    @property
    def value(self) -> int:
        return (self.node_id << 5 | self.cmd_id)

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
    def from_can_message(cls: Type[OdriveCanMessageT], msg: can.Message) -> OdriveCanMessageT:
        arbitration_id = _ArbitrationID.from_can_message(msg)
        if not cls.matches(msg):
            raise ValueError(f"CAN message does not match the class desired {cls.__name__}!")

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
        msg_data_values = struct.unpack('<IBBB', bytes(msg.data[:7]))
        self.axis_error, self.axis_state, self.procedure_result, self.trajectory_done_flag = msg_data_values

class EncoderEstimatesMessage(OdriveCanMessage):
    cmd_id = 0x09

    # float32, starts at 0th byte, unit: revolutions, axis.pos_vel_mapper.pos_rel or .pos_abs depending
    # on ODrive.Controller.Config.absolute_setpoints
    pos_estimate: float
    # float32, starts at 4th byte, unit: rev/s, axis.pos_vel_mapper.vel
    vel_estimate: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.pos_estimate, self.vel_estimate = struct.unpack('<ff', bytes(msg.data))

class IqMessage(OdriveCanMessage):
    cmd_id = 0x14
     # TODO: add data args and parse function.

class ErrorMessage(OdriveCanMessage):
    cmd_id = 0x03

    active_errors: int # starts at 0 byte, 4 byte, uint32
    disarm_reason: int # starts at 4 byte, 4 byte, uint32
     # TODO: add data args and parse function.

class TemperatureMessage(OdriveCanMessage):
    cmd_id = 0x15
     # TODO: add data args and parse function.

class BusVoltageCurrentMessage(OdriveCanMessage):
    cmd_id = 0x17
     # TODO: add data args and parse function.

class TorquesMessage(OdriveCanMessage):
    cmd_id = 0x1c
     # TODO: add data args and parse function.


##################################################################################################################
# COMMAND MESSAGES ###############################################################################################
##################################################################################################################

class SetAxisStateMessage(OdriveCanMessage):
    cmd_id = 0x07

    axis_state: int

    def _gen_can_msg_data(self) -> bytes:
        # 8 will set closed loop control mode
        return struct.pack('<I', self.axis_state)

class SetVelocityMessage(OdriveCanMessage):
    cmd_id = 0x0d

    velocity: float

    def _gen_can_msg_data(self) -> bytes:
        return struct.pack('<ff', self.velocity, 0.0)

