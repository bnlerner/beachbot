from __future__ import annotations

from typing import Any, Optional, Type, TypeVar, Union

import can
import pydantic

CanMessageT = TypeVar("CanMessageT", bound="CanMessage")


class OdriveArbitrationID(pydantic.BaseModel):
    """CAN arbitration ID is used to uniquely identify the message by the node and
    the command id.
    """

    node_id: int
    cmd_id: int

    @classmethod
    def from_can_message(cls, msg: can.Message) -> OdriveArbitrationID:
        return cls(
            node_id=(msg.arbitration_id >> 5), cmd_id=(msg.arbitration_id & 0b11111)
        )

    @property
    def value(self) -> int:
        return self.node_id << 5 | self.cmd_id

    def __hash__(self) -> int:
        return hash((self.node_id, self.cmd_id))


class MyActuatorArbitrationID(pydantic.BaseModel):
    """CAN arbitration ID is used to uniquely identify the message by the node and
    the command id.
    """

    node_id: int
    cmd_id: int
    custom_value: Optional[int] = None  # For special cases like CANIDCommand

    @classmethod
    def from_can_message(cls, msg: can.Message) -> MyActuatorArbitrationID:
        # MyActuator format - see motor_protocol.md
        # Single motor: 0x140+ID (node_id 1-32) for sending, 0x240+ID for receiving
        # Extract node_id by subtracting the base (0x140 or 0x240)
        if 0x140 <= msg.arbitration_id < 0x160:  # Command message
            return cls(
                node_id=(msg.arbitration_id - 0x140),
                cmd_id=msg.data[0],  # Command is first byte of data
            )
        elif 0x240 <= msg.arbitration_id < 0x260:  # Reply message
            return cls(
                node_id=(msg.arbitration_id - 0x240),
                cmd_id=msg.data[0],  # Command is first byte of data
            )
        # Multi-motor command messages and motion mode control messages are not yet implemented
        else:
            raise ValueError(f"Invalid MyActuator arbitration ID: {msg.arbitration_id}")

    @property
    def value(self) -> int:
        # Use custom value if provided, otherwise use standard format
        if self.custom_value is not None:
            return self.custom_value
        # For sending to motor, use 0x140+ID format
        return 0x140 + self.node_id


class X424ArbitrationID(pydantic.BaseModel):
    """CAN arbitration ID is used to uniquely identify the message by the node and
    the command id.
    """

    node_id: int
    cmd_id: int

    @classmethod
    def from_can_message(cls, msg: can.Message) -> X424ArbitrationID:
        node_id = (msg.arbitration_id >> 8) & 0xFF
        cmd_id = msg.arbitration_id & 0xFF
        return cls(node_id=node_id, cmd_id=cmd_id)

    @property
    def value(self) -> int:
        return self.node_id


class CanMessage:
    """A class representing different types of CAN messages that can be
    sent to the node and received from the node.
    """

    cmd_id: int

    def __init__(self, node_id: int, **kwargs: Any):
        self._node_id = node_id
        self._arbitration_id = self._gen_arbitration_id()

        for arg_name, value in kwargs.items():
            setattr(self, arg_name, value)

    @property
    def node_id(self) -> int:
        return self._node_id

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        raise NotImplementedError

    def _gen_arbitration_id(
        self,
    ) -> Union[OdriveArbitrationID, MyActuatorArbitrationID, X424ArbitrationID]:
        raise NotImplementedError

    @classmethod
    def from_can_message(cls: Type[CanMessageT], msg: can.Message) -> CanMessageT:
        """Convert a CAN message to a specific message type.
        This needs to be implemented by subclasses."""
        raise NotImplementedError

    def as_can_message(self) -> can.Message:
        return can.Message(
            arbitration_id=self._arbitration_id.value,
            data=self._gen_can_msg_data(),
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
        return f"{self.__class__.__name__}({values_str})"
