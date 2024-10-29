from __future__ import annotations

import enum
import json
import pathlib
from typing import Literal

import pydantic

# Str inheritance required so pydantic treats this as a string when
# serializing base model objects.
class MotorLocation(str, enum.Enum):
    """The location of the motor on the chassis relative to the antenna
    location (front).
    """
    FRONT_LEFT = "FRONT_LEFT"
    FRONT_RIGHT = "FRONT_RIGHT"
    REAR_LEFT = "REAR_LEFT"
    REAR_RIGHT = "REAR_RIGHT"


class MotorConfig(pydantic.BaseModel):
    """Representative of the motor, useful for CAN communication and indentifying
    the motor location.
    """
    node_id: int
    location: MotorLocation

    @property
    def direction(self) -> Literal[1, -1]:
        """The velocity direction relative to the mount point on the chassis.
        Motors on the left chassis need to turn in reverse to all respond in the
        same way to a positive body velocity.
        """
        if self.location in (MotorLocation.FRONT_LEFT, MotorLocation.REAR_LEFT):
            return -1
        else:
            return 1

    @classmethod
    def from_json(cls, file_path: pathlib.Path) -> MotorConfig:
        if not file_path.exists():
            raise ValueError(f"File path does not exist. {file_path}")

        with open(file_path, "r") as f:
            motor_config_dict = json.load(f)

        location = MotorLocation(file_path.stem.upper())
        node_id = motor_config_dict["axis0.config.can.node_id"]

        return MotorConfig(node_id=node_id, location=location)

    def __hash__(self) -> int:
        return hash((self.__class__, self.node_id))
