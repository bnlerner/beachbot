from __future__ import annotations

import json
import pathlib
from typing import Literal

import pydantic
from config import robot_config


class Motor(pydantic.BaseModel):
    """Representative of the motor, useful for CAN communication and indentifying
    the motor location.
    """

    node_id: int
    location: robot_config.DrivetrainLocation

    @property
    def direction(self) -> Literal[1, -1]:
        """The velocity direction relative to the mount point on the chassis.
        Motors on the left chassis need to turn in reverse to all respond in the
        same way to a positive body velocity.
        """
        if self.location in (
            robot_config.DrivetrainLocation.FRONT_LEFT,
            robot_config.DrivetrainLocation.REAR_LEFT,
        ):
            return -1
        else:
            return 1

    @classmethod
    def from_json(cls, file_path: pathlib.Path) -> Motor:
        if not file_path.exists():
            raise ValueError(f"File path does not exist. {file_path}")

        with open(file_path, "r") as f:
            motor_config_dict = json.load(f)

        location = robot_config.DrivetrainLocation(file_path.stem.upper())
        node_id = motor_config_dict["axis0.config.can.node_id"]

        return Motor(node_id=node_id, location=location)

    def __hash__(self) -> int:
        return hash((self.__class__, self.node_id))
