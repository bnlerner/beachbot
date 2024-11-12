from __future__ import annotations

import enum
import functools
import json
import math
import pathlib
from typing import List, Literal

import pydantic


class Wheel(pydantic.BaseModel):
    """The configuration of the wheel"""

    # Found on their product page from amazon
    # 13", in meters
    diameter: float = 0.3302
    # 7.1", in meters
    tread: float = 0.1803

    @property
    def circumference(self) -> float:
        return self.diameter * math.pi


# TODO: Move the location and motor into a driver primitives since its not really a
# config.
# Str inheritance required so pydantic treats this as a string when
# serializing base model objects.
class DrivetrainLocation(str, enum.Enum):
    """The location of the drivetrain on the chassis relative to the antenna
    location (front).
    """

    FRONT_LEFT = "FRONT_LEFT"
    FRONT_RIGHT = "FRONT_RIGHT"
    REAR_LEFT = "REAR_LEFT"
    REAR_RIGHT = "REAR_RIGHT"


class Motor(pydantic.BaseModel):
    """Representative of the motor, useful for CAN communication and identifying
    the motor location.
    """

    node_id: int
    location: DrivetrainLocation

    @functools.cached_property
    def side(self) -> Literal["left", "right"]:
        """Side of the robot the motor is on."""
        if self.location in (
            DrivetrainLocation.FRONT_LEFT,
            DrivetrainLocation.REAR_LEFT,
        ):
            return "left"
        else:
            return "right"

    @classmethod
    def from_json(cls, file_path: pathlib.Path) -> Motor:
        if not file_path.exists():
            raise ValueError(f"File path does not exist. {file_path}")

        with open(file_path, "r") as f:
            motor_config_dict = json.load(f)

        location = DrivetrainLocation(file_path.stem.upper())
        node_id = motor_config_dict["axis0.config.can.node_id"]

        return Motor(node_id=node_id, location=location)

    def __hash__(self) -> int:
        return hash((self.__class__, self.node_id))


class Drivetrain(pydantic.BaseModel):
    """The drivetrain of a single axle and its location, There are 4 drivetrains in a
    4 wheeled non-holonomic robot.
    """

    location: DrivetrainLocation
    wheel: Wheel


class Beachbot(pydantic.BaseModel):
    """A beachbot robot. Contains a robot with 4 independently driven wheels + motor
    drivetrain.
    """

    drivetrain: List[Drivetrain]

    inner_axle_wheel_distance: float = 0.393
    wheel_base: float = 0.405

    @property
    def track_width(self) -> float:
        """The width between the two wheel centers."""
        return (
            self.inner_axle_wheel_distance
            + self.front_left_wheel.tread / 2
            + self.front_right_wheel.tread / 2
        )

    @property
    def front_left_wheel(self) -> Wheel:
        drivetrain = self._get_filtered_drivetrain(DrivetrainLocation.FRONT_LEFT)
        return drivetrain.wheel

    @property
    def front_right_wheel(self) -> Wheel:
        drivetrain = self._get_filtered_drivetrain(DrivetrainLocation.FRONT_RIGHT)
        return drivetrain.wheel

    @property
    def rear_left_wheel(self) -> Wheel:
        drivetrain = self._get_filtered_drivetrain(DrivetrainLocation.REAR_LEFT)
        return drivetrain.wheel

    @property
    def rear_right_wheel(self) -> Wheel:
        drivetrain = self._get_filtered_drivetrain(DrivetrainLocation.REAR_RIGHT)
        return drivetrain.wheel

    @functools.lru_cache()
    def _get_filtered_drivetrain(self, location: DrivetrainLocation) -> Drivetrain:
        return filter(lambda x: x.location == location, self.drivetrain).__next__()
