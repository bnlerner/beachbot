from __future__ import annotations

import enum
import functools
from typing import List

import pydantic


class Wheel(pydantic.BaseModel):
    """The configuration of the wheel"""

    # Found on their product page from amazon
    # 13", in meters
    diameter: float = 0.3302
    # 7.1", in meters
    tread: float = 0.1803


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
