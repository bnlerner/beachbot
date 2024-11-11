from typing import Tuple

from config import robot_config
from drivers import primitives as hw_primitives
from planning import primitives as pl_primitives


class FollowPathController:
    """Follows a nav path."""

    def __init__(self, path: pl_primitives.NavigationPath) -> None:
        self._path = path

    def update(
        self, target: Tuple[float, float], measured: Tuple[float, float]
    ) -> None:
        """Takes the target and measured pseudo twist values [linear velocity forward of
        the robot, angular speed about its z-axis]. Required to get the actual
        """
        vel_error = target[0] - measured[0]
        spin_error = target[1] - measured[1]

    def velocity(self, motor: hw_primitives.Motor) -> float:
        linear_vel = self._linear_velocity()
        angular_vel = self._angular_velocity()

        if motor.location in (
            robot_config.DrivetrainLocation.FRONT_LEFT,
            robot_config.DrivetrainLocation.REAR_LEFT,
        ):
            return (linear_vel - angular_vel) * motor.direction
        elif motor.location in (
            robot_config.DrivetrainLocation.FRONT_RIGHT,
            robot_config.DrivetrainLocation.REAR_RIGHT,
        ):
            return (linear_vel + angular_vel) * motor.direction
        else:
            raise ValueError(f"Unknown motor config location {motor=}")

    def _linear_velocity(self) -> float:
        return 0.0

    def _angular_velocity(self) -> float:
        return 0.0
