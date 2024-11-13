from typing import Tuple

from config import robot_config
from models import motor_velocity_model
from planning import primitives


class FollowPathController:
    """Follows a nav path."""

    def __init__(
        self, path: primitives.NavigationPath, config: robot_config.Beachbot
    ) -> None:
        self._path = path
        self._motor_model = motor_velocity_model.MotorVelocityModel(config)

    def update(
        self, target: Tuple[float, float], measured: Tuple[float, float]
    ) -> None:
        """Takes the target and measured pseudo twist values [linear velocity forward of
        the robot, angular speed about its z-axis]. Required to get the actual
        """
        vel_error = target[0] - measured[0]
        spin_error = target[1] - measured[1]
        linear_vel = self._linear_velocity()
        angular_vel = self._angular_velocity()
        self._motor_model.update(linear_vel, angular_vel)

    def velocity(self, motor: robot_config.Motor) -> float:
        return self._motor_model.velocity(motor)

    def _linear_velocity(self) -> float:
        return 0.5

    def _angular_velocity(self) -> float:
        return 0.0
