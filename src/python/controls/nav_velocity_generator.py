from config import robot_config
from drivers import primitives


class NavVelocityGenerator:
    """Generates a navigation velocity from an input target forward velocity and spin."""

    def __init__(self) -> None:
        self._linear_velocity: float = 0.0
        self._angular_velocity: float = 0.0

    def update(self, linear_velocity: float, angular_velocity: float) -> None:
        self._linear_velocity = linear_velocity
        self._angular_velocity = angular_velocity

    def velocity(self, motor: primitives.Motor) -> float:
        if motor.location in (
            robot_config.DrivetrainLocation.FRONT_LEFT,
            robot_config.DrivetrainLocation.REAR_LEFT,
        ):
            return (self._linear_velocity - self._angular_velocity) * motor.direction
        elif motor.location in (
            robot_config.DrivetrainLocation.FRONT_RIGHT,
            robot_config.DrivetrainLocation.REAR_RIGHT,
        ):
            return (self._linear_velocity + self._angular_velocity) * motor.direction
        else:
            raise ValueError(f"Unknown motor config location {motor=}")
