import collections
from typing import DefaultDict

from config import robot_config
from drivers import primitives
from pynput import keyboard


class RCVelocityGenerator:
    """Inputs a motor velocity and the motor configs and will output the motor velocities to respond to key presses."""

    def __init__(self, velocity_default: float) -> None:
        self._velocity_default = velocity_default
        self._pressed_keys: DefaultDict[keyboard.Key, bool] = collections.defaultdict(
            lambda: False
        )

    def update(self, key: keyboard.Key, *, pressed: bool) -> None:
        self._pressed_keys[key] = pressed

    def velocity(self, motor: primitives.Motor) -> float:
        """The motor velocity in turns / s to achieve the current RC key presses."""
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

    def _angular_velocity(self) -> float:
        """Angular velocity assuming that the velocity rotates about the robots body axis which would be with
        the x-axis pointed forward and the z-axis pointed skyward.
        """
        vel = 0.0
        if self._right_key_pressed:
            vel -= self._velocity_default
        # Left turn is a positive angular velocity.
        if self._left_key_pressed:
            vel += self._velocity_default

        return vel

    def _linear_velocity(self) -> float:
        """The linear velocity the robot will travel forward."""
        vel = 0.0
        if self._up_key_pressed:
            vel += self._velocity_default
        if self._down_key_pressed:
            vel -= self._velocity_default

        return vel

    @property
    def _up_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.up]

    @property
    def _down_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.down]

    @property
    def _left_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.left]

    @property
    def _right_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.right]
