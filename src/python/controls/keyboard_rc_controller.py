import collections
from typing import DefaultDict

from config import robot_config
from models import body_model
from pynput import keyboard

_LINEAR_VELOCITY_DEFAULT = 2  # m/s
_ANGULAR_VELOCITY_DEFAULT = 5  # deg/s


class KeyboardRCController:
    """Generates a velocity for a motor config based on the input keyboard key
    presses.
    """

    def __init__(self, config: robot_config.Beachbot) -> None:
        self._pressed_keys: DefaultDict[keyboard.Key, bool] = collections.defaultdict(
            lambda: False
        )
        self._body_model = body_model.BodyModel(config)

    def update(self, key: keyboard.Key, *, pressed: bool) -> None:
        self._pressed_keys[key] = pressed
        linear_vel = self._linear_velocity()
        angular_vel = self._angular_velocity()
        self._body_model.update(linear_vel, angular_vel)

    def velocity(self, motor: robot_config.Motor) -> float:
        """The motor velocity in turns / s to achieve the current RC key presses."""
        return self._body_model.velocity(motor)

    def _angular_velocity(self) -> float:
        """Angular velocity assuming that the velocity rotates about the robots body axis which would be with
        the x-axis pointed forward and the z-axis pointed skyward.
        """
        vel = 0.0
        if self._right_key_pressed:
            vel -= _ANGULAR_VELOCITY_DEFAULT
        # Left turn is a positive angular velocity.
        if self._left_key_pressed:
            vel += _ANGULAR_VELOCITY_DEFAULT

        return vel

    def _linear_velocity(self) -> float:
        """The linear velocity the robot will travel forward."""
        vel = 0.0
        if self._up_key_pressed:
            vel += _LINEAR_VELOCITY_DEFAULT
        if self._down_key_pressed:
            vel -= _LINEAR_VELOCITY_DEFAULT

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
