import collections
import time
from typing import DefaultDict, List

import geometry
from config import robot_config
from models import body_model

_TIMEOUT = 0.5  # Seconds
# The max the torque setpoint is allowed change between time steps.
_MAX_TORQUE_DELTA = 0.015
# The max the wheel speed setpoint is allowed change between time steps.
_MAX_SPEED_DELTA = 0.05


class MotorVelocityController:
    """Outputs the commands to control to the motor to the target linear and angular
    speed. Solves the problem of "Updates to the target velocity come slower than the
    motor's own update frequency". In practices updating the motor at a slower update
    rate causes chatter due to the update value delta is large. Setting the target at a
    slower rate is possible with this controller by effectively causing the wheel speed
    target and torque target to be ramped.

    Typically run this controller at 100 hz or higher for good performance. Updates to
    the target can be done at a lower frequency.
    """

    def __init__(self, motors: List[robot_config.Motor], config: robot_config.Beachbot):
        self._motors = motors
        self._model = body_model.BodyModel(config)
        # Caches the set last wheel speed and feedforward torque so it can be used to
        # effectively ramp the values.
        self._cached_wheel_speeds: DefaultDict[
            robot_config.Motor, float
        ] = collections.defaultdict(lambda: 0.0)
        self._cached_ff_torque: float = 0.0
        self._timestamp = time.perf_counter()

    def set_target(self, linear_speed: float, angular_speed: float) -> None:
        """Sets the target speeds of the robot."""
        self._model.set_target(linear_speed, angular_speed)
        # Used to ensure we are not using stale targets.
        self._timestamp = time.perf_counter()

    def calc_wheel_speed(self, motor: robot_config.Motor) -> float:
        """The wheel speed calculated from the set target velocities. Ramps the output
        wheel speed over time based on how frequently this method is called. Stale
        targets can expire which causes this controller to set its target velocities to
        zero and command the motors to stop.
        """
        if self._is_expired():
            self._model.set_target(0.0, 0.0)

        self._update_wheel_speed(motor)
        return self._cached_wheel_speeds[motor]

    def calc_feedforward_torque(self) -> float:
        """The feedforward calculated from the set target velocities. Useful to ensure
        turning the robot matches expectation at lower linear speeds. Ramps the
        feedforward torque over time based on how frequently this method is called.
        """
        if self._is_expired():
            self._model.set_target(0.0, 0.0)

        self._update_wheel_ff_torque()
        return self._cached_ff_torque

    def _is_expired(self) -> bool:
        """Last set command expires if it exceeds the timeout."""
        return time.perf_counter() - self._timestamp > _TIMEOUT

    def _update_wheel_speed(self, motor: robot_config.Motor) -> None:
        wheel_speed = self._model.wheel_speed(motor)
        diff = wheel_speed - self._cached_wheel_speeds[motor]

        self._cached_wheel_speeds[motor] += geometry.clip(
            diff, -_MAX_SPEED_DELTA, _MAX_SPEED_DELTA
        )

    def _update_wheel_ff_torque(self) -> None:
        ff_torque = self._model.feedforward_torque()
        diff = ff_torque - self._cached_ff_torque
        self._cached_ff_torque += geometry.clip(
            diff, -_MAX_TORQUE_DELTA, _MAX_TORQUE_DELTA
        )
