import math
from typing import Dict

import geometry
from config import robot_config

from models import constants


class TwistEstimator:
    """Estimate of the body twist using the motor wheel velocities."""

    def __init__(self, config: robot_config.Beachbot) -> None:
        self._config = config
        # Initialize all motor velocities to zero.
        self._motor_velocities: Dict[robot_config.DrivetrainLocation, float] = {}
        for drivetrain in config.drivetrain:
            self._motor_velocities[drivetrain.location] = 0.0

    @property
    def _front_left_velocity(self) -> float:
        return self._motor_velocities[robot_config.DrivetrainLocation.FRONT_LEFT]

    @property
    def _front_right_velocity(self) -> float:
        return self._motor_velocities[robot_config.DrivetrainLocation.FRONT_RIGHT]

    @property
    def _rear_left_velocity(self) -> float:
        return self._motor_velocities[robot_config.DrivetrainLocation.REAR_LEFT]

    @property
    def _rear_right_velocity(self) -> float:
        return self._motor_velocities[robot_config.DrivetrainLocation.REAR_RIGHT]

    def update(self, motor: robot_config.Motor, velocity: float) -> None:
        self._motor_velocities[motor.location] = velocity

    def twist(self) -> geometry.Twist:
        """The estimated twist in the robots body frame base on the wheel velocities."""
        left_tread_speed = self._calc_left_side_tread_speed()
        right_tread_speed = self._calc_right_side_tread_speed()

        forward_speed = (right_tread_speed - left_tread_speed) / 2
        linear_velocity = geometry.Velocity(geometry.BODY, forward_speed, 0, 0)
        angular_velocity = self._body_spin(left_tread_speed, right_tread_speed)

        return geometry.Twist(linear_velocity, angular_velocity)

    def _calc_left_side_tread_speed(self) -> float:
        """The left side tread speed assuming no wheel slippage in m/s."""
        blended_wheel_vel = (self._front_left_velocity + self._rear_left_velocity) / 2
        return self._tread_speed(blended_wheel_vel)

    def _calc_right_side_tread_speed(self) -> float:
        """The right side tread speed assuming no wheel slippage in m/s."""
        blended_wheel_vel = (self._front_right_velocity + self._rear_right_velocity) / 2
        return self._tread_speed(blended_wheel_vel)

    def _body_spin(
        self, left_tread_speed: float, right_tread_speed: float
    ) -> geometry.AngularVelocity:
        raw_tangential_speed = right_tread_speed + left_tread_speed
        # Yaw speed in degrees/s, converted from radians/s.
        frictionless_yaw_speed = math.degrees(
            raw_tangential_speed / self._config.track_width
        )
        yaw_speed = frictionless_yaw_speed * constants.WHEEL_RESISTANCE_FACTOR
        return geometry.AngularVelocity(geometry.BODY, 0, 0, yaw_speed)

    def _tread_speed(self, wheel_speed: float) -> float:
        """The tread speed of a wheel when rotating at the target wheel speed. Tread
        speed is in m/s and wheel speed is in turns/s.
        """
        return wheel_speed * self._wheel.circumference

    @property
    def _wheel(self) -> robot_config.Wheel:
        return self._config.drivetrain[0].wheel
