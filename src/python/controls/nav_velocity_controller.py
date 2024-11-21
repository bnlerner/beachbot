from typing import Tuple

import geometry
from config import robot_config
from models import motor_velocity_model

from controls import pid_controller

_LINEAR_P_GAIN = 0.3
_LINEAR_I_GAIN = 0.5
_ANGULAR_P_GAIN = 0.3
_ANGULAR_I_GAIN = 0.05


class NavVelocityController:
    """Controls the velocity of the robot for navigation. Takes updates in the form of
    a target and measured twist and outputs the motor velocities needed to achieve them.
    """

    def __init__(self, config: robot_config.Beachbot) -> None:
        self._motor_model = motor_velocity_model.MotorVelocityModel(config)
        self._linear_pi_control = pid_controller.PIDController(
            _LINEAR_P_GAIN, _LINEAR_I_GAIN
        )
        self._angular_pi_control = pid_controller.PIDController(
            _ANGULAR_P_GAIN, _ANGULAR_I_GAIN
        )

    def velocities(
        self, target: geometry.Twist, measured: geometry.Twist
    ) -> Tuple[float, float]:
        """Takes the target and measured twist values to output the linear and angular
        velocity the robot needs to take to achieve them.
        """
        linear_vel_fb = self._linear_velocity(target.velocity, measured.velocity)
        angular_vel_fb = self._angular_velocity(target.spin, measured.spin)
        linear_speed = target.velocity.x + linear_vel_fb
        angular_speed = target.spin.z + angular_vel_fb

        return linear_speed, angular_speed

    def _linear_velocity(
        self, target: geometry.Velocity, measured: geometry.Velocity
    ) -> float:
        error = target.x - measured.x
        return self._linear_pi_control.control_signal(error)

    def _angular_velocity(
        self, target: geometry.AngularVelocity, measured: geometry.AngularVelocity
    ) -> float:
        error = target.z - measured.z
        return self._angular_pi_control.control_signal(error)
