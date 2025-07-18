from typing import List

import geometry
from config import robot_config
from models import body_model

from controls import motor_velocity_controller, pid_controller

_LINEAR_P_GAIN = 0.3
_LINEAR_I_GAIN = 0.5
_ANGULAR_P_GAIN = 0.3
_ANGULAR_I_GAIN = 0.05


class NavVelocityController:
    """Controls the velocity of the robot for navigation. Takes updates in the form of
    a target and measured twist and outputs the motor velocities needed to achieve them.
    """

    def __init__(
        self, motors: List[robot_config.Motor], config: robot_config.Beachbot
    ) -> None:
        self._body_model = body_model.BodyModel(config)
        self._linear_pi_control = pid_controller.PIDController(
            _LINEAR_P_GAIN, _LINEAR_I_GAIN
        )
        self._angular_pi_control = pid_controller.PIDController(
            _ANGULAR_P_GAIN, _ANGULAR_I_GAIN
        )
        self._motor_vel_control = motor_velocity_controller.MotorVelocityController(
            motors, config
        )

    def update(self, target: geometry.Twist, measured: geometry.Twist) -> None:
        """Takes the target and measured twist values to output the linear and angular
        velocity the robot needs to take to achieve them.
        """
        linear_vel_fb = self._linear_velocity(target.velocity, measured.velocity)
        angular_vel_fb = self._angular_velocity(target.spin, measured.spin)
        linear_speed = target.velocity.x + linear_vel_fb
        angular_speed = target.spin.z + angular_vel_fb

        self._motor_vel_control.set_target(linear_speed, angular_speed)

    def velocity(self, motor: robot_config.Motor) -> float:
        """Motor velocity in turns/s to achieve the control inputs."""
        return self._motor_vel_control.calc_wheel_speed(motor)

    def feedforward_torque(self) -> float:
        return self._motor_vel_control.calc_feedforward_torque()

    def reset(self) -> None:
        self._linear_pi_control.reset()
        self._angular_pi_control.reset()

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
