import geometry
from config import robot_config
from models import motor_velocity_model

from controls import pid_controller

_LINEAR_PROPORTIONAL_GAIN = 0.3
_LINEAR_INTEGRAL_GAIN = 0.5
_ANGULAR_PROPORTIONAL_GAIN = 0.3
_ANGULAR_INTEGRAL_GAIN = 0.05


class FollowPathController:
    """Follows a nav path."""

    def __init__(self, config: robot_config.Beachbot) -> None:
        self._motor_model = motor_velocity_model.MotorVelocityModel(config)
        self._linear_pi_control = pid_controller.PIDController(
            _LINEAR_PROPORTIONAL_GAIN, _LINEAR_INTEGRAL_GAIN
        )
        self._angular_pi_control = pid_controller.PIDController(
            _ANGULAR_PROPORTIONAL_GAIN, _ANGULAR_INTEGRAL_GAIN
        )

    def update(self, target: geometry.Twist, measured: geometry.Twist) -> None:
        """Takes the target and measured twist values to update the internal state.
        Required to get the actual
        """
        linear_vel_fb = self._linear_velocity(target.velocity, measured.velocity)
        angular_vel_fb = self._angular_velocity(target.spin, measured.spin)
        linear_speed = target.velocity.x + linear_vel_fb
        angular_speed = target.spin.z + angular_vel_fb
        self._motor_model.update(linear_speed, angular_speed)

    def velocity(self, motor: robot_config.Motor) -> float:
        return self._motor_model.velocity(motor)

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
