import math

import geometry
from config import robot_config

from models import constants

# The linear speed at which feedforward compensation is no longer needed to achieve the
# angular speed target.
_FEEDFORWARD_LINEAR_SPEED_CUT_OFF = 2.0


class BodyModel:
    """Generates a model of the robot body, taking target linear and angular velocities
    the body should achieve and outputs the motor velocity to do so.
    """

    def __init__(self, config: robot_config.Beachbot) -> None:
        self._config = config

        self._linear_speed: float = 0.0
        self._angular_speed: float = 0.0

    def set_target(self, linear_speed: float, angular_speed: float) -> None:
        """Sets the target linear velocity (m/s) and angular velocity (deg/s) in the
        model.
        """
        self._linear_speed = linear_speed
        self._angular_speed = angular_speed

    def wheel_speed(self, motor: robot_config.Motor) -> float:
        """The motor wheel speed (turns/s) to target to achieve the stored linear and
        angular velocity targets.
        """
        tangential_speed = self._wheel_tangential_speed(self._angular_speed)
        # The velocity direction relative to the mount point on the chassis.
        # Motors on the left chassis need to turn in reverse to all respond in the
        # same way to a positive body velocity.
        if motor.side == "left":
            tread_speed = tangential_speed - self._linear_speed
        else:
            tread_speed = tangential_speed + self._linear_speed

        return self._convert_tread_velocity_to_motor_velocity(tread_speed)

    def feedforward_torque(self) -> float:
        """The feedforward torque required to compensate for turning at low linear speeds.
        When strictly counterrotating, the friction of the wheels to the ground must be
        overcome to achieve the target speed.
        """
        # Not needed if not turning
        if self._angular_speed == 0.0:
            return 0.0

        # Sets a linear speed at which this feedforward torque is no longer
        # needed due to forward motion overcomming wheel friction.
        scalar = 1.0 - geometry.linear_ramp(
            abs(self._linear_speed), _FEEDFORWARD_LINEAR_SPEED_CUT_OFF
        )
        sign = geometry.sign(self._angular_speed)
        return sign * constants.TURNING_STATIC_FRICTION_TORQUE * scalar

    def _wheel_tangential_speed(self, angular_speed: float) -> float:
        """The tangential speed a single wheel velocity must rotate to achieve the
        angular speed. This value is side agnostic as it can be inverted depending on
        the wheel location. Angular speed is in degrees / second.
        """
        half_track_width = self._config.track_width / 2
        rad_s = math.radians(angular_speed)
        return (rad_s * half_track_width) / constants.WHEEL_RESISTANCE_FACTOR

    def _convert_tread_velocity_to_motor_velocity(self, tread_speed: float) -> float:
        """Tread velocity is the velocity of the wheel located at the contact point of
        the tread. In order to covert to wheel velocity we must divide by the wheel
        circumference to get turns /s. For example, if the tread velocity is 1 m/s and
        the circumference of the wheel is 3 m then it is turning at 1/3 turns/s.
        """
        return tread_speed / self._wheel.circumference

    @property
    def _wheel(self) -> robot_config.Wheel:
        return self._config.drivetrain[0].wheel
