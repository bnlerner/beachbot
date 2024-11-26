import math
from typing import Literal, Optional

import geometry
from config import robot_config

_X_POS_GAIN = 0.001
_Y_POS_GAIN = 22.9
_THETA_ANGLE_GAIN = 22.9


class NavVelTargetGenerator:
    """Generates the target navigation velocities to control to given a target pose in
    the body of the robot and a reference speed and turn radius to travel along.
    """

    def __init__(self, config: robot_config.Beachbot):
        self._config = config

    def gen_twist(
        self,
        signed_turn_radius: Optional[float],
        reference_speed: float,
        target_in_body: geometry.Pose,
    ) -> geometry.Twist:
        """Twist to target for the robot. Modifies the twist to try to control the robot gradually back to the nav point if its"""
        path_direction = target_in_body.orientation.x_axis()
        ref_twist = self._reference_twist(
            path_direction, signed_turn_radius, reference_speed
        )

        driving_direction = geometry.sign(path_direction.x)
        corrective_twist = self._corrective_twist(target_in_body, driving_direction)

        return ref_twist + corrective_twist

    def _reference_twist(
        self,
        path_direction: geometry.Direction,
        signed_turn_radius: Optional[float],
        reference_speed: float,
    ) -> geometry.Twist:
        """The modeled twist to achieve the reference speed along the path at the target
        turn radius.
        """
        linear_speed = self._calc_linear_speed(signed_turn_radius, reference_speed)
        velocity = geometry.Velocity.from_direction(path_direction, linear_speed)
        yaw_speed = self._calc_yaw_speed(signed_turn_radius, reference_speed)
        angular_vel = geometry.AngularVelocity(geometry.BODY, 0, 0, yaw_speed)
        return geometry.Twist(velocity, angular_vel)

    def _corrective_twist(
        self, target_in_body: geometry.Pose, driving_direction: Literal[-1, 0, 1]
    ) -> geometry.Twist:
        """Attempts to correct for any errors in the position and orientation of the
        target in the current body frame.
        """
        speed_error = _X_POS_GAIN * target_in_body.position.x
        corrective_vel = geometry.Velocity.unit_x(geometry.BODY) * speed_error
        spin_speed = (
            _Y_POS_GAIN * target_in_body.position.y
            + _THETA_ANGLE_GAIN * math.sin(target_in_body.orientation.yaw)
        )
        corrective_spin = geometry.AngularVelocity(
            geometry.BODY, 0, 0, spin_speed * driving_direction
        )

        return geometry.Twist(corrective_vel, corrective_spin)

    def _calc_linear_speed(
        self, signed_turn_radius: Optional[float], reference_speed: float
    ) -> float:
        # Travelling straight so no need to account for turning.
        if signed_turn_radius is None:
            return reference_speed

        abs_turn_radius = abs(signed_turn_radius)
        half_track_width = self._config.track_width / 2
        inside_track_dist = abs_turn_radius - half_track_width
        outside_track_dist = abs_turn_radius + half_track_width
        return reference_speed / 2 * (1 + inside_track_dist / outside_track_dist)

    def _calc_yaw_speed(
        self, signed_turn_radius: Optional[float], reference_speed: float
    ) -> float:
        # No yaw speed when travelling straight.
        if signed_turn_radius is None:
            return 0.0

        abs_turn_radius = abs(signed_turn_radius)
        half_track_width = self._config.track_width / 2
        outside_track_dist = abs_turn_radius + half_track_width
        return reference_speed / outside_track_dist * geometry.sign(signed_turn_radius)
