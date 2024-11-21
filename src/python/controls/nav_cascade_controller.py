import math
from typing import Literal

import geometry
from config import robot_config
from models import motor_velocity_model
from planning import primitives

from controls import nav_velocity_controller

_X_POS_GAIN = 0.001
_Y_POS_GAIN = 22.9
_THETA_ANGLE_GAIN = 22.9


class NavigationCascadeController:
    """A cascade controller to handle navigating along the path."""

    def __init__(self, config: robot_config.Beachbot):
        self._robot_config = config
        self._velocity_controller = nav_velocity_controller.NavVelocityController(
            config
        )
        self._motor_model = motor_velocity_model.MotorVelocityModel(config)

    def update(
        self,
        navpoint: primitives.NavigationPoint,
        cur_pose: geometry.Pose,
        reference_speed: float,
        cur_twist: geometry.Twist,
    ) -> None:
        self._navpoint = navpoint
        self._cur_pose = cur_pose
        self._reference_speed = reference_speed
        linear_speed, angular_speed = self._velocity_controller.velocities(
            self._target_twist(), cur_twist
        )
        self._motor_model.update(linear_speed, angular_speed)

    def velocity(self, motor: robot_config.Motor) -> float:
        """Motor velocity in turns/s to achieve the control inputs."""
        return self._motor_model.velocity(motor)

    def _target_twist(self) -> geometry.Twist:
        """Twist to target for the robot."""
        ref_twist = self._reference_twist()
        target_in_body = self._cur_pose.transform(self._navpoint.pose_2d)
        corrective_twist = self._corrective_twist(
            target_in_body, geometry.sign(ref_twist.velocity.x)
        )
        return ref_twist + corrective_twist

    def _reference_twist(self) -> geometry.Twist:
        signed_turn_radius = self._navpoint.signed_turn_radius
        direction = self._navpoint.pose_2d.orientation.x_axis()
        if signed_turn_radius:
            linear_speed = self._calc_linear_speed(signed_turn_radius)
            yaw_speed = self._calc_yaw_speed(signed_turn_radius)
            angular_vel = geometry.AngularVelocity(geometry.UTM, 0, 0, yaw_speed)
        else:
            linear_speed = self._reference_speed
            angular_vel = geometry.AngularVelocity.zero(geometry.UTM)

        velocity = geometry.Velocity.from_direction(
            geometry.UTM, direction, linear_speed
        )
        return geometry.Twist(velocity, angular_vel)

    def _calc_linear_speed(self, signed_turn_radius: float) -> float:
        abs_turn_radius = abs(signed_turn_radius)
        half_track_width = self._robot_config.track_width / 2
        inside_track_dist = abs_turn_radius - half_track_width
        outside_track_dist = abs_turn_radius + half_track_width
        return self._reference_speed / 2 * (1 + inside_track_dist / outside_track_dist)

    def _calc_yaw_speed(self, signed_turn_radius: float) -> float:
        abs_turn_radius = abs(signed_turn_radius)
        half_track_width = self._robot_config.track_width / 2
        outside_track_dist = abs_turn_radius + half_track_width
        return (
            self._reference_speed
            / outside_track_dist
            * geometry.sign(signed_turn_radius)
        )

    def _corrective_twist(
        self, pose_diff: geometry.Pose, driving_direction: Literal[-1, 0, 1]
    ) -> geometry.Twist:
        speed_error = _X_POS_GAIN * pose_diff.position.x
        corrective_vel = geometry.Velocity.unit_x(geometry.BODY) * speed_error
        spin_speed = (
            _Y_POS_GAIN * pose_diff.position.y * driving_direction
            + _THETA_ANGLE_GAIN * math.sin(pose_diff.orientation.yaw)
        )
        corrective_spin = geometry.AngularVelocity(geometry.BODY, 0, 0, spin_speed)

        return geometry.Twist(corrective_vel, corrective_spin)
