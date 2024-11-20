import asyncio
import collections
import math
import os
import sys
import time
from typing import DefaultDict, Literal, Optional

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
from config import robot_config
from controls import follow_path_controller
from ipc import messages, registry, session
from planning import navigation_path_planner, navigation_progress_tracker, primitives

from node import base_node

_CONTROL_RATE = 50  # In Hz


class NavigationServer(base_node.BaseNode):
    """Runs and servers any navigation request."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.NAVIGATION)
        # No obstacles added for now.
        self._nav_planner = navigation_path_planner.NavigationPathPlanner([])
        self._request: Optional[messages.NavigateRequest] = None
        self._cur_pose: geometry.Pose
        self._nav_progress_tracker: navigation_progress_tracker.NavigationProgressTracker
        self._controller: follow_path_controller.FollowPathController
        self._motors = session.get_robot_motors()
        self._robot_config = session.get_robot_config()
        self._motor_velocities: DefaultDict[
            robot_config.DrivetrainLocation, float
        ] = collections.defaultdict(lambda: 0.0)

        self.add_subscribers(
            {
                registry.Channels.BODY_KINEMATICS: self._update_body_kinematics_state,
                registry.Channels.FRONT_LEFT_MOTOR_VELOCITY: self._update_motor_velocity,
                registry.Channels.FRONT_RIGHT_MOTOR_VELOCITY: self._update_motor_velocity,
                registry.Channels.REAR_LEFT_MOTOR_VELOCITY: self._update_motor_velocity,
                registry.Channels.REAR_RIGHT_MOTOR_VELOCITY: self._update_motor_velocity,
            }
        )
        self.add_publishers(
            registry.Channels.FRONT_LEFT_MOTOR_CMD,
            registry.Channels.FRONT_RIGHT_MOTOR_CMD,
            registry.Channels.REAR_LEFT_MOTOR_CMD,
            registry.Channels.REAR_RIGHT_MOTOR_CMD,
        )
        self.set_request_server(registry.Requests.NAVIGATE, self._rcv_request)

    async def _rcv_request(
        self, request: messages.NavigateRequest
    ) -> Literal["success", "fail"]:
        self._request = request
        self._path = self._nav_planner.gen_path(self._cur_pose, self._request.target)
        self._controller = follow_path_controller.FollowPathController(
            session.get_robot_config()
        )
        try:
            await self._run_control_loop()
        finally:
            self._request = None
            self._stop_motors()

        return "success" if self._finished() else "fail"

    def _update_body_kinematics_state(
        self, msg: messages.VehicleKinematicsMessage
    ) -> None:
        self._cur_pose = msg.pose
        self._cur_twist = msg.twist

    def _update_motor_velocity(self, msg: messages.MotorVelocityMessage) -> None:
        self._motor_velocities[msg.motor.location] = msg.estimated_velocity

    async def _run_control_loop(self) -> None:
        total_time = 1.0 / _CONTROL_RATE
        while self._request is not None:
            start_time = time.perf_counter()

            self._check_and_replan()
            self._update_controller()
            for motor in self._motors:
                self._publish_motor_cmd_msg(motor)

            write_time = time.perf_counter() - start_time
            sleep_time = total_time - write_time
            await asyncio.sleep(sleep_time)

    def _publish_motor_cmd_msg(self, motor: robot_config.Motor) -> None:
        velocity = self._controller.velocity(motor)
        msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
        channel = registry.motor_command_channel(motor)
        self.publish(channel, msg)

    def _check_and_replan(self) -> None:
        """Verifies if a replan is necessary and replans."""
        pass

    def _update_controller(self) -> None:
        self._nav_progress_tracker.update(self._cur_pose)
        navpoint = self._nav_progress_tracker.current_navpoint
        reference_speed = self._nav_progress_tracker.reference_speed_along_path()
        target = self._target_twist(navpoint, reference_speed)
        self._controller.update(target, self._cur_twist)

    def _target_twist(
        self, navpoint: primitives.NavigationPoint, reference_speed: float
    ) -> geometry.Twist:
        """Twist to target for the robot."""
        ref_twist = self._reference_twist(navpoint, reference_speed)
        target_in_body = self._nav_progress_tracker.ref_pose_2d.transform(
            navpoint.pose_2d
        )
        corrective_twist = self._corrective_twist(
            target_in_body, geometry.sign(ref_twist.velocity.x)
        )
        return ref_twist + corrective_twist

    def _reference_twist(
        self, navpoint: primitives.NavigationPoint, reference_speed: float
    ) -> geometry.Twist:
        signed_turn_radius = navpoint.signed_turn_radius(self._path.turn_radius)
        direction = navpoint.pose_2d.orientation.x_axis()
        if signed_turn_radius:
            linear_speed = self._calc_linear_speed(signed_turn_radius, reference_speed)
            angular_vel = geometry.AngularVelocity(
                geometry.UTM,
                0,
                0,
                self._calc_yaw_speed(signed_turn_radius, reference_speed),
            )
        else:
            linear_speed = reference_speed
            angular_vel = geometry.AngularVelocity.zero(geometry.UTM)

        velocity = geometry.Velocity.from_direction(
            geometry.UTM, direction, linear_speed
        )
        return geometry.Twist(velocity, angular_vel)

    def _calc_linear_speed(
        self, signed_turn_radius: float, reference_speed: float
    ) -> float:
        abs_turn_radius = abs(signed_turn_radius)
        half_track_width = self._robot_config.track_width / 2
        inside_track_dist = abs_turn_radius - half_track_width
        outside_track_dist = abs_turn_radius + half_track_width
        return reference_speed / 2 * (1 + inside_track_dist / outside_track_dist)

    def _calc_yaw_speed(
        self, signed_turn_radius: float, reference_speed: float
    ) -> float:
        abs_turn_radius = abs(signed_turn_radius)
        half_track_width = self._robot_config.track_width / 2
        outside_track_dist = abs_turn_radius + half_track_width
        return reference_speed / outside_track_dist * geometry.sign(signed_turn_radius)

    def _corrective_twist(
        self, pose_diff: geometry.Pose, driving_direction: Literal[-1, 0, 1]
    ) -> geometry.Twist:
        _X_POS_GAIN = 0.001
        _Y_POS_GAIN = 22.9
        _THETA_ANGLE_GAIN = 22.9

        speed_error = _X_POS_GAIN * pose_diff.position.x
        corrective_vel = geometry.Velocity.unit_x(geometry.BODY) * speed_error
        spin_speed = (
            _Y_POS_GAIN * pose_diff.position.y * driving_direction
            + _THETA_ANGLE_GAIN * math.sin(pose_diff.orientation.yaw)
        )
        corrective_spin = geometry.AngularVelocity(geometry.BODY, 0, 0, spin_speed)

        return geometry.Twist(corrective_vel, corrective_spin)

    # def _motor_wheel_twist(self) -> geometry.Twist:
    #     """The twist in the robots body frame expressed as two floats of a linear and
    #     angular velocity.
    #     """
    #     # TODO: Break this out into a helper file with unit tests.
    #     front_left_vel = self._motor_velocities[
    #         robot_config.DrivetrainLocation.FRONT_LEFT
    #     ]
    #     front_right_vel = self._motor_velocities[
    #         robot_config.DrivetrainLocation.FRONT_RIGHT
    #     ]
    #     rear_left_vel = self._motor_velocities[
    #         robot_config.DrivetrainLocation.REAR_LEFT
    #     ]
    #     rear_right_vel = self._motor_velocities[
    #         robot_config.DrivetrainLocation.REAR_RIGHT
    #     ]

    #     left_vel = (front_left_vel + rear_left_vel) / 2
    #     right_vel = (front_right_vel + rear_right_vel) / 2

    #     linear_velocity = (right_vel - left_vel) / 2
    #     angular_velocity = (right_vel + left_vel) / 2

    #     return geometry.Twist(linear_velocity, angular_velocity)

    def _stop_motors(self) -> None:
        for motor in self._motors:
            msg = messages.MotorCommandMessage(motor=motor, velocity=0.0)
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)

    def _finished(self) -> bool:
        return False


if __name__ == "__main__":
    node = NavigationServer()
    node.start()
