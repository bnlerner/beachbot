import asyncio
import os
import sys
import time
from typing import DefaultDict, Literal, Optional, Tuple

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import collections

import log
from config import robot_config
from controls import follow_path_controller
from ipc import messages, registry, session
from planning import navigation_path_planner, primitives

from node import base_node

_CONTROL_RATE = 50  # In Hz


class NavigationServer(base_node.BaseNode):
    """Runs and servers any navigation request."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.NAVIGATION)
        # No obstacles added for now.
        self._nav_planner = navigation_path_planner.NavigationPathPlanner([])
        self._request: Optional[messages.NavigateRequest] = None
        self._cur_gps_point: primitives.GPSPoint
        self._cur_heading: float
        self._controller: follow_path_controller.FollowPathController
        self._motors = session.get_robot_motors()
        self._motor_velocities: DefaultDict[
            robot_config.DrivetrainLocation, float
        ] = collections.defaultdict(lambda: 0.0)

        self.add_subscribers(
            {
                registry.Channels.BODY_GPS: self._update_gps_state,
                registry.Channels.BODY_DYNAMICS: self._update_body_dyn_state,
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
        self._path = self._nav_planner.gen_path(
            self._cur_gps_point, self._cur_heading, self._request.target
        )
        self._controller = follow_path_controller.FollowPathController(
            self._path, session.get_robot_config()
        )
        try:
            await self._run_control_loop()
        finally:
            self._request = None
            self._stop_motors()

        return "success" if self._finished() else "fail"

    def _update_gps_state(self, msg: messages.GPSMessage) -> None:
        self._cur_gps_point = primitives.GPSPoint(
            latitude=msg.latitude, longitude=msg.longitude
        )

    def _update_body_dyn_state(self, msg: messages.VehicleDynamicsMessage) -> None:
        self._cur_heading = msg.heading

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
        target = self._target_twist()
        measured = self._cur_twist()
        self._controller.update(target, measured)

    def _target_twist(self) -> Tuple[float, float]:
        return 1.0, 0.0

    def _cur_twist(self) -> Tuple[float, float]:
        """The twist in the robots body frame expressed as two floats of a linear and
        angular velocity.
        """
        # TODO: Break this out into a helper file with unit tests.
        front_left_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.FRONT_LEFT
        ]
        front_right_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.FRONT_RIGHT
        ]
        rear_left_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.REAR_LEFT
        ]
        rear_right_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.REAR_RIGHT
        ]

        left_vel = (front_left_vel + rear_left_vel) / 2
        right_vel = (front_right_vel + rear_right_vel) / 2

        linear_velocity = (right_vel - left_vel) / 2
        angular_velocity = (right_vel + left_vel) / 2

        log.data(linear_velocity=linear_velocity, angular_velocity=angular_velocity)
        return linear_velocity, angular_velocity

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
