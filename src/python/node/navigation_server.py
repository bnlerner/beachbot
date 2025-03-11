import asyncio
import os
import sys
from typing import Literal, Optional

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
import log
from controls import nav_cascade_controller
from ipc import messages, registry, session
from mapping import obstacle_map
from planning import nav_path_planner, nav_progress_tracker
from typing_helpers import req

from node import base_node

# Rates to update the controller and publishing in Hz.
_CONTROL_RATE = 50
_PUBLISH_RATE = 100


class NavigationServer(base_node.BaseNode):
    """Runs and servers any navigation request."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.NAVIGATION)

        self._obstacle_map = obstacle_map.ObstacleMap()
        self._nav_planner = nav_path_planner.NavPathPlanner(self._obstacle_map)
        self._request: Optional[messages.NavigateRequest] = None
        self._cur_pose: Optional[geometry.Pose] = None
        self._nav_progress_tracker: nav_progress_tracker.NavProgressTracker
        self._controller: nav_cascade_controller.NavCascadeController
        self._motors = session.get_robot_motors()
        self._robot_config = session.get_robot_config()

        self.add_subscribers(
            {
                registry.Channels.BODY_KINEMATICS: self._update_body_kinematics_state,
                registry.Channels.FRONT_OBSTACLES: self._update_obstacles,
                registry.Channels.REAR_OBSTACLES: self._update_obstacles,
            }
        )
        self.add_publishers(
            registry.Channels.MOTOR_CMD_FRONT_LEFT,
            registry.Channels.MOTOR_CMD_FRONT_RIGHT,
            registry.Channels.MOTOR_CMD_REAR_LEFT,
            registry.Channels.MOTOR_CMD_REAR_RIGHT,
        )
        self.set_request_server(registry.Requests.NAVIGATE, self._rcv_request)
        self.add_looped_tasks(
            {
                self._run_update_control_loop: _CONTROL_RATE,
                self._publish_motor_cmd_msgs: _PUBLISH_RATE,
            }
        )

    async def _rcv_request(
        self, request: messages.NavigateRequest
    ) -> Literal["success", "fail"]:
        await self._wait_for_kinematics()
        self._initialize_planning_and_control(request)
        self._request = request
        try:
            await self._wait_for_nav_finished()
        finally:
            self._request = None
            self._stop_motors()

        return "success" if self._nav_progress_tracker.is_finished() else "fail"

    def _update_body_kinematics_state(
        self, msg: messages.VehicleKinematicsMessage
    ) -> None:
        self._cur_pose = msg.pose
        self._cur_twist = msg.twist

    def _update_obstacles(self, msg: messages.TrackedObjectsMessage) -> None:
        # TODO: Transform into map?
        self._obstacle_map.update(msg.objects)

    async def _wait_for_kinematics(self) -> None:
        while self._cur_pose is None:
            log.info("No valid kinematic pose.")
            await asyncio.sleep(2)

    async def _wait_for_nav_finished(self) -> None:
        while self._request is not None:
            await asyncio.sleep(0.1)

    def _run_update_control_loop(self) -> None:
        if not self._is_active_request():
            return

        if self._nav_progress_tracker.is_finished():
            self._request = None
            return

        if self._nav_progress_tracker.is_ready():
            self._check_and_replan()

        self._update_controller()

    def _initialize_planning_and_control(
        self, request: messages.NavigateRequest
    ) -> None:
        self._path = self._nav_planner.gen_path(req(self._cur_pose), request.target)
        self._nav_progress_tracker = nav_progress_tracker.NavProgressTracker(self._path)
        self._controller = nav_cascade_controller.NavCascadeController(
            self._motors, self._robot_config
        )

    def _publish_motor_cmd_msgs(self) -> None:
        if not self._is_active_request() or not self._nav_progress_tracker.is_ready():
            return

        # Reset each motor integral if we reach a cusp point.
        reset_integral = self._nav_progress_tracker.current_navpoint.is_cusp_point
        feedforward_torque = self._controller.feedforward_torque()
        for motor in self._motors:
            velocity = self._controller.velocity(motor)
            msg = messages.MotorCommandMessage(
                motor=motor,
                velocity=velocity,
                reset_integral=reset_integral,
                feedforward_torque=feedforward_torque,
            )
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)

    def _is_active_request(self) -> bool:
        return self._request is not None

    def _check_and_replan(self) -> None:
        """Verifies if a replan is necessary and replans."""
        if self._nav_progress_tracker.is_off_nav_path():
            log.error(
                "Too far away from nav path \n"
                f"\t{self._cur_pose=}\n\t{self._cur_twist=}\n"
                f"\t{self._path.end=}\n\t{self._path.start=}"
            )
            # TODO: Eventually add a replanning step but for now we stop the nav if its
            # far off target.
            self._request = None

    def _update_controller(self) -> None:
        self._nav_progress_tracker.update(req(self._cur_pose))
        navpoint = self._nav_progress_tracker.current_navpoint
        reference_speed = self._nav_progress_tracker.reference_speed_along_path()
        cur_pose = self._nav_progress_tracker.ref_pose_2d
        self._controller.update(navpoint, cur_pose, reference_speed, self._cur_twist)

    def _stop_motors(self) -> None:
        for motor in self._motors:
            msg = messages.MotorCommandMessage(motor=motor, velocity=0.0)
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)


if __name__ == "__main__":
    node = NavigationServer()
    node.start()
