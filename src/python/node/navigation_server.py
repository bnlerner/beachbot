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
from planning import nav_path_planner, nav_progress_tracker
from typing_helpers import req

from node import base_node

_CONTROL_RATE = 50  # In Hz


class NavigationServer(base_node.BaseNode):
    """Runs and servers any navigation request."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.NAVIGATION)
        # No obstacles added for now.
        self._nav_planner = nav_path_planner.NavPathPlanner([])
        self._request: Optional[messages.NavigateRequest] = None
        self._cur_pose: Optional[geometry.Pose] = None
        self._nav_progress_tracker: nav_progress_tracker.NavProgressTracker
        self._controller: nav_cascade_controller.NavCascadeController
        self._motors = session.get_robot_motors()
        self._robot_config = session.get_robot_config()

        self.add_subscribers(
            {registry.Channels.BODY_KINEMATICS: self._update_body_kinematics_state}
        )
        self.add_publishers(
            registry.Channels.MOTOR_CMD_FRONT_LEFT,
            registry.Channels.MOTOR_CMD_FRONT_RIGHT,
            registry.Channels.MOTOR_CMD_REAR_LEFT,
            registry.Channels.MOTOR_CMD_REAR_RIGHT,
        )
        self.set_request_server(registry.Requests.NAVIGATE, self._rcv_request)
        self.add_looped_tasks({self._run_control_loop: _CONTROL_RATE})

    async def _rcv_request(
        self, request: messages.NavigateRequest
    ) -> Literal["success", "fail"]:
        await self._wait_for_kinematics()
        self._path = self._nav_planner.gen_path(req(self._cur_pose), request.target)
        self._nav_progress_tracker = nav_progress_tracker.NavProgressTracker(self._path)
        self._controller = nav_cascade_controller.NavCascadeController(
            self._robot_config
        )
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

    async def _wait_for_kinematics(self) -> None:
        while self._cur_pose is None:
            log.info("No valid kinematic pose.")
            await asyncio.sleep(2)

    async def _wait_for_nav_finished(self) -> None:
        while self._request is not None:
            await asyncio.sleep(0.2)

    def _run_control_loop(self) -> None:
        if self._request is None:
            return

        if self._nav_progress_tracker.is_finished():
            self._request = None
            return

        self._check_and_replan()
        self._update_controller()
        self._publish_motor_cmd_msgs()

    def _publish_motor_cmd_msgs(self) -> None:
        # Reset each motor integral if we reach a cusp point.
        reset_integral = self._nav_progress_tracker.current_navpoint.is_cusp_point
        for motor in self._motors:
            # TODO: how to get torque and velocity?
            velocity = self._controller.velocity(motor)
            msg = messages.MotorCommandMessage(
                motor=motor, velocity=velocity, reset_integral=reset_integral
            )
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)

    def _check_and_replan(self) -> None:
        """Verifies if a replan is necessary and replans."""
        pass

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
