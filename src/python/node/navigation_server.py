import asyncio
import os
import sys
import time
from typing import Optional, Tuple

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

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

        self.add_subscribers(
            {
                registry.Channels.BODY_GPS: self._update_gps_state,
                registry.Channels.BODY_DYNAMICS: self._update_body_dyn_state,
            }
        )
        self.add_publishers(
            registry.Channels.FRONT_LEFT_MOTOR_CMD,
            registry.Channels.FRONT_RIGHT_MOTOR_CMD,
            registry.Channels.REAR_LEFT_MOTOR_CMD,
            registry.Channels.REAR_RIGHT_MOTOR_CMD,
        )
        self.set_request_server(registry.Requests.NAVIGATE, self._rcv_request)

    async def _rcv_request(self, request: messages.NavigateRequest) -> None:
        self._request = request
        self._path = self._nav_planner.gen_path(
            self._cur_nav_point(), self._request.target
        )
        self._controller = follow_path_controller.FollowPathController(
            self._path, session.get_robot_config()
        )
        try:
            await self._run_control_loop()
        finally:
            self._request = None

    def _update_gps_state(self, msg: messages.GPSMessage) -> None:
        self._cur_gps_point = primitives.GPSPoint(
            latitude=msg.latitude, longitude=msg.longitude
        )

    def _update_body_dyn_state(self, msg: messages.VehicleDynamicsMessage) -> None:
        self._cur_vel = ...
        self._cur_heading = msg.heading

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
        channel = registry.motor_channel(motor)
        self.publish(channel, msg)

    def _check_and_replan(self) -> None:
        """Verifies if a replan is necessary and replans."""
        pass

    def _update_controller(self) -> None:
        target = ...
        measured = self._cur_twist()
        self._controller.update()

    def _cur_nav_point(self) -> primitives.NavigationPoint:
        return primitives.NavigationPoint(
            point=self._cur_gps_point, yaw=self._cur_heading
        )

    def _cur_twist(self) -> Tuple[float, float]:
        ...


if __name__ == "__main__":
    node = NavigationServer()
    node.start()
