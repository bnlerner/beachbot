import asyncio
import os
import sys
import time
from typing import Dict, Optional

import uvicorn
from starlette import requests, responses, routing
from starlette.staticfiles import StaticFiles

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
from ipc import core, messages, registry, session
from localization import gps_transformer, primitives
from models import body_model, constants

from node import base_node

# UTM zone for Florida is zone number 17N.
_DEFAULT_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)


class UINode(base_node.BaseNode):
    """A node to host the UI of beachbot and allow interface to its servers and actions.
    Does not use the base node because flask has issues running using async.
    """

    def __init__(self, port: int, *, debug: bool = False) -> None:
        self._node_id = registry.NodeIDs.UI
        self._port = port
        self._debug = debug
        self._gps_transformer = gps_transformer.GPSTransformer(_DEFAULT_UTM_ZONE)
        self._motor_configs = session.get_robot_motors()
        self._rc_controller = body_model.BodyModel(session.get_robot_config())
        self._stop_robot: bool = True
        self._nav_task: Optional[asyncio.Task] = None
        self._client_connections: Dict[str, float] = {}
        self._http_server: Optional[uvicorn.Server] = None

        self.add_publishers(
            registry.Channels.MOTOR_CMD_FRONT_LEFT,
            registry.Channels.MOTOR_CMD_FRONT_RIGHT,
            registry.Channels.MOTOR_CMD_REAR_LEFT,
            registry.Channels.MOTOR_CMD_REAR_RIGHT,
            registry.Channels.STOP_MOTORS,
        )
        self.add_request_clients(registry.Requests.NAVIGATE)

        routes = [
            routing.Route("/tab_switch", self._tab_switch, methods=["POST"]),
            routing.Route("/joystick_input", self._rc_input, methods=["POST"]),
            routing.Route("/navigate", self._navigate, methods=["POST", "DELETE"]),
            routing.Route("/e-stop", self._e_stop, methods=["POST"]),
            routing.Route("/keep-alive", self._keep_alive, methods=["POST"]),
            # Mount the static files directory
            routing.Mount(
                "/", StaticFiles(directory="static", html=True), name="static"
            ),
        ]
        self.set_http_server(self._port, routes)

    async def _tab_switch(self, request: requests.Request) -> responses.Response:
        if (data := await request.json()) is None:
            return responses.Response(
                {"status": "failed", "message": "Invalid JSON"}, 400
            )

        tab_name = self._get_tab_name(data)
        log.info(f"Switched to tab: {tab_name}")
        self._cancel_nav_if_on_wrong_tab(tab_name)
        # You can perform additional actions here based on the tab change
        return responses.Response({"status": "success", "tab": tab_name}, 200)

    async def _rc_input(self, request: requests.Request) -> responses.Response:
        # Capture joystick input from POST request to RC
        if (data := await request.json()) is None:
            return responses.Response(
                {"status": "failed", "message": "Invalid JSON"}, 400
            )

        self._update_rc_controller(data)
        self._publish_rc_cmd_msgs()
        return responses.Response(
            {"status": "success", "message": "Data received"}, 201
        )

    async def _navigate(self, request: requests.Request) -> responses.Response:
        if request.method == "POST":
            if (data := await request.json()) is None:
                return responses.Response(
                    {"status": "failed", "message": "Invalid JSON"}, 400
                )

            return await self._exec_navigate_action(data)
        elif request.method == "DELETE":
            if self._nav_task is not None:
                self._nav_task.cancel()
                return responses.Response({"status": "Navigation cancelled."}, 200)
            else:
                return responses.Response(
                    {"status": "No active navigation request"}, 404
                )
        else:
            raise ValueError(f"Unexpected request {request=}")

    def _e_stop(self, _: requests.Request) -> responses.Response:
        self._publish_e_stop()
        log.info("Emergency Stop triggered!")
        return responses.Response({"status": "E-Stop activated"}, 201)

    def _keep_alive(self, request: requests.Request) -> responses.Response:
        if client_ip := request.client:
            self._client_connections[client_ip.host] = time.perf_counter()

        return responses.Response({"status": "alive"}, 201)

    def _update_rc_controller(self, data: Dict[str, str]) -> None:
        # Positive X is turn right
        x: float = eval(x_str) if (x_str := data.get("x")) else 0.0

        # Positive Y is forward
        y: float = eval(y_str) if (y_str := data.get("y")) else 0.0

        # NOTE: Order not guaranteed, ensures that we stop before moving again.
        self._stop_robot = bool(data.get("stop")) or False
        if not isinstance(self._stop_robot, bool):
            raise ValueError(f"Unexpected stop robot type {self._stop_robot=}")

        linear_velocity = constants.MAX_LINEAR_SPEED * y * 0.5
        # Negative since positive spin is to the left.
        angular_velocity = constants.MAX_ANGULAR_SPEED * -x
        self._rc_controller.update(linear_velocity, angular_velocity)

    async def _navigate_to_target(self, nav_request: core.Request) -> Optional[str]:
        """Navigates to the target, returning a message if it does not sucdeed."""
        self._nav_task = asyncio.create_task(
            self.send_request(registry.Requests.NAVIGATE, nav_request)
        )
        response = await self._nav_task

        self._nav_task = None

        if response.cancelled:
            return "Navigation was cancelled."
        elif response.result == "fail":
            return "Failed to navigate to the target."
        elif response.result == "success":
            return None
        else:
            raise ValueError(f"Unexpected response. {response=}")

    def _publish_rc_cmd_msgs(self) -> None:
        for motor in self._motor_configs:
            velocity = 0.0 if self._stop_robot else self._rc_controller.velocity(motor)
            # Sets the integrator torque to zero during a stop event.
            msg = messages.MotorCommandMessage(
                motor=motor, velocity=velocity, reset_integral=self._stop_robot
            )
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)

    def _publish_e_stop(self) -> None:
        msg = messages.StopMotorsMessage()
        self.publish(registry.Channels.STOP_MOTORS, msg)

    async def _exec_navigate_action(self, data: Dict[str, float]) -> responses.Response:
        if (nav_request := self._gen_navigate_request(data)) is None:
            return responses.Response(
                {"status": "failed", "message": "No Latitude or longitude received"},
                400,
            )

        if self._nav_task is not None and not self._nav_task.done():
            return responses.Response(
                {"status": "failed", "message": "Nav task running."}, 409
            )

        try:
            return_msg = await self._navigate_to_target(nav_request)
        except asyncio.CancelledError:
            return responses.Response(
                {"status": "failed", "message": "Request Cancelled"}, 200
            )
        else:
            if return_msg is not None:
                return responses.Response(
                    {"status": "failed", "message": return_msg}, 500
                )
            else:
                return responses.Response(
                    {"status": "success", "message": "finished navigating."}, 201
                )

    def _gen_navigate_request(
        self, data: Dict[str, float]
    ) -> Optional[messages.NavigateRequest]:
        if (latitude := data.get("latitude")) and (longitude := data.get("longitude")):
            altitude = data.get("altitude")
            # Process the GPS data here (store it, log it, etc.)
            log.info(
                f"Received GPS data: Latitude = {latitude}, Longitude = {longitude}, altitude = {altitude}"
            )
            position = self._gps_transformer.transform_position(
                longitude, latitude, height_above_sea_level=altitude
            )
            return messages.NavigateRequest(target=position)
        else:
            return None

    def _get_tab_name(self, data: Dict[str, str]) -> str:
        if tab_name := data.get("tab"):
            return tab_name
        else:
            raise ValueError(f"Expected tab key in data {data=}")

    def _cancel_nav_if_on_wrong_tab(self, tab_name: str) -> None:
        if tab_name == "rc_tab":
            if self._nav_task is not None:
                # cancel request
                log.info("Cancelling existing nav task to allow RC.")
                self._nav_task.cancel()
        elif tab_name == "autonomous_tab":
            if self._nav_task is None:
                # stop publishing RC and publish zero?
                self._rc_controller.update(0.0, 0.0)
            else:
                # If a nav task exists when switching tabs cancel it. Should not happen.
                log.error(
                    "Unexpected state switching to autonomous tab with existing nav task."
                )
                self._nav_task.cancel()
        else:
            raise ValueError(f"Unexpected tab name {tab_name=}")

    async def shutdown_hook(self) -> None:
        if self._nav_task is not None:
            self._nav_task.cancel()
            self._nav_task = None


if __name__ == "__main__":
    node = UINode(5000)
    node.start()
