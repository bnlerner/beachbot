import asyncio
import os
import sys
import time
from typing import Any, Dict, Optional

from starlette import requests, responses, routing
from starlette.staticfiles import StaticFiles

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
import system_info
from controls import motor_velocity_controller
from ipc import core, messages, registry, session
from localization import gps_transformer, primitives
from models import constants

from node import base_node

# UTM zone for Florida is zone number 17N.
_DEFAULT_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)
_STATIC_RESOURCE_PATH = system_info.get_root_project_directory() / "env" / "static"
_CLIENT_TIMEOUT = 10  # timeout in seconds
_CONTROL_RATE = 100  # In Hz
_RC_TIMEOUT = 0.5  # Seconds


class UINode(base_node.BaseNode):
    """A node to host the UI of beachbot and allow interface to its servers and actions.
    Does not use the base node because flask has issues running using async.
    """

    def __init__(self, port: int, *, debug: bool = False) -> None:
        super().__init__(registry.NodeIDs.UI)
        self._port = port
        self._debug = debug
        self._gps_transformer = gps_transformer.GPSTransformer(_DEFAULT_UTM_ZONE)
        self._motor_configs = session.get_robot_motors()
        self._controller = motor_velocity_controller.MotorVelocityController(
            session.get_robot_motors(), session.get_robot_config()
        )
        self._rc_watchdog_ts = time.perf_counter()
        self._nav_task: Optional[asyncio.Task] = None
        self._client_sessions: Dict[str, float] = {}
        self._cur_tab: Optional[str] = None

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
            routing.Route("/start-session", self._start_session, methods=["POST"]),
            # Mount the static files directory
            routing.Mount(
                "/",
                StaticFiles(directory=_STATIC_RESOURCE_PATH, html=True),
                name="static",
            ),
        ]
        self.set_http_server(self._port, routes)
        self.add_looped_tasks({self._publish_rc_cmd_msgs: _CONTROL_RATE})

    async def _tab_switch(self, request: requests.Request) -> responses.Response:
        if (data := await self._extract_data_from_request(request)) is None:
            return responses.JSONResponse({"status": "Invalid client request."}, 409)

        self._cur_tab = self._get_tab_name(data)
        log.info(f"Switched to tab: {self._cur_tab}")
        self._cancel_nav_if_on_wrong_tab()
        # You can perform additional actions here based on the tab change
        return responses.JSONResponse({"status": "success", "tab": self._cur_tab}, 200)

    async def _rc_input(self, request: requests.Request) -> responses.Response:
        if (data := await self._extract_data_from_request(request)) is None:
            return responses.JSONResponse({"status": "Invalid client request."}, 409)

        self._update_rc_controller(data)
        return responses.JSONResponse(
            {"status": "success", "message": "Data received"}, 201
        )

    async def _navigate(self, request: requests.Request) -> responses.Response:
        if (data := await self._extract_data_from_request(request)) is None:
            return responses.JSONResponse({"status": "Invalid client request."}, 409)

        if request.method == "POST":
            return await self._exec_navigate_action(data)
        elif request.method == "DELETE":
            if self._nav_task is not None:
                self._nav_task.cancel()
                return responses.JSONResponse({"status": "Navigation cancelled."}, 200)
            else:
                return responses.JSONResponse(
                    {"status": "No active navigation request"}, 404
                )
        else:
            raise ValueError(f"Unexpected request {request=}")

    async def _e_stop(self, request: requests.Request) -> responses.Response:
        if await self._extract_data_from_request(request) is None:
            return responses.JSONResponse({"status": "Invalid client request."}, 409)

        self._publish_e_stop()
        log.info("Emergency Stop triggered!")
        return responses.JSONResponse({"status": "E-Stop activated"}, 201)

    async def _keep_alive(self, request: requests.Request) -> responses.Response:
        if (data := await request.json()) is None or (
            token := data.get("token")
        ) is None:
            return responses.JSONResponse({"status": "invalid keep alive request"}, 400)

        if self._active_client() == token or self._is_active_client_expired():
            self._client_sessions[token] = time.perf_counter()
            return responses.JSONResponse({"status": "alive"}, 200)
        else:
            return responses.JSONResponse({"status": "multiple"}, 200)

    async def _start_session(self, request: requests.Request) -> responses.Response:
        if (data := await request.json()) is None or (
            token := data.get("token")
        ) is None:
            return responses.JSONResponse({"status": "Token missing"}, 400)

        # Store the token with a timestamp (or user association)
        self._client_sessions[token] = time.perf_counter()
        return responses.JSONResponse(
            {"status": "success", "message": "Session started"}, 201
        )

    def _update_rc_controller(self, data: Dict[str, str]) -> None:
        # Positive X is turn right
        x: float = eval(x_str) if (x_str := data.get("x")) else 0.0

        # Positive Y is forward
        y: float = eval(y_str) if (y_str := data.get("y")) else 0.0

        # NOTE: Order not guaranteed, ensures that we stop before moving again.
        stop_robot = bool(data.get("stop")) or False
        if not isinstance(stop_robot, bool):
            raise ValueError(f"Unexpected stop robot type {stop_robot=}")

        linear_velocity = 0.0 if stop_robot else constants.MAX_LINEAR_SPEED * y * 0.5
        # Negative since positive spin is to the left.
        angular_velocity = 0.0 if stop_robot else constants.MAX_ANGULAR_SPEED * -x
        # Reset the watchdog timer
        self._rc_watchdog_ts = time.perf_counter()
        self._controller.set_target(linear_velocity, angular_velocity)

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
        # Only publish RC commands if on the RC tab.
        if self._cur_tab != "rc_tab":
            return None

        # Checks if the RC commands are expired and ensures that the target velocities
        # are set to zero.
        if self._is_rc_expired():
            self._controller.set_target(0.0, 0.0)

        for motor in self._motor_configs:
            msg = messages.MotorCommandMessage(
                motor=motor,
                velocity=self._controller.calc_wheel_speed(motor),
                feedforward_torque=self._controller.calc_feedforward_torque(),
            )
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)

    def _publish_e_stop(self) -> None:
        msg = messages.StopMotorsMessage()
        self.publish(registry.Channels.STOP_MOTORS, msg)

    async def _exec_navigate_action(self, data: Dict[str, float]) -> responses.Response:
        if (nav_request := self._gen_navigate_request(data)) is None:
            return responses.JSONResponse(
                {"status": "failed", "message": "No Latitude or longitude received"},
                400,
            )

        if self._nav_task is not None and not self._nav_task.done():
            return responses.JSONResponse(
                {"status": "failed", "message": "Nav task running."}, 409
            )

        try:
            return_msg = await self._navigate_to_target(nav_request)
        except asyncio.CancelledError:
            return responses.JSONResponse(
                {"status": "failed", "message": "Request Cancelled"}, 200
            )
        else:
            if return_msg is not None:
                return responses.JSONResponse(
                    {"status": "failed", "message": return_msg}, 500
                )
            else:
                return responses.JSONResponse(
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

    def _active_client(self) -> Optional[str]:
        if len(self._client_sessions) > 0:
            return max(self._client_sessions, key=lambda x: self._client_sessions[x])
        else:
            return None

    def _is_active_client_expired(self) -> bool:
        if active_client := self._active_client():
            return (
                self._client_sessions[active_client] + _CLIENT_TIMEOUT
                < time.perf_counter()
            )
        else:
            # No client so its expired by default.
            return True

    async def _extract_data_from_request(
        self, request: requests.Request
    ) -> Optional[Dict[str, Any]]:
        if (data := await request.json()) is None:
            return None

        if (token := data.get("token")) is None:
            return None

        if self._active_client() != token or self._is_active_client_expired():
            return None

        return data

    def _cancel_nav_if_on_wrong_tab(self) -> None:
        if self._cur_tab == "rc_tab":
            if self._nav_task is not None:
                # Cancel nav request
                log.info("Cancelling existing nav task to allow RC.")
                self._nav_task.cancel()
        elif self._cur_tab == "home":
            if self._nav_task is not None:
                # Cancel nav request
                log.info("Cancelling existing nav task since we are in the wrong tab.")
                self._nav_task.cancel()
        elif self._cur_tab == "autonomous_tab":
            if self._nav_task is None:
                self._controller.set_target(0.0, 0.0)
            else:
                # If a nav task exists when switching tabs cancel it. Should not happen.
                log.error(
                    "Unexpected state switching to autonomous tab with existing nav task."
                )
                self._nav_task.cancel()
        else:
            raise ValueError(f"Unexpected tab name {self._cur_tab=}")

    def _is_rc_expired(self) -> bool:
        """Indicates the RC commands have expired if the watchdog timestamp exceeds the
        timeout.
        """
        return time.perf_counter() - self._rc_watchdog_ts > _RC_TIMEOUT

    async def shutdown_hook(self) -> None:
        if self._nav_task is not None:
            self._nav_task.cancel()
            self._nav_task = None


if __name__ == "__main__":
    node = UINode(5000)
    node.start()
