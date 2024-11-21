import asyncio
import os
import sys
from typing import Dict, Optional, Tuple

from flask import Flask, Response, jsonify, render_template, request

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
from ipc import core, messages, pubsub, registry, session
from ipc import request as ipc_request
from localization import gps_transformer, primitives
from models import constants, motor_velocity_model

_INDEX_PATH = "index.html"
_RC_ENDPOINT = "/joystick_input"
_NAVIGATE_ENDPOINT = "/navigate"
_CERT_PATH = "env/auth/cert.pem"
_KEY_PATH = "env/auth/key.pem"
# UTM zone for Florida is zone number 17N.
_DEFAULT_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)


class UINode:
    """A node to host the UI of beachbot and allow interface to its servers and actions.
    Does not use the base node because flask has issues running using async.
    """

    def __init__(self, host: str, port: int, *, debug: bool = False) -> None:
        self._node_id = registry.NodeIDs.UI
        self._host = host
        self._port = port
        self._debug = debug
        self._gps_transformer = gps_transformer.GPSTransformer(_DEFAULT_UTM_ZONE)

        self._motor_configs = session.get_robot_motors()
        self._rc_controller = motor_velocity_model.MotorVelocityModel(
            session.get_robot_config()
        )
        self._app = Flask(__name__)
        self._setup_routes()
        self._stop_robot: bool = True
        self._publishers: Dict[core.ChannelSpec, pubsub.Publisher] = {}
        self._request_clients: Dict[core.RequestSpec, ipc_request.RequestClient] = {}
        self._nav_task: Optional[asyncio.Task] = None

        self._add_rc_cmd_publishers()
        self._add_request_clients(registry.Requests.NAVIGATE)

    def start(self) -> None:
        """Starts the UI node."""
        try:
            self._app.run(
                host=self._host,
                port=self._port,
                debug=self._debug,
                ssl_context=(_CERT_PATH, _KEY_PATH),
            )
        finally:
            self._clean_up_tasks()

    def _add_rc_cmd_publishers(self) -> None:
        self._publishers[registry.Channels.MOTOR_CMD_FRONT_LEFT] = pubsub.Publisher(
            self._node_id, registry.Channels.MOTOR_CMD_FRONT_LEFT
        )
        self._publishers[registry.Channels.MOTOR_CMD_FRONT_RIGHT] = pubsub.Publisher(
            self._node_id, registry.Channels.MOTOR_CMD_FRONT_RIGHT
        )
        self._publishers[registry.Channels.MOTOR_CMD_REAR_LEFT] = pubsub.Publisher(
            self._node_id, registry.Channels.MOTOR_CMD_REAR_LEFT
        )
        self._publishers[registry.Channels.MOTOR_CMD_REAR_RIGHT] = pubsub.Publisher(
            self._node_id, registry.Channels.MOTOR_CMD_REAR_RIGHT
        )
        self._publishers[registry.Channels.STOP_MOTORS] = pubsub.Publisher(
            self._node_id, registry.Channels.STOP_MOTORS
        )

    def _add_request_clients(self, *request_specs: core.RequestSpec) -> None:
        for request_spec in request_specs:
            self._request_clients[request_spec] = ipc_request.RequestClient(
                self._node_id, request_spec
            )

    def _setup_routes(self) -> None:
        """Setups all routes"""

        @self._app.route("/")
        def index() -> str:
            return render_template(_INDEX_PATH)

        @self._app.route("/tab_switch", methods=["POST"])
        def tab_switch() -> Tuple[Response, int]:
            if (data := request.json) is None:
                return jsonify({"status": "failed", "message": "Invalid JSON"}), 400

            tab_name = self._get_tab_name(data)
            log.info(f"Switched to tab: {tab_name}")
            self._cancel_nav_if_on_wrong_tab(tab_name)
            # You can perform additional actions here based on the tab change
            return jsonify({"status": "success", "tab": tab_name}), 200

        @self._app.route(_RC_ENDPOINT, methods=["POST"])
        def rc_input() -> Tuple[Response, int]:
            # Capture joystick input from POST request to RC
            if (data := request.json) is None:
                return jsonify({"status": "failed", "message": "Invalid JSON"}), 400

            self._update_rc_controller(data)
            self._publish_rc_cmd_msgs()
            return jsonify({"status": "success", "message": "Data received"}), 201

        @self._app.route(_NAVIGATE_ENDPOINT, methods=["POST", "DELETE"])
        async def navigate() -> Tuple[Response, int]:
            if request.method == "POST":
                if (data := request.json) is None:
                    return jsonify({"status": "failed", "message": "Invalid JSON"}), 400

                return await self._exec_navigate_action(data)
            elif request.method == "DELETE":
                if self._nav_task is not None:
                    self._nav_task.cancel()
                    return jsonify({"status": "Navigation cancelled."}), 200
                else:
                    return jsonify({"status": "No active navigation request"}), 404
            else:
                raise ValueError(f"Unexpected request {request=}")

        @self._app.route("/e-stop", methods=["POST"])
        def e_stop() -> Tuple[Response, int]:
            self._publish_e_stop()
            log.info("Emergency Stop triggered!")
            return jsonify({"status": "E-Stop activated"}), 201

    def _update_rc_controller(self, data: Dict[str, str]) -> None:
        # Positive X is turn right
        x: float = eval(x_str) if (x_str := data.get("x")) else 0.0

        # Positive Y is forward
        y: float = eval(y_str) if (y_str := data.get("y")) else 0.0

        # NOTE: Order not guaranteed, ensures that we stop before moving again.
        self._stop_robot = bool(data.get("stop")) or False
        if not isinstance(self._stop_robot, bool):
            raise ValueError(f"Unexpected stop robot type {self._stop_robot=}")

        linear_velocity = constants.MAX_LINEAR_SPEED * y
        # Negative since positive spin is to the left.
        angular_velocity = constants.MAX_ANGULAR_SPEED * -x
        self._rc_controller.update(linear_velocity, angular_velocity)

    async def _navigate_to_target(self, nav_request: core.Request) -> Optional[str]:
        """Navigates to the target, returning a message if it does not sucdeed."""
        self._nav_task = asyncio.create_task(
            self._send_request(registry.Requests.NAVIGATE, nav_request)
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
            msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
            channel = registry.motor_command_channel(motor)
            self._publishers[channel].publish(msg)

    def _publish_e_stop(self) -> None:
        msg = messages.StopMotorsMessage()
        self._publishers[registry.Channels.STOP_MOTORS].publish(msg)

    async def _send_request(
        self, spec: core.RequestSpec, request_msg: core.Request
    ) -> core.RequestResponse:
        if spec not in self._request_clients:
            raise RuntimeError(
                "Unrecognized Request spec, did you forget to add it? "
                f"{spec.base_channel.upper()}"
            )

        return await self._request_clients[spec].send(request_msg)

    async def _exec_navigate_action(
        self, data: Dict[str, float]
    ) -> Tuple[Response, int]:
        if (nav_request := self._gen_navigate_request(data)) is None:
            return jsonify(
                {"status": "failed", "message": "No Latitude or longitude received"}
            ), 400

        if self._nav_task is not None and not self._nav_task.done():
            return jsonify({"status": "failed", "message": "Nav task running."}), 409

        try:
            return_msg = await self._navigate_to_target(nav_request)
        except asyncio.CancelledError:
            return jsonify({"status": "failed", "message": "Request Cancelled"}), 200
        else:
            if return_msg is not None:
                return jsonify({"status": "failed", "message": return_msg}), 500
            else:
                return jsonify(
                    {"status": "success", "message": "finished navigating."}
                ), 201

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

    def _clean_up_tasks(self) -> None:
        if self._nav_task is not None:
            self._nav_task.cancel()

        for client in self._request_clients.values():
            client.close()

        for pub in self._publishers.values():
            pub.close()


if __name__ == "__main__":
    node = UINode("0.0.0.0", 5000)
    node.start()
