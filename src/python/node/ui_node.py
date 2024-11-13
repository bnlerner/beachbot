import os
import sys
from typing import Dict, Optional

from flask import Flask, Response, jsonify, render_template, request

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
from ipc import core, messages, pubsub, registry, session
from ipc import request as ipc_request
from models import constants, motor_velocity_model
from planning import primitives

_INDEX_PATH = "index.html"
_RC_ENDPOINT = "/joystick_input"
_NAVIGATE_ENDPOINT = "/navigate"
_CERT_PATH = "env/auth/cert.pem"
_KEY_PATH = "env/auth/key.pem"


class UINode:
    """A node to host the UI of beachbot and allow interface to its servers and actions.
    Does not use the base node because flask has issues running using async.
    """

    def __init__(self, host: str, port: int, *, debug: bool = False) -> None:
        self._node_id = registry.NodeIDs.UI
        self._host = host
        self._port = port
        self._debug = debug

        self._motor_configs = session.get_robot_motors()
        self._rc_controller = motor_velocity_model.MotorVelocityModel(
            session.get_robot_config()
        )
        self._app = Flask(__name__)
        self._setup_routes()
        self._stop_robot: bool = True
        self._publishers: Dict[core.ChannelSpec, pubsub.Publisher] = {}
        self._request_clients: Dict[core.RequestSpec, ipc_request.RequestClient] = {}

        self._add_rc_cmd_publishers()
        self._add_request_clients(registry.Requests.NAVIGATE)

    def start(self) -> None:
        """Starts the UI node."""
        self._app.run(
            host=self._host,
            port=self._port,
            debug=self._debug,
            ssl_context=(_CERT_PATH, _KEY_PATH),
        )

    def _add_rc_cmd_publishers(self) -> None:
        self._publishers[registry.Channels.FRONT_LEFT_MOTOR_CMD] = pubsub.Publisher(
            self._node_id, registry.Channels.FRONT_LEFT_MOTOR_CMD
        )
        self._publishers[registry.Channels.FRONT_RIGHT_MOTOR_CMD] = pubsub.Publisher(
            self._node_id, registry.Channels.FRONT_RIGHT_MOTOR_CMD
        )
        self._publishers[registry.Channels.REAR_LEFT_MOTOR_CMD] = pubsub.Publisher(
            self._node_id, registry.Channels.REAR_LEFT_MOTOR_CMD
        )
        self._publishers[registry.Channels.REAR_RIGHT_MOTOR_CMD] = pubsub.Publisher(
            self._node_id, registry.Channels.REAR_RIGHT_MOTOR_CMD
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

        @self._app.route(_RC_ENDPOINT, methods=["POST"])
        def rc_input() -> Response:
            # Capture joystick input from POST request to RC
            if (data := request.json) is None:
                return jsonify({"status": "failed", "message": "Invalide JSON"})

            self._update_rc_controller(data)
            self._publish_rc_cmd_msgs()
            return jsonify({"status": "success", "message": "Data received"})

        @self._app.route(_NAVIGATE_ENDPOINT, methods=["POST"])
        async def come_to_me_request() -> Response:
            if (data := request.json) is None:
                return jsonify({"status": "failed", "message": "Invalide JSON"})

            if (nav_request := self._gen_navigate_request(data)) is None:
                return jsonify(
                    {"status": "failed", "message": "No Latitude or longitude received"}
                )

            await self._send_request(registry.Requests.NAVIGATE, nav_request)
            return jsonify({"status": "success", "message": "finished navigating."})

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

    def _publish_rc_cmd_msgs(self) -> None:
        for motor in self._motor_configs:
            velocity = 0.0 if self._stop_robot else self._rc_controller.velocity(motor)
            msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
            channel = registry.motor_channel(motor)
            self._publishers[channel].publish(msg)

    async def _send_request(
        self, spec: core.RequestSpec, request_msg: core.Request
    ) -> core.RequestResponse:
        if spec not in self._request_clients:
            raise RuntimeError(
                "Unrecognized Request spec, did you forget to add it? "
                f"{spec.base_channel.upper()}"
            )

        return await self._request_clients[spec].send(request_msg)

    def _gen_navigate_request(
        self, data: Dict[str, str]
    ) -> Optional[messages.NavigateRequest]:
        if (lat_str := data.get("latitude")) and (long_str := data.get("longitude")):
            latitude: float = eval(lat_str)
            longitude: float = eval(long_str)

            # Process the GPS data here (store it, log it, etc.)
            log.info(
                f"Received GPS data: Latitude = {latitude}, Longitude = {longitude}"
            )
            gps_pt = primitives.GPSPoint(latitude=latitude, longitude=longitude)
            return messages.NavigateRequest(target=gps_pt)
        else:
            return None


if __name__ == "__main__":
    node = UINode("0.0.0.0", 5000)
    node.start()
