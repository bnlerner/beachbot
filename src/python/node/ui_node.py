import os
import sys
from typing import Dict

from flask import Flask, Response, jsonify, render_template, request

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ipc import core, messages, pubsub, registry, session
from models import constants, motor_velocity_model

_INDEX_PATH = "index.html"
_JOYSTICK_ENDPOINT = "/joystick_input"


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
        self._controller = motor_velocity_model.MotorVelocityModel(
            session.get_robot_config()
        )
        self._app = Flask(__name__)
        self._setup_routes()
        self._stop_robot: bool = True
        self._publishers: Dict[core.ChannelSpec, pubsub.Publisher] = {}

        self._add_publishers(
            registry.Channels.FRONT_LEFT_MOTOR_CMD,
            registry.Channels.FRONT_RIGHT_MOTOR_CMD,
            registry.Channels.REAR_LEFT_MOTOR_CMD,
            registry.Channels.REAR_RIGHT_MOTOR_CMD,
        )

    def start(self) -> None:
        """Starts the UI node."""
        self._app.run(host=self._host, port=self._port, debug=self._debug)

    def _add_publishers(self, *channels: core.ChannelSpec) -> None:
        for channel in channels:
            self._publishers[channel] = pubsub.Publisher(self._node_id, channel)

    def _setup_routes(self) -> None:
        """Setups all routes"""

        @self._app.route("/")
        def index() -> str:
            return render_template(_INDEX_PATH)

        @self._app.route(_JOYSTICK_ENDPOINT, methods=["POST"])
        def joystick_input() -> Response:
            # Capture joystick input from POST request
            if (data := request.json) is None:
                return jsonify({"status": "failed", "message": "Invalide JSON"})

            self._update_controller(data)
            self._publish_motor_cmd_msgs()
            return jsonify({"status": "success", "message": "Data received"})

    def _update_controller(self, data: Dict[str, str]) -> None:
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
        self._controller.update(linear_velocity, angular_velocity)

    def _publish_motor_cmd_msgs(self) -> None:
        for motor in self._motor_configs:
            velocity = 0.0 if self._stop_robot else self._controller.velocity(motor)
            msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
            channel = registry.motor_channel(motor)
            self._publishers[channel].publish(msg)


if __name__ == "__main__":
    node = UINode("0.0.0.0", 5000)
    node.start()
