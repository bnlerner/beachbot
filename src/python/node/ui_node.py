import os
import sys

from flask import Flask, Response, jsonify, render_template, request

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import motor_config
from controls import nav_velocity_generator
from ipc import core, messages, registry, session

from node import base_node

_INDEX_PATH = "index.html"
_JOYSTICK_ENDPOINT = "/joystick_input"
# Modifies the velocity as the incoming velocity is always capped betwen -1 and 1 to
# higher target velocities if desired.
_VELOCITY_MODIFIER = 1.0


class UINode(base_node.BaseNode):
    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.UI)

        self._motor_configs = session.get_robot_motor_configs()
        self._controller = nav_velocity_generator.NavVelocityGenerator()
        self._app = Flask(__name__)
        self._setup_routes()
        self._stop_robot: bool = True

        self.add_publishers(
            registry.Channels.FRONT_LEFT_MOTOR_CMD,
            registry.Channels.FRONT_RIGHT_MOTOR_CMD,
            registry.Channels.REAR_LEFT_MOTOR_CMD,
            registry.Channels.REAR_RIGHT_MOTOR_CMD,
        )

        self.add_tasks(self._run_server, self._publish_motor_cmd_msgs)

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

            # Positive X is turn right
            x: float = data.get("x", 0.0)

            # Positive Y is forward
            y: float = data.get("y", 0.0)

            # NOTE: Order not guaranteed, ensures that we stop before moving again.
            self._stop_robot = data.get("stopped", False)

            linear_velocity = _VELOCITY_MODIFIER * x
            # Negative since positive spin is to the left.
            angular_velocity = _VELOCITY_MODIFIER * -y
            self._controller.update(linear_velocity, angular_velocity)

            return jsonify({"status": "success", "message": "Data received"})

    def _publish_motor_cmd_msgs(self) -> None:
        for motor in self._motor_configs:
            velocity = 0.0 if self._stop_robot else self._controller.velocity(motor)
            msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
            channel = self._get_motor_channel(motor)
            self.publish(channel, msg)

    def _get_motor_channel(self, motor: motor_config.MotorConfig) -> core.ChannelSpec:
        if motor.location == motor_config.MotorLocation.FRONT_LEFT:
            return registry.Channels.FRONT_LEFT_MOTOR_CMD
        elif motor.location == motor_config.MotorLocation.FRONT_RIGHT:
            return registry.Channels.FRONT_RIGHT_MOTOR_CMD
        elif motor.location == motor_config.MotorLocation.REAR_LEFT:
            return registry.Channels.REAR_LEFT_MOTOR_CMD
        elif motor.location == motor_config.MotorLocation.REAR_RIGHT:
            return registry.Channels.REAR_RIGHT_MOTOR_CMD
        else:
            raise ValueError("unknown channel")

    def _run_server(
        self, host: str = "0.0.0.0", port: int = 5000, debug: bool = True
    ) -> None:
        self._app.run(host=host, port=port, debug=debug)


if __name__ == "__main__":
    node = UINode()
    node.start()
