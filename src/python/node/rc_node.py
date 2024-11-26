import os
import sys
from typing import Optional, Union

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controls import keyboard_rc_controller
from ipc import messages, registry, session
from pynput import keyboard

from node import base_node

_PUBLISH_RATE = 100  # In Hz


class RCRobotNode(base_node.BaseNode):
    """A node that takes in RC commands and writes them as target motor velocities.

    NOTE: Requires setting the DISPLAY environment variable prior to running on robot
    DISPLAY=":1" python3 src/python/node/rc_node.py
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.RC)

        self._motor_configs = session.get_robot_motors()
        self._rc_listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )

        # Moves at 1.0 turns/s default for any RC command
        self._rc_velocity_generator = keyboard_rc_controller.KeyboardRCController(
            session.get_robot_config()
        )
        self.add_publishers(
            registry.Channels.MOTOR_CMD_FRONT_LEFT,
            registry.Channels.MOTOR_CMD_FRONT_RIGHT,
            registry.Channels.MOTOR_CMD_REAR_LEFT,
            registry.Channels.MOTOR_CMD_REAR_RIGHT,
        )
        self.add_tasks(self._rc_listener.start)
        self.add_looped_tasks({self._publish_motor_cmd_msgs: _PUBLISH_RATE})

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=True)

    def _on_release(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=False)

    def _publish_motor_cmd_msgs(self) -> None:
        for motor in self._motor_configs:
            velocity = self._rc_velocity_generator.velocity(motor)
            msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
            channel = registry.motor_command_channel(motor)
            self.publish(channel, msg)

    async def shutdown_hook(self) -> None:
        self._rc_listener.stop()


if __name__ == "__main__":
    node = RCRobotNode()
    node.start()
