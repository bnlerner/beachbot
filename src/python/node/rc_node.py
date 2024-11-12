import asyncio
import os
import sys
import time
from typing import Optional, Union

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from controls import rc_velocity_generator
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
        self._rc_velocity_generator = rc_velocity_generator.RCVelocityGenerator(
            session.get_robot_config()
        )
        self.add_publishers(
            registry.Channels.FRONT_LEFT_MOTOR_CMD,
            registry.Channels.FRONT_RIGHT_MOTOR_CMD,
            registry.Channels.REAR_LEFT_MOTOR_CMD,
            registry.Channels.REAR_RIGHT_MOTOR_CMD,
        )
        self.add_tasks(self._rc_listener.start, self._control_motors)

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=True)

    def _on_release(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=False)

    async def _control_motors(self) -> None:
        total_time = 1.0 / _PUBLISH_RATE
        while True:
            start_time = time.perf_counter()
            self._publish_motor_cmd_msgs()

            write_time = time.perf_counter() - start_time
            sleep_time = total_time - write_time
            await asyncio.sleep(sleep_time)

    def _publish_motor_cmd_msgs(self) -> None:
        for motor in self._motor_configs:
            velocity = self._rc_velocity_generator.velocity(motor)
            msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
            channel = registry.motor_channel(motor)
            self.publish(channel, msg)

    async def shutdown_hook(self) -> None:
        self._rc_listener.stop()


if __name__ == "__main__":
    node = RCRobotNode()
    node.start()
