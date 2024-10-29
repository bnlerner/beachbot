"""
A simple example to print RC commands.
"""
import asyncio
import os
import sys
import time
from typing import Optional, Union

from python.ipc import registry

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import motor_config
from controls import rc_velocity_generator
from ipc import core, messages, session
from pynput import keyboard

from node import base_node

_PUBLISH_RATE = 100  # In Hz


class RCRobotNode(base_node.BaseNode):
    """A node that takes in RC commands and writes them as target motor velocities.

    NOTE: Requires setting the DISPLAY environment variable like so prior to running on beachbot
    DISPLAY=":1" python3 src/python/node/rc_node.py
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.RC)

        self._motor_configs = session.get_robot_motor_configs("beachbot-1")
        self._rc_listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )

        # Moves at 1.0 turns/s default for any RC command
        self._rc_velocity_generator = rc_velocity_generator.RCVelocityGenerator(1.0)
        self.add_publishers(
            registry.Channels.FRONT_LEFT_MOTOR_CMD,
            registry.Channels.FRONT_RIGHT_MOTOR_CMD,
            registry.Channels.REAR_LEFT_MOTOR_CMD,
            registry.Channels.REAR_RIGHT_MOTOR_CMD,
        )
        self.add_tasks(self._rc_listener.start, self._control_motors)

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if key == keyboard.Key.esc:
            sys.exit(0)

        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=True)

    def _on_release(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=False)

    async def _control_motors(self) -> None:
        total_time = 1.0 / _PUBLISH_RATE
        while True:
            start_time = time.perf_counter()
            for motor in self._motor_configs:
                self._publish_motor_cmd_msg(motor)

            write_time = time.perf_counter() - start_time
            sleep_time = total_time - write_time
            await asyncio.sleep(sleep_time)

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

    def _publish_motor_cmd_msg(self, motor: motor_config.MotorConfig) -> None:
        velocity = self._rc_velocity_generator.velocity(motor)
        msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
        channel = self._get_motor_channel(motor)
        self.publish(channel, msg)

    async def shutdown_hook(self) -> None:
        self._rc_listener.stop()


if __name__ == "__main__":
    node = RCRobotNode()
    try:
        node.start()
    except KeyboardInterrupt:
        # Little trick to prevent ^C or ^Z from being printed
        sys.stderr.write("\r")
