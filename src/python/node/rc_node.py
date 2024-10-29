"""
A simple example to print RC commands.
"""
import asyncio
import sys
from typing import Optional, Union

import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import motor_config
from controls import rc_velocity_generator
from ipc import core, messages, session, channels
from pynput import keyboard

from node import base_node


class RCRobotNode(base_node.BaseNode):
    """A node that takes in RC commands and writes them as target motor velocities.

    NOTE: Requires setting the DISPLAY environment variable like so prior to running on beachbot
    DISPLAY=":1" python3 src/python/node/rc_node.py
    """

    def __init__(self) -> None:
        super().__init__(core.NodeID(name="rc"))

        self._motor_configs = session.get_robot_motor_configs("beachbot-1")
        self._rc_listener = keyboard.Listener(
            on_press=self._on_press, on_release=self._on_release
        )
        self._rc_listener.start()

        # Moves at 1.0 turns/s for any RC command
        self._rc_velocity_generator = rc_velocity_generator.RCVelocityGenerator(1.0)
        self.add_publishers(
            channels.Channels.FRONT_LEFT_MOTOR_CMD, 
            channels.Channels.FRONT_RIGHT_MOTOR_CMD,
            channels.Channels.REAR_LEFT_MOTOR_CMD, 
            channels.Channels.REAR_RIGHT_MOTOR_CMD,
            )
        self.add_tasks(self._update_motor_velocity)

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if key == keyboard.Key.esc:
            sys.exit(0)

        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=True)

    def _on_release(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=False)

    async def _update_motor_velocity(self) -> None:
        while True:
            for motor in self._motor_configs:
                velocity = self._rc_velocity_generator.velocity(motor)
                msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
                channel = self._get_motor_channel(motor)
                self.publish(channel, msg)
            await asyncio.sleep(0.01)

    def _get_motor_channel(self, motor: motor_config.MotorConfig) -> core.ChannelSpec:
        if motor.location == motor_config.MotorLocation.FRONT_LEFT:
            return channels.Channels.FRONT_LEFT_MOTOR_CMD
        elif motor.location == motor_config.MotorLocation.FRONT_RIGHT:
            return channels.Channels.FRONT_RIGHT_MOTOR_CMD
        elif motor.location == motor_config.MotorLocation.REAR_LEFT:
            return channels.Channels.REAR_LEFT_MOTOR_CMD
        elif motor.location == motor_config.MotorLocation.REAR_RIGHT:
            return channels.Channels.REAR_RIGHT_MOTOR_CMD
        else:
            raise ValueError("unknown channel")

    def _publish_motor_cmd_msg(
        self, motor: motor_config.MotorConfig, velocity: float
    ) -> None:
        msg = messages.MotorCommandMessage(motor=motor, velocity=velocity)
        msg.write(motor.location.value)

    async def shutdown_hook(self) -> None:
        self._rc_listener.stop()


if __name__ == "__main__":
    node = RCRobotNode()
    try:
        node.start()
    except KeyboardInterrupt:
        # Little trick to prevent ^C or ^Z from being printed
        sys.stderr.write("\r")
