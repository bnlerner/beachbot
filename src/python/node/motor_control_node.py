import asyncio
from odrive import enums as odrive_enums  # type: ignore[import-untyped]
import sys
import os
from pynput import keyboard
from typing import Optional, Union

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import motor_config
from controls import rc_velocity_generator
from drivers.can import connection, enums, messages
from ipc import session
from node import base_node


class MotorControlNode(base_node.BaseNode):
    """A Node to control the ODrive motor controllers via a CAN bus interface.

    NOTE: Requires setting the DISPLAY environment variable like so prior to running on beachbot
    DISPLAY=":1" python3 src/python/node/motor_control_node.py
    """

    def __init__(self) -> None:
        super().__init__()
        self._can_bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)
        self._motor_configs = session.get_robot_motor_configs("beachbot-1")

        # RC control
        self._rc_listener = keyboard.Listener(on_press=self._on_press, on_release=self._on_release)
        self._rc_listener.start()
        # Moves at 1.0 turns/s for any RC command
        self._rc_velocity_generator = rc_velocity_generator.RCVelocityGenerator(1.0, self._motor_configs)

        self._can_bus.register_callbacks(
            (messages.EncoderEstimatesMessage, _async_print_msg),
            (messages.HeartbeatMessage, _async_print_msg),
        )
        self.add_tasks(self._set_closed_loop_axis_state, self._update_motor_velocity, self._can_bus.listen)

    async def shutdown_hook(self) -> None:
        # Sends a zero velocity to stop the motors.
        for motor in self._motor_configs:
            await self._send_velocity_cmd(motor, 0.0)

        await self._set_axis_state(odrive_enums.AxisState.IDLE)
        self._can_bus.shutdown()
        self._rc_listener.stop()

    async def _set_closed_loop_axis_state(self) -> None:
        await self._set_axis_state(odrive_enums.AxisState.CLOSED_LOOP_CONTROL)

    async def _set_axis_state(self, axis_state: odrive_enums.AxisState) -> None:
        for motor in self._motor_configs:
            axis_msg = messages.SetAxisStateMessage(motor.node_id, axis_state=axis_state.value)
            await self._can_bus.send(axis_msg)
            # Wait for the motor to set and respond
            await asyncio.sleep(0.1)


    async def _send_velocity_cmd(self, motor: motor_config.MotorConfig, velocity: float) -> None:
        vel_msg = messages.SetVelocityMessage(motor.node_id, velocity=velocity)
        await self._can_bus.send(vel_msg)

    def _on_press(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if key == keyboard.Key.esc:
            exit(0)

        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=True)

    def _on_release(self, key: Optional[Union[keyboard.Key, keyboard.KeyCode]]) -> None:
        if isinstance(key, keyboard.Key):
            self._rc_velocity_generator.update(key, pressed=False)

    async def _update_motor_velocity(self) -> None:
        while True:
            for motor, velocity in self._rc_velocity_generator.velocities().items():
                await self._send_velocity_cmd(motor, velocity)

            await asyncio.sleep(0.01)

async def _async_print_msg(msg: messages.OdriveCanMessage) -> None:
    print(msg)

if __name__ == "__main__":
    node = MotorControlNode()
    node.start()
