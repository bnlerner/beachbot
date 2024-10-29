import asyncio
import collections
from typing import DefaultDict

import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers.can import connection, enums
from drivers.can import messages as can_messages
from ipc import channels, core, session
from ipc import messages as ipc_messages
from odrive import enums as odrive_enums  # type: ignore[import-untyped]

from node import base_node


class MotorControlNode(base_node.BaseNode):
    """A Node to control the ODrive motor controllers via a CAN bus interface."""

    def __init__(self) -> None:
        super().__init__(core.NodeID(name="motor_control"))
        self._can_bus = connection.CANSimple(
            enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN
        )
        self._motor_configs = session.get_robot_motor_configs("beachbot-1")

        self._motor_axis_state: DefaultDict[int, int] = collections.defaultdict(
            lambda: -1
        )

        self._can_bus.register_callbacks(
            (can_messages.EncoderEstimatesMessage, _async_print_msg),
            (can_messages.HeartbeatMessage, self._set_axis_state_from_heartbeat),
        )
        self.add_subscribers(
            {
                channels.Channels.FRONT_LEFT_MOTOR_CMD: self._send_motor_cmd,
                channels.Channels.FRONT_RIGHT_MOTOR_CMD: self._send_motor_cmd,
                channels.Channels.REAR_LEFT_MOTOR_CMD: self._send_motor_cmd,
                channels.Channels.REAR_RIGHT_MOTOR_CMD: self._send_motor_cmd,
            }
        )
        self.add_tasks(self._set_closed_loop_axis_state, self._can_bus.listen)

    async def shutdown_hook(self) -> None:
        # Sends a zero velocity to stop the motors.
        for motor in self._motor_configs:
            msg = ipc_messages.MotorCommandMessage(motor=motor, velocity=0.0)
            await self._send_motor_cmd(msg)

        await self._set_axis_state(odrive_enums.AxisState.IDLE)
        self._can_bus.shutdown()

    async def _set_closed_loop_axis_state(self) -> None:
        await self._set_axis_state(odrive_enums.AxisState.CLOSED_LOOP_CONTROL)

    async def _set_axis_state(self, axis_state: odrive_enums.AxisState) -> None:
        for motor in self._motor_configs:
            axis_msg = can_messages.SetAxisStateMessage(
                motor.node_id, axis_state=axis_state
            )
            await self._can_bus.send(axis_msg)

        # Wait for all motors to set and respond
        await self._wait_for_closed_loop_axis_state()

    async def _wait_for_closed_loop_axis_state(self) -> None:
        closed_loop_state = odrive_enums.AxisState.CLOSED_LOOP_CONTROL.value
        while True:
            if all(
                self._motor_axis_state[motor.node_id] == closed_loop_state
                for motor in self._motor_configs
            ):
                break

            await asyncio.sleep(0.01)

    async def _send_motor_cmd(self, msg: ipc_messages.MotorCommandMessage) -> None:
        vel_msg = can_messages.SetVelocityMessage(
            msg.motor.node_id, velocity=msg.velocity, torque=0.0
        )
        await self._can_bus.send(vel_msg)

    async def _set_axis_state_from_heartbeat(
        self, msg: can_messages.HeartbeatMessage
    ) -> None:
        self._motor_axis_state[msg.node_id] = msg.axis_state


async def _async_print_msg(msg: can_messages.OdriveCanMessage) -> None:
    print(msg)


if __name__ == "__main__":
    node = MotorControlNode()
    try:
        node.start()
    except KeyboardInterrupt:
        # Little trick to prevent ^C or ^Z from being printed
        sys.stderr.write("\r")
