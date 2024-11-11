import collections
import math
import os
import sys
from typing import DefaultDict

from odrive import enums as odrive_enums  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers.can import connection, enums
from drivers.can import messages as can_messages
from ipc import messages as ipc_messages
from ipc import registry, session

from node import base_node

_CLOSED_LOOP_STATE_INT = (
    closed_loop_state
) = odrive_enums.AxisState.CLOSED_LOOP_CONTROL.value


class MotorControlNode(base_node.BaseNode):
    """A Node to control the ODrive motor controllers via a CAN bus interface."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.MOTOR_CONTROL)
        self._can_bus = connection.CANSimple(
            enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN
        )
        self._motor_configs = session.get_robot_motors()

        self._motor_axis_state: DefaultDict[int, int] = collections.defaultdict(
            lambda: -1
        )
        self._motor_velocity: DefaultDict[int, float] = collections.defaultdict(
            lambda: math.nan
        )

        self._can_bus.register_callbacks(
            (can_messages.EncoderEstimatesMessage, self._set_motor_velocity),
            (can_messages.HeartbeatMessage, self._set_axis_state_from_heartbeat),
        )
        self.add_subscribers(
            {
                registry.Channels.FRONT_LEFT_MOTOR_CMD: self._send_motor_cmd,
                registry.Channels.FRONT_RIGHT_MOTOR_CMD: self._send_motor_cmd,
                registry.Channels.REAR_LEFT_MOTOR_CMD: self._send_motor_cmd,
                registry.Channels.REAR_RIGHT_MOTOR_CMD: self._send_motor_cmd,
            }
        )
        self.add_tasks(self._set_closed_loop_axis_state, self._can_bus.listen)

    async def shutdown_hook(self) -> None:
        # Sends a zero velocity to stop the motors.
        for motor in self._motor_configs:
            msg = ipc_messages.MotorCommandMessage(motor=motor, velocity=0.0)
            await self._send_motor_cmd(msg)
            await self._set_axis_state(motor.node_id, odrive_enums.AxisState.IDLE)
        self._can_bus.shutdown()

    async def _set_closed_loop_axis_state(self) -> None:
        for motor in self._motor_configs:
            await self._set_axis_state(
                motor.node_id, odrive_enums.AxisState.CLOSED_LOOP_CONTROL
            )

    async def _set_axis_state(
        self, node_id: int, axis_state: odrive_enums.AxisState
    ) -> None:
        axis_msg = can_messages.SetAxisStateMessage(node_id, axis_state=axis_state)
        await self._can_bus.send(axis_msg)

    async def _send_motor_cmd(self, msg: ipc_messages.MotorCommandMessage) -> None:
        # Return early and wait until the motor enters a closed loop axis state prior to
        # sending any commands to it.
        if not self._ready_to_move(msg.motor.node_id):
            return None

        vel_msg = can_messages.SetVelocityMessage(
            msg.motor.node_id, velocity=msg.velocity, torque=0.0
        )
        await self._can_bus.send(vel_msg)

    async def _set_axis_state_from_heartbeat(
        self, msg: can_messages.HeartbeatMessage
    ) -> None:
        self._motor_axis_state[msg.node_id] = msg.axis_state

    async def _set_motor_velocity(
        self, msg: can_messages.EncoderEstimatesMessage
    ) -> None:
        self._motor_velocity[msg.node_id] = msg.vel_estimate

    def _ready_to_move(self, node_id: int) -> bool:
        return self._motor_axis_state[node_id] == _CLOSED_LOOP_STATE_INT


if __name__ == "__main__":
    node = MotorControlNode()
    node.start()
