import collections
import math
import os
import sys
from typing import DefaultDict

from odrive import enums as odrive_enums  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
from drivers.can import connection, enums
from drivers.can import messages as can_messages
from ipc import messages as ipc_messages
from ipc import registry, session

from node import base_node

_CLOSED_LOOP_STATE = odrive_enums.AxisState.CLOSED_LOOP_CONTROL
_NO_ERROR = odrive_enums.ODriveError.NONE
_VEL_PUB_RATE = 50  # Roughly publish at 50 hz or 20 ms


class MotorControlNode(base_node.BaseNode):
    """A Node to control the ODrive motor controllers via a CAN bus interface."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.MOTOR_CONTROL)
        self._can_bus = connection.CANSimple(
            enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN
        )
        self._motor_configs = session.get_robot_motors()

        self._motor_axis_state: DefaultDict[
            int, odrive_enums.AxisState
        ] = collections.defaultdict(lambda: odrive_enums.AxisState.UNDEFINED)
        self._motor_axis_error: DefaultDict[
            int, odrive_enums.ODriveError
        ] = collections.defaultdict(lambda: odrive_enums.ODriveError.NONE)
        self._measured_velocity: DefaultDict[int, float] = collections.defaultdict(
            lambda: math.nan
        )
        self._command_setpoint: DefaultDict[int, float] = collections.defaultdict(
            lambda: math.nan
        )

        self._can_bus.register_callbacks(
            (can_messages.EncoderEstimatesMessage, self._set_motor_velocity),
            (can_messages.HeartbeatMessage, self._update_from_heartbeat),
        )
        self.add_subscribers(
            {
                registry.Channels.MOTOR_CMD_FRONT_LEFT: self._send_motor_cmd,
                registry.Channels.MOTOR_CMD_FRONT_RIGHT: self._send_motor_cmd,
                registry.Channels.MOTOR_CMD_REAR_LEFT: self._send_motor_cmd,
                registry.Channels.MOTOR_CMD_REAR_RIGHT: self._send_motor_cmd,
                registry.Channels.STOP_MOTORS: self._activate_e_stop,
            }
        )
        self.add_publishers(
            registry.Channels.MOTOR_VELOCITY_FRONT_LEFT,
            registry.Channels.MOTOR_VELOCITY_FRONT_RIGHT,
            registry.Channels.MOTOR_VELOCITY_REAR_LEFT,
            registry.Channels.MOTOR_VELOCITY_REAR_RIGHT,
        )
        self.add_tasks(self._initialize_motors, self._can_bus.listen)
        self.add_looped_tasks({self._publish_velocity: _VEL_PUB_RATE})

    async def shutdown_hook(self) -> None:
        # Sends a zero velocity to stop the motors.
        for motor in self._motor_configs:
            msg = ipc_messages.MotorCommandMessage(motor=motor, velocity=0.0)
            await self._send_motor_cmd(msg)
            await self._set_axis_state(motor.node_id, odrive_enums.AxisState.IDLE)
        self._can_bus.shutdown()

    async def _initialize_motors(self) -> None:
        for motor in self._motor_configs:
            # Clears errors for good measure in case there is a latent error in the
            # motor.
            await self._clear_errors(motor.node_id)
            # Closed loop control set to allow velocity control.
            await self._set_axis_state(
                motor.node_id, odrive_enums.AxisState.CLOSED_LOOP_CONTROL
            )

    def _publish_velocity(self) -> None:
        for motor in self._motor_configs:
            channel = registry.motor_velocity_channel(motor)
            estimated_velocity = self._measured_velocity[motor.node_id]
            msg = ipc_messages.MotorVelocityMessage(
                motor=motor, estimated_velocity=estimated_velocity
            )
            self.publish(channel, msg)

    async def _set_axis_state(
        self, node_id: int, axis_state: odrive_enums.AxisState
    ) -> None:
        axis_msg = can_messages.SetAxisStateMessage(node_id, axis_state=axis_state)
        await self._can_bus.send(axis_msg)

    async def _send_motor_cmd(self, msg: ipc_messages.MotorCommandMessage) -> None:
        # Return early and wait until the motor enters a closed loop axis state prior to
        # sending any commands to it. Unable to move if an error exists.
        if not self._ready_to_move(msg.motor.node_id):
            return None

        # Return early if the motor is already set to this command. Done to save some
        # bus bandwidth.
        if self._setpoint_equal(msg):
            return None

        self._command_setpoint[msg.motor.node_id] = msg.velocity
        can_msg = can_messages.SetVelocityMessage(
            msg.motor.node_id, velocity=msg.velocity, torque=msg.feedforward_torque
        )
        await self._can_bus.send(can_msg)

    async def _activate_e_stop(self, _: ipc_messages.StopMotorsMessage) -> None:
        for motor in self._motor_configs:
            e_stop_msg = can_messages.EStop(motor.node_id)
            await self._can_bus.send(e_stop_msg)

    async def _update_from_heartbeat(self, msg: can_messages.HeartbeatMessage) -> None:
        self._motor_axis_state[msg.node_id] = msg.axis_state
        self._motor_axis_error[msg.node_id] = msg.axis_error
        if msg.axis_error != _NO_ERROR:
            error = self._motor_axis_error[msg.node_id]
            motor = filter(
                lambda x: x.node_id == msg.node_id, self._motor_configs
            ).__next__()
            log.error(f"Axis error: {error.name} on motor: {motor.location}")

    async def _set_motor_velocity(
        self, msg: can_messages.EncoderEstimatesMessage
    ) -> None:
        self._measured_velocity[msg.node_id] = msg.vel_estimate

    def _ready_to_move(self, node_id: int) -> bool:
        return (
            self._motor_axis_state[node_id] == _CLOSED_LOOP_STATE
            and self._motor_axis_error[node_id] == _NO_ERROR
        )

    def _setpoint_equal(self, msg: ipc_messages.MotorCommandMessage) -> bool:
        """Returns True if the setpoint for the motor already exists."""
        return self._command_setpoint[msg.motor.node_id] == msg.velocity

    async def _clear_errors(self, node_id: int) -> None:
        msg = can_messages.ClearErrorsCommand(node_id)
        await self._can_bus.send(msg)


if __name__ == "__main__":
    node = MotorControlNode()
    node.start()
