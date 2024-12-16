import collections
import math
import os
import sys
from typing import DefaultDict, Optional

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
_WATCHDOG_EXPIRY = odrive_enums.ODriveError.WATCHDOG_TIMER_EXPIRED
_NO_ERROR = odrive_enums.ODriveError.NONE
_VEL_PUB_RATE = 50  # Roughly publish at 50 hz or 20 ms
_MOTOR_CMD_PUB_RATE = 100  # Roughly publish at 100 hz or 10 ms
_INTEGRATOR_PATH = "axis0.controller.vel_integrator_torque"


class MotorControlNode(base_node.BaseNode):
    """A Node to control the ODrive motor controllers via a CAN bus interface."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.MOTOR_CONTROL)
        self._can_bus = connection.CANSimple(
            enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN
        )
        self._motor_configs = session.get_robot_motors()
        self._endpoint_data = session.get_motor_endpoint_data()

        self._motor_axis_state: DefaultDict[
            int, odrive_enums.AxisState
        ] = collections.defaultdict(lambda: odrive_enums.AxisState.UNDEFINED)
        self._motor_axis_error: DefaultDict[
            int, odrive_enums.ODriveError
        ] = collections.defaultdict(lambda: odrive_enums.ODriveError.NONE)
        self._motor_vel_cmd: DefaultDict[
            int, Optional[ipc_messages.MotorCommandMessage]
        ] = collections.defaultdict(lambda: None)
        self._measured_velocity: DefaultDict[int, float] = collections.defaultdict(
            lambda: math.nan
        )

        self._can_bus.register_callbacks(
            (can_messages.EncoderEstimatesMessage, self._set_motor_velocity),
            (can_messages.HeartbeatMessage, self._update_from_heartbeat),
        )
        self.add_subscribers(
            {
                registry.Channels.MOTOR_CMD_FRONT_LEFT: self._set_motor_cmd,
                registry.Channels.MOTOR_CMD_FRONT_RIGHT: self._set_motor_cmd,
                registry.Channels.MOTOR_CMD_REAR_LEFT: self._set_motor_cmd,
                registry.Channels.MOTOR_CMD_REAR_RIGHT: self._set_motor_cmd,
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
        self.add_looped_tasks(
            {
                self._publish_velocity: _VEL_PUB_RATE,
                self._publish_motor_cmds: _MOTOR_CMD_PUB_RATE,
            }
        )

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
            # Sets the control mode to velocity control using a passthrough or immediate
            # set of the input velocity and torque. Doing this allows more control of
            # the motor behavior. Feedforward torque control was found to only work when
            # using passthrough control.
            await self._set_controller_mode(
                motor.node_id,
                odrive_enums.ControlMode.VELOCITY_CONTROL,
                odrive_enums.InputMode.PASSTHROUGH,
            )

    def _publish_velocity(self) -> None:
        for motor in self._motor_configs:
            channel = registry.motor_velocity_channel(motor)
            estimated_velocity = self._measured_velocity[motor.node_id]
            msg = ipc_messages.MotorVelocityMessage(
                motor=motor, estimated_velocity=estimated_velocity
            )
            self.publish(channel, msg)

    async def _publish_motor_cmds(self) -> None:
        for motor in self._motor_configs:
            if msg := self._motor_vel_cmd[motor.node_id]:
                if not msg.expired():
                    await self._send_motor_cmd(msg)
                else:
                    # Stop the motor if we havent received a motor command in some time
                    # that the message has expired.
                    no_vel_msg = ipc_messages.MotorCommandMessage(
                        motor=motor, velocity=0.0
                    )
                    await self._send_motor_cmd(no_vel_msg)
                    await self._reset_motor_integral(msg.motor.node_id)

    async def _set_axis_state(
        self, node_id: int, axis_state: odrive_enums.AxisState
    ) -> None:
        axis_msg = can_messages.SetAxisStateMessage(node_id, axis_state=axis_state)
        await self._can_bus.send(axis_msg)

    async def _set_controller_mode(
        self,
        node_id: int,
        control_mode: odrive_enums.ControlMode,
        input_mode: odrive_enums.InputMode,
    ) -> None:
        msg = can_messages.SetControllerMode(
            node_id, control_mode=control_mode, input_mode=input_mode
        )
        await self._can_bus.send(msg)

    def _set_motor_cmd(self, msg: ipc_messages.MotorCommandMessage) -> None:
        self._motor_vel_cmd[msg.motor.node_id] = msg

    async def _send_motor_cmd(self, msg: ipc_messages.MotorCommandMessage) -> None:
        # Return early and wait until the motor enters a closed loop axis state prior to
        # sending any commands to it. Unable to move if an error exists.
        if not self._ready_to_move(msg.motor.node_id):
            return None

        if msg.reset_integral:
            await self._reset_motor_integral(msg.motor.node_id)

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
        correct_axis_state = self._motor_axis_state[node_id] == _CLOSED_LOOP_STATE
        no_serious_axis_error = self._motor_axis_error[node_id] in (
            _NO_ERROR or _WATCHDOG_EXPIRY
        )
        return correct_axis_state and no_serious_axis_error

    async def _reset_motor_integral(self, node_id: int) -> None:
        """Resets the motor integral term to zero. Useful since this can wind up
        significantly during velocity control.
        """
        endpoint_id = self._endpoint_data["endpoints"][_INTEGRATOR_PATH]["id"]
        endpoint_type = self._endpoint_data["endpoints"][_INTEGRATOR_PATH]["type"]

        msg = can_messages.WriteParameterCommand(
            node_id, endpoint_id=endpoint_id, value_type=endpoint_type, value=0.0
        )
        await self._can_bus.send(msg)

    async def _clear_errors(self, node_id: int) -> None:
        msg = can_messages.ClearErrorsCommand(node_id)
        await self._can_bus.send(msg)


if __name__ == "__main__":
    node = MotorControlNode()
    node.start()
