import collections
import math
import os
import sys
from typing import DefaultDict, Dict, Optional

from odrive import enums as odrive_enums  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
from config import robot_config
from drivers import can as can_messages
from drivers.can import connection, enums
from ipc import messages as ipc_messages
from ipc import registry, session

from node import base_node

_CLOSED_LOOP_STATE = odrive_enums.AxisState.CLOSED_LOOP_CONTROL
_WATCHDOG_EXPIRY = odrive_enums.ODriveError.WATCHDOG_TIMER_EXPIRED
_NO_ERROR = odrive_enums.ODriveError.NONE
_VEL_PUB_RATE = 50  # Roughly publish at 50 hz or 20 ms
_MOTOR_CMD_PUB_RATE = 100  # Roughly publish at 100 hz or 10 ms
# MyActuator is request/response; poll multi-turn angle for position feedback.
_MYACT_STATUS_POLL_RATE = 50
_INTEGRATOR_PATH = "axis0.controller.vel_integrator_torque"


class MotorControlNode(base_node.BaseNode):
    """Controls drivetrain motors over CAN.

    Supports ODrive (velocity) and MyActuator V3 (absolute position, cmd 0xA4) on
    the same SocketCAN bus. MyActuator uses only the official position-move API
    (absolute multi-turn degrees + max speed dps); no torque/current constants.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.MOTOR_CONTROL)
        self._can_bus = connection.CANSimple(
            enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN
        )
        self._motor_configs = session.get_robot_motors()
        self._motors_by_id: Dict[int, robot_config.Motor] = {
            motor.node_id: motor for motor in self._motor_configs
        }
        self._odrive_motors = [m for m in self._motor_configs if m.is_odrive]
        self._myact_motors = [m for m in self._motor_configs if m.is_myactuator]

        self._endpoint_data = (
            session.get_motor_endpoint_data() if self._odrive_motors else {}
        )

        self._motor_axis_state: DefaultDict[
            int, odrive_enums.AxisState
        ] = collections.defaultdict(lambda: odrive_enums.AxisState.UNDEFINED)
        self._motor_axis_error: DefaultDict[
            int, odrive_enums.ODriveError
        ] = collections.defaultdict(lambda: odrive_enums.ODriveError.NONE)
        # MyActuator readiness and last reported error flags (protocol error word).
        self._myact_ready: DefaultDict[int, bool] = collections.defaultdict(
            lambda: False
        )
        self._myact_error_state: DefaultDict[int, int] = collections.defaultdict(
            lambda: 0
        )
        self._motor_vel_cmd: DefaultDict[
            int, Optional[ipc_messages.MotorCommandMessage]
        ] = collections.defaultdict(lambda: None)
        self._measured_velocity: DefaultDict[int, float] = collections.defaultdict(
            lambda: math.nan
        )
        self._measured_position_deg: DefaultDict[int, float] = collections.defaultdict(
            lambda: math.nan
        )

        self._register_can_callbacks()
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

        looped_tasks = {
            self._publish_velocity: _VEL_PUB_RATE,
            self._publish_motor_cmds: _MOTOR_CMD_PUB_RATE,
        }
        if self._myact_motors:
            looped_tasks[self._poll_myact_status] = _MYACT_STATUS_POLL_RATE
        self.add_looped_tasks(looped_tasks)

        log.info(
            "MotorControlNode controllers: "
            f"odrive={len(self._odrive_motors)} myactuator={len(self._myact_motors)}"
        )

    def _register_can_callbacks(self) -> None:
        callbacks = []
        if self._odrive_motors:
            callbacks.extend(
                [
                    (can_messages.EncoderEstimatesMessage, self._set_motor_velocity),
                    (can_messages.HeartbeatMessage, self._update_from_heartbeat),
                ]
            )
        if self._myact_motors:
            callbacks.extend(
                [
                    (
                        can_messages.ReadMotorStatus1Message,
                        self._update_from_myact_status1,
                    ),
                    (
                        can_messages.ReadMultiTurnAngleMessage,
                        self._update_from_myact_angle,
                    ),
                ]
            )
        if callbacks:
            self._can_bus.register_callbacks(*callbacks)

    async def shutdown_hook(self) -> None:
        for motor in self._motor_configs:
            if motor.is_odrive:
                msg = ipc_messages.MotorCommandMessage(motor=motor, velocity=0.0)
                await self._send_motor_cmd(msg)
                await self._set_axis_state(motor.node_id, odrive_enums.AxisState.IDLE)
            elif motor.is_myactuator:
                # Official API: stopMotor (0x81) then shutdownMotor (0x80).
                await self._can_bus.send(
                    can_messages.MotorStopCommand(node_id=motor.node_id)
                )
                await self._can_bus.send(
                    can_messages.MotorShutdownCommand(node_id=motor.node_id)
                )
                await self._can_bus.send(
                    can_messages.SystemBrakeLockCommand(node_id=motor.node_id)
                )
        self._can_bus.shutdown()

    async def _initialize_motors(self) -> None:
        for motor in self._odrive_motors:
            await self._initialize_odrive_motor(motor)
        for motor in self._myact_motors:
            await self._initialize_myact_motor(motor)

    async def _initialize_odrive_motor(self, motor: robot_config.Motor) -> None:
        await self._clear_errors(motor.node_id)
        await self._set_axis_state(
            motor.node_id, odrive_enums.AxisState.CLOSED_LOOP_CONTROL
        )
        await self._set_controller_mode(
            motor.node_id,
            odrive_enums.ControlMode.VELOCITY_CONTROL,
            odrive_enums.InputMode.PASSTHROUGH,
        )

    async def _initialize_myact_motor(self, motor: robot_config.Motor) -> None:
        """Bring a MyActuator V3 motor online for absolute position control."""
        node_id = motor.node_id
        log.info(
            f"Initializing MyActuator V3 motor {motor.location} (node_id={node_id})"
        )
        await self._can_bus.send(can_messages.ReadMotorStatus1Message(node_id=node_id))
        # Official API: releaseBrake (0x77).
        await self._can_bus.send(
            can_messages.SystemBrakeReleaseCommand(node_id=node_id)
        )
        await self._can_bus.send(can_messages.ReadMultiTurnAngleMessage(node_id=node_id))
        await self._can_bus.send(can_messages.ReadMotorStatus1Message(node_id=node_id))
        self._myact_ready[node_id] = True

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
                if not msg.is_expired():
                    await self._send_motor_cmd(msg)
                else:
                    if motor.is_odrive:
                        no_vel_msg = ipc_messages.MotorCommandMessage(
                            motor=motor, velocity=0.0
                        )
                        await self._send_motor_cmd(no_vel_msg)
                        await self._reset_motor_integral(msg.motor.node_id)
                    elif motor.is_myactuator:
                        # Hold last position on command timeout: re-send stop (0x81).
                        await self._can_bus.send(
                            can_messages.MotorStopCommand(node_id=motor.node_id)
                        )

    async def _poll_myact_status(self) -> None:
        """Request multi-turn angle (0x92) for position feedback."""
        for motor in self._myact_motors:
            await self._can_bus.send(
                can_messages.ReadMultiTurnAngleMessage(node_id=motor.node_id)
            )

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
        if self._is_conflicting_msg(msg):
            log.error(f"Received conflicting message: {msg}, shutting down the server.")
            self.shutdown()

        self._motor_vel_cmd[msg.motor.node_id] = msg

    async def _send_motor_cmd(self, msg: ipc_messages.MotorCommandMessage) -> None:
        motor = msg.motor
        if not self._ready_to_move(motor.node_id):
            if (
                motor.is_myactuator
                and self._myact_error_state[motor.node_id] == 0
                and not self._myact_ready[motor.node_id]
            ):
                await self._initialize_myact_motor(motor)
            if not self._ready_to_move(motor.node_id):
                return None

        if motor.is_odrive:
            await self._send_odrive_motor_cmd(msg)
        elif motor.is_myactuator:
            await self._send_myact_position_cmd(msg)
        else:
            log.error(f"Unsupported motor controller type: {motor.controller_type}")

    async def _send_odrive_motor_cmd(
        self, msg: ipc_messages.MotorCommandMessage
    ) -> None:
        if msg.reset_integral:
            await self._reset_motor_integral(msg.motor.node_id)

        can_msg = can_messages.SetVelocityMessage(
            msg.motor.node_id, velocity=msg.velocity, torque=msg.feedforward_torque
        )
        await self._can_bus.send(can_msg)

    async def _send_myact_position_cmd(
        self, msg: ipc_messages.MotorCommandMessage
    ) -> None:
        """Absolute position move via MyActuator cmd 0xA4.

        Equivalent to myactuator_rmd ActuatorInterface.sendPositionAbsoluteSetpoint(
            position_deg, max_speed_dps).
        """
        if msg.position is None:
            log.error(
                f"MyActuator {msg.motor.location} requires MotorCommandMessage.position "
                "(absolute degrees); ignoring command without position."
            )
            return

        max_speed = (
            msg.max_speed_dps
            if msg.max_speed_dps is not None
            else msg.motor.default_max_speed_dps
        )
        can_msg = can_messages.PositionControlCommand(
            node_id=msg.motor.node_id,
            position=float(msg.position),
            max_speed=int(max_speed),
        )
        await self._can_bus.send(can_msg)

    async def _activate_e_stop(self, _: ipc_messages.StopMotorsMessage) -> None:
        for motor in self._motor_configs:
            if motor.is_odrive:
                await self._can_bus.send(can_messages.EStop(motor.node_id))
            elif motor.is_myactuator:
                # Official API: stopMotor (0x81).
                await self._can_bus.send(
                    can_messages.MotorStopCommand(node_id=motor.node_id)
                )
                self._myact_ready[motor.node_id] = False

    async def _update_from_heartbeat(self, msg: can_messages.HeartbeatMessage) -> None:
        self._motor_axis_state[msg.node_id] = msg.axis_state
        self._motor_axis_error[msg.node_id] = msg.axis_error
        if msg.axis_error != _NO_ERROR:
            error = self._motor_axis_error[msg.node_id]
            motor = self._motors_by_id.get(msg.node_id)
            location = motor.location if motor else msg.node_id
            log.error(f"Axis error: {error.name} on motor: {location}")

    async def _set_motor_velocity(
        self, msg: can_messages.EncoderEstimatesMessage
    ) -> None:
        self._measured_velocity[msg.node_id] = msg.vel_estimate

    async def _update_from_myact_status1(
        self, msg: can_messages.ReadMotorStatus1Message
    ) -> None:
        self._myact_error_state[msg.node_id] = msg.error_state
        if msg.error_state != 0:
            self._myact_ready[msg.node_id] = False
            motor = self._motors_by_id.get(msg.node_id)
            location = motor.location if motor else msg.node_id
            log.error(
                f"MyActuator error 0x{msg.error_state:04x} on motor: {location} "
                f"(temp={msg.temperature}C V={msg.voltage:.1f})"
            )
        elif msg.node_id in self._motors_by_id:
            self._myact_ready[msg.node_id] = True

    async def _update_from_myact_angle(
        self, msg: can_messages.ReadMultiTurnAngleMessage
    ) -> None:
        # getMultiTurnAngle equivalent (cmd 0x92); units are degrees.
        self._measured_position_deg[msg.node_id] = msg.angle

    def _ready_to_move(self, node_id: int) -> bool:
        motor = self._motors_by_id.get(node_id)
        if motor is None:
            return False
        if motor.is_odrive:
            correct_axis_state = self._motor_axis_state[node_id] == _CLOSED_LOOP_STATE
            no_serious_axis_error = self._motor_axis_error[node_id] in (
                _NO_ERROR,
                _WATCHDOG_EXPIRY,
            )
            return correct_axis_state and no_serious_axis_error
        if motor.is_myactuator:
            return self._myact_ready[node_id] and self._myact_error_state[node_id] == 0
        return False

    async def _reset_motor_integral(self, node_id: int) -> None:
        """Resets the ODrive velocity integral term to zero."""
        if not self._endpoint_data:
            return
        endpoint_id = self._endpoint_data["endpoints"][_INTEGRATOR_PATH]["id"]
        endpoint_type = self._endpoint_data["endpoints"][_INTEGRATOR_PATH]["type"]

        msg = can_messages.WriteParameterCommand(
            node_id, endpoint_id=endpoint_id, value_type=endpoint_type, value=0.0
        )
        await self._can_bus.send(msg)

    async def _clear_errors(self, node_id: int) -> None:
        msg = can_messages.ClearErrorsCommand(node_id)
        await self._can_bus.send(msg)

    def _is_conflicting_msg(self, msg: ipc_messages.MotorCommandMessage) -> bool:
        """A conflicting message such as a command from a different node that is
        outside of the expiry time of the current motor velocity command message.
        """
        if cur_msg := self._motor_vel_cmd[msg.motor.node_id]:
            return msg.origin != cur_msg.origin and not cur_msg.is_expired()
        else:
            return False


if __name__ == "__main__":
    node = MotorControlNode()
    node.start()
