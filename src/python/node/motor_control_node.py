import asyncio
from odrive import enums as odrive_enums  # type: ignore[import-untyped]

from config import motor_config
from drivers.can import connection, enums, messages
from ipc import session
from node import base_node

# in turns per second
_INITIAL_VELOCITY = 1.0


class MotorControlNode(base_node.BaseNode):
    """A Node to control the ODrive motor controllers via a CAN bus interface."""

    def __init__(self):
        super().__init__()
        self._can_bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)
        self._motor_configs = session.get_robot_motor_configs("beachbot-1")

        self._can_bus.register_callbacks(
            (messages.EncoderEstimatesMessage, print),
            (messages.HeartbeatMessage, print),
        )
        self.add_tasks(self._startup_motors, self._can_bus.listen)

    async def shutdown_hook(self):
        # Sends a zero velocity to stop the motors.
        for motor in self._motor_configs:
            await self._send_velocity_cmd(motor, 0.0)

        self._can_bus.shutdown()

    async def _startup_motors(self) -> None:
        for motor in self._motor_configs:
            axis_msg = messages.SetAxisStateMessage(
                motor.node_id,
                axis_state=odrive_enums.AxisState.CLOSED_LOOP_CONTROL.value
            )
            await asyncio.sleep(0.01)
            await self._can_bus.send(axis_msg)
            await self._send_velocity_cmd(motor, _INITIAL_VELOCITY)

    async def _send_velocity_cmd(self, motor: motor_config.MotorConfig, velocity: float) -> None:
        signed_velocity = motor.direction * velocity
        vel_msg = messages.SetVelocityMessage(motor.node_id, velocity=signed_velocity)
        await self._can_bus.send(vel_msg)

if __name__ == "__main__":
    node = MotorControlNode()
    node.start()
