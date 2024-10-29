"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""
import asyncio

from config import motor_config
from drivers.can import connection, enums, messages
from ipc import session
from odrive import enums as odrive_enums  # type: ignore[import-untyped]


async def _control_motor(
    bus: connection.CANSimple, motor: motor_config.MotorConfig
) -> None:
    """Node ID must match `<odrv>.axis0.config.can.node_id`. The default is 0."""
    # Put axis into closed loop control state
    await _set_control_loop_state(bus, motor.node_id)
    await asyncio.sleep(0.1)
    await _set_velocity(bus, motor, 1.0)


async def _set_control_loop_state(bus: connection.CANSimple, node_id: int) -> None:
    axis_state = odrive_enums.AxisState.CLOSED_LOOP_CONTROL
    axis_msg = messages.SetAxisStateMessage(node_id, axis_state=axis_state)
    await bus.send(axis_msg)


async def _set_velocity(
    bus: connection.CANSimple, motor: motor_config.MotorConfig, velocity: float
) -> None:
    """Sets velocity in turns/s"""
    signed_velocity = motor.direction * velocity
    vel_msg = messages.SetVelocityMessage(
        motor.node_id, velocity=signed_velocity, torque=0.0
    )
    await bus.send(vel_msg)


async def _listen_to_cyclic_traffic(bus: connection.CANSimple) -> None:
    bus.register_callbacks(
        (messages.EncoderEstimatesMessage, _async_print_msg),
        (messages.HeartbeatMessage, _async_print_msg),
    )
    await bus.listen()


async def _async_print_msg(msg: messages.OdriveCanMessage) -> None:
    print(msg)


async def _stop_all_motors(bus: connection.CANSimple) -> None:
    for motor in session.get_robot_motor_configs("beachbot-1"):
        await _set_velocity(bus, motor, 0.0)


async def main(bus: connection.CANSimple) -> None:
    try:
        for motor in session.get_robot_motor_configs("beachbot-1"):
            await _control_motor(bus, motor)
        # Print encoder feedback
        await _listen_to_cyclic_traffic(bus)
    except KeyboardInterrupt:
        pass
    finally:
        await _stop_all_motors(bus)
        bus.shutdown()


if __name__ == "__main__":
    bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)
    asyncio.run(main(bus))
