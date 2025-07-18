"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""
import asyncio
import os
import sys

from odrive import enums as odrive_enums  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import robot_config
from drivers import can
from ipc import session


async def _control_motor(bus: can.CANSimple, motor: robot_config.Motor) -> None:
    """Node ID must match `<odrv>.axis0.config.can.node_id`. The default is 0."""
    # Put axis into closed loop control state
    await _set_control_loop_state(bus, motor.node_id)
    await asyncio.sleep(0.1)
    await _set_velocity(bus, motor, 1.0)


async def _set_control_loop_state(bus: can.CANSimple, node_id: int) -> None:
    axis_state = odrive_enums.AxisState.CLOSED_LOOP_CONTROL
    axis_msg = can.SetAxisStateMessage(node_id, axis_state=axis_state)
    await bus.send(axis_msg)


async def _set_velocity(
    bus: can.CANSimple, motor: robot_config.Motor, velocity: float
) -> None:
    """Sets velocity in turns/s"""
    direction = -1 if motor.side == "left" else 1
    signed_velocity = direction * velocity
    vel_msg = can.SetVelocityMessage(
        motor.node_id, velocity=signed_velocity, torque=0.0
    )
    await bus.send(vel_msg)


async def _listen_to_cyclic_traffic(bus: can.CANSimple) -> None:
    bus.register_callbacks(
        (can.EncoderEstimatesMessage, _async_print_msg),
        (can.HeartbeatMessage, _async_print_msg),
    )
    await bus.listen()


async def _async_print_msg(msg: can.CanMessage) -> None:
    print(msg)


async def _stop_all_motors(bus: can.CANSimple) -> None:
    for motor in session.get_robot_motors():
        await _set_velocity(bus, motor, 0.0)


async def main(bus: can.CANSimple) -> None:
    try:
        for motor in session.get_robot_motors():
            await _control_motor(bus, motor)
        # Print encoder feedback
        await _listen_to_cyclic_traffic(bus)
    except KeyboardInterrupt:
        pass
    finally:
        await _stop_all_motors(bus)
        bus.shutdown()


if __name__ == "__main__":
    bus = can.CANSimple(can.CANInterface.ODRIVE, can.BusType.SOCKET_CAN)
    asyncio.run(main(bus))
