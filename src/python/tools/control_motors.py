"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""
import asyncio
from odrive import enums as odrive_enums  # type: ignore[import-untyped]
import sys
import os

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import motor_config
from drivers.can import connection, enums, messages
from ipc import session


async def _control_motor(bus: connection.CANSimple, motor: motor_config.MotorConfig) -> None:
    """Node ID must match `<odrv>.axis0.config.can.node_id`. The default is 0."""
    # Put axis into closed loop control state
    print("starting to control motor")
    await _set_control_loop_state(bus, motor.node_id)
    await asyncio.sleep(0.1)
    await _set_velocity(bus, motor, 1.0)


async def _set_control_loop_state(bus: connection.CANSimple, node_id:int) -> None:
    axis_state = odrive_enums.AxisState.CLOSED_LOOP_CONTROL.value
    axis_msg = messages.SetAxisStateMessage(node_id, axis_state=axis_state)
    print("sending control loop state msg")
    await bus.send(axis_msg)
    print("finished sending")


async def _set_velocity(bus: connection.CANSimple, motor: motor_config.MotorConfig, velocity: float) -> None:
    """Sets velocity in turns/s"""
    signed_velocity = motor.direction * velocity
    vel_msg = messages.SetVelocityMessage(motor.node_id, velocity=signed_velocity)
    await bus.send(vel_msg)


async def _print_heartbeat(bus: connection.CANSimple) -> None:
    bus.register_callbacks((messages.HeartbeatMessage, _print_heartbeat_msg))
    await bus.listen()


async def _print_heartbeat_msg(msg: messages.HeartbeatMessage) -> None:
    print(f"{msg.node_id=}, {msg.axis_error=}, {msg.procedure_result=}, {msg.axis_state=}, {msg.trajectory_done_flag=}")


async def _print_encoder_feedback(bus: connection.CANSimple) -> None:
    bus.register_callbacks((messages.EncoderEstimatesMessage, _print_encoder_data), (messages.HeartbeatMessage, _print_heartbeat_msg))
    await bus.listen()


async def _print_encoder_data(msg: messages.EncoderEstimatesMessage) -> None:
    print(f"{msg.arbitration_id=}, pos: {msg.pos_estimate:.3f} [turns], vel: {msg.vel_estimate:.3f} [turns/s]")


async def _stop_all_motors(bus: connection.CANSimple) -> None:
    for motor in session.get_robot_motor_configs("beachbot-1"):
        await _set_velocity(bus, motor, 0.0)


async def main(bus: connection.CANSimple) -> None:
    try:
        for motor in session.get_robot_motor_configs("beachbot-1"):
            await _control_motor(bus, motor)
        # Print encoder feedback
        await _print_encoder_feedback(bus)
    except KeyboardInterrupt:
        pass
    finally:
        await _stop_all_motors(bus)
        bus.shutdown()

if __name__ == "__main__":
    bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)
    print("Starting motor control")
    asyncio.run(main(bus))

    print("Shutting down")