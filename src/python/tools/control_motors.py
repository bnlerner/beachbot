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
from odrive import enums as odrive_enums  # type: ignore[import-untyped]
from drivers.can import connection, enums, messages

_ALL_MOTORS = [
    motor_config.MotorConfig(node_id=0, location=motor_config.MotorLocation.FRONT_LEFT),
    motor_config.MotorConfig(node_id=1, location=motor_config.MotorLocation.FRONT_RIGHT),
    motor_config.MotorConfig(node_id=2, location=motor_config.MotorLocation.REAR_LEFT),
    motor_config.MotorConfig(node_id=3, location=motor_config.MotorLocation.REAR_RIGHT),
]


async def _control_motor(bus: connection.CANSimple, node_id:int) -> None:
    """Node ID must match `<odrv>.axis0.config.can.node_id`. The default is 0."""
    # Put axis into closed loop control state
    await _set_control_loop_state(bus, node_id)
    await _set_velocity(bus, node_id, 1.0)


async def _set_control_loop_state(bus: connection.CANSimple, node_id:int) -> None:
    axis_state = odrive_enums.AxisState.CLOSED_LOOP_CONTROL.value
    axis_msg = messages.SetAxisStateMessage(node_id, axis_state=axis_state)
    await bus.send(axis_msg)
    listener = connection.CANSimpleListener[messages.HeartbeatMessage]()
    # Wait for axis to enter closed loop control by scanning heartbeat messages
    while True:
        msg = await listener.get_message()
        if msg.node_id == node_id and msg.axis_state == axis_state:
            break


async def _set_velocity(bus: connection.CANSimple, node_id:int, velocity: float) -> None:
    """Sets velocity in turns/s"""
    vel_msg = messages.SetVelocityMessage(node_id, velocity=velocity)
    await bus.send(vel_msg)


async def _print_heartbeat(bus: connection.CANSimple) -> None:
    bus.register_callbacks((messages.HeartbeatMessage, _print_heartbeat_msg))
    await bus.listen()


def _print_heartbeat_msg(msg: messages.HeartbeatMessage) -> None:
    print(f"{msg.node_id=}, {msg.axis_error=}, {msg.procedure_result=}, {msg.axis_state=}, {msg.trajectory_done_flag=}")


async def _print_encoder_feedback(bus: connection.CANSimple) -> None:
    bus.register_callbacks((messages.EncoderEstimatesMessage, _print_encoder_data))
    await bus.listen()


def _print_encoder_data(msg: messages.EncoderEstimatesMessage) -> None:
    print(f"{msg.arbitration_id=}, pos: {msg.pos_estimate:.3f} [turns], vel: {msg.vel_estimate:.3f} [turns/s]")


async def _stop_all_motors(bus: connection.CANSimple) -> None:
    for motor in _ALL_MOTORS:
        await _set_velocity(bus, motor.node_id, 0.0)


async def main(bus: connection.CANSimple) -> None:
    try:
        for motor in _ALL_MOTORS:
            await _control_motor(bus, motor.node_id)
        # Print encoder feedback
        await _print_encoder_feedback(bus)
    except KeyboardInterrupt:
        pass
    finally:
        await _stop_all_motors(bus)

if __name__ == "__main__":
    bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)
    print("Starting motor control")
    asyncio.run(main(bus))
    bus.shutdown()
    print("Shutting down")