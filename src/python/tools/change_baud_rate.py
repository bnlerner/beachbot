import asyncio
import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import robot_config
from drivers import can
from ipc import session


async def _write_baud_rate(bus: can.CANSimple, motor: robot_config.Motor) -> None:
    endpoint_data = session.get_motor_endpoint_data()
    config = session.get_motor_config(motor)
    path = "can.config.baud_rate"
    endpoint_id = endpoint_data["endpoints"][path]["id"]
    endpoint_type = endpoint_data["endpoints"][path]["type"]
    expected_value = config[path]
    write_msg = can.WriteParameterCommand(
        motor.node_id,
        endpoint_id=endpoint_id,
        value_type=endpoint_type,
        value=expected_value,
    )
    await bus.send(write_msg)
    print(f"Writing config value for motor: {motor.node_id}: {path} = {expected_value}")

    reboot_msg = can.Reboot(motor.node_id, action=1)
    await bus.send(reboot_msg)
    print(f"Rebooting motor: {motor.node_id}")


async def main() -> None:
    print("opening CAN bus...")
    bus = can.CANSimple(can.CANInterface.ODRIVE, can.BusType.SOCKET_CAN)

    try:
        motors = session.get_robot_motors()
        for motor in motors:
            await _write_baud_rate(bus, motor)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
