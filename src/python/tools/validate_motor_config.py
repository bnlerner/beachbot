import argparse
import asyncio
import os
import sys

import numpy as np

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import robot_config
from drivers import can
from ipc import session


async def _validate_motor_config(
    bus: can.CANSimple, motor: robot_config.Motor, should_fix_issues: bool
) -> None:
    endpoint_data = session.get_motor_endpoint_data()

    did_write_config = False

    config = session.get_motor_config(motor)
    for path, expected_value in config.items():
        endpoint_id = endpoint_data["endpoints"][path]["id"]
        endpoint_type = endpoint_data["endpoints"][path]["type"]
        if endpoint_type in ("function", "endpoint_ref"):
            raise ValueError(f"Unknown endpoint type {endpoint_type=}")

        msg = can.ReadParameterCommand(motor.node_id, endpoint_id=endpoint_id)
        await bus.send(msg)
        response = await bus.await_parameter_response(
            motor.node_id, value_type=endpoint_type
        )
        if response is None or not np.isclose(response.value, expected_value):
            print(
                f"For motor: {motor.node_id}: {path} = {response.value if response else None}, expected: {expected_value}"
            )
            if should_fix_issues:
                write_msg = can.WriteParameterCommand(
                    motor.node_id,
                    endpoint_id=endpoint_id,
                    value_type=endpoint_type,
                    value=expected_value,
                )
                await bus.send(write_msg)
                print(
                    f"Writing config value for motor: {motor.node_id}: {path} = {expected_value}"
                )
                did_write_config = True

    if did_write_config:
        reboot_msg = can.Reboot(motor.node_id, action=1)
        await bus.send(reboot_msg)
        print(f"Rebooting motor: {motor.node_id}")


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Script to validate the configured ODrive over CAN bus and fix issues."
    )
    parser.add_argument(
        "--fix-issues",
        action="store_true",
        help="Use this option to fix issues that are in the config",
    )
    args = parser.parse_args()

    print("opening CAN bus...")
    bus = can.CANSimple(can.CANInterface.ODRIVE, can.BusType.SOCKET_CAN)

    try:
        motors = session.get_robot_motors()
        for motor in motors:
            await _validate_motor_config(bus, motor, args.fix_issues)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
