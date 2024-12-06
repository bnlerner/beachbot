import asyncio
import json
import os
import sys
from typing import Dict, Any

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import system_info
from ipc import session
from config import robot_config
from drivers.can import connection, enums, messages

_FLAT_ENDPOINT_PATH = (
    system_info.get_root_project_directory() / "env/motor_configs/flat_endpoints.json"
)
_MOTOR_CONFIG_PATH = system_info.get_root_project_directory() / "env" / "motor_configs"


def _motor_config(motor: robot_config.Motor) -> Dict[str, Any]:
    file_path = _MOTOR_CONFIG_PATH / session.get_robot_name() / f"{motor.location.value.lower()}.json"

    with open(file_path, "r") as f:
        motor_config_dict = json.load(f)

    return motor_config_dict

async def _validate_motor_config(bus: connection.CANSimple, motor: robot_config.Motor) -> None:
    with open(_FLAT_ENDPOINT_PATH, "r") as fp:
        endpoint_data = json.load(fp)

    config = _motor_config(motor)
    for path, expected_value in config.items():
        endpoint_id = endpoint_data["endpoints"][path]["id"]
        endpoint_type = endpoint_data["endpoints"][path]["type"]
        if endpoint_type in ("function", "endpoint_ref"):
            raise ValueError(f"Unknown endpoint type {endpoint_type=}")

        msg = messages.ReadParameterCommand(motor.node_id, endpoint_id=endpoint_id)
        await bus.send(msg)
        response = await bus.await_parameter_response(motor.node_id, value_type=endpoint_type)
        if response is None or response.value != expected_value:
            print(f"{path} = {response.value if response else None}, expected: {expected_value}")


async def main() -> None:
    print("opening CAN bus...")
    bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)

    try:
        motors = session.get_robot_motors()
        for motor in motors:
            await _validate_motor_config(bus, motor)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
