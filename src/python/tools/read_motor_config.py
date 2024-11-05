import argparse
import asyncio
import json
import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import system_info
from drivers.can import connection, enums, messages

_FLAT_ENDPOINT_PATH = (
    system_info.get_root_project_directory() / "env/motor_configs/flat_endpoints.json"
)


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Script to read the configured ODrive over CAN bus."
    )
    parser.add_argument(
        "--node-id", type=int, required=True, help="CAN Node ID of the ODrive."
    )
    parser.add_argument(
        "--path",
        type=str,
        required=True,
        help="""The path to the specified parameter to read. \n For example reading the CAN cyclic message showing power is 'axis0.config.can.powers_msg_rate_ms'""",
    )
    args = parser.parse_args()

    with open(_FLAT_ENDPOINT_PATH, "r") as fp:
        endpoint_data = json.load(fp)

    print("opening CAN bus...")
    bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)
    bus.register_callbacks()

    try:
        endpoint_id = endpoint_data["endpoints"][args.path]["id"]
        endpoint_type = endpoint_data["endpoints"][args.path]["type"]
        if endpoint_type in ("function", "endpoint_ref"):
            raise ValueError(f"Unknown endpoint type {endpoint_type=}")

        msg = messages.ReadParameterCommand(args.node_id, endpoint_id=endpoint_id)
        await bus.send(msg)
        response = await bus.await_parameter_response(
            args.node_id, value_type=endpoint_type
        )
        print(f"{args.path} = {response.value if response else None}")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
