import argparse
import asyncio
import os
import sys
from typing import List

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers.can import connection, enums, messages
from ipc import session


async def _print_path(
    bus: connection.CANSimple, node_ids: List[int], path: str
) -> None:
    endpoint_data = session.get_motor_endpoint_data()

    endpoint_id = endpoint_data["endpoints"][path]["id"]
    endpoint_type = endpoint_data["endpoints"][path]["type"]
    if endpoint_type in ("function", "endpoint_ref"):
        raise ValueError(f"Unknown endpoint type {endpoint_type=}")

    for node_id in node_ids:
        msg = messages.ReadParameterCommand(node_id, endpoint_id=endpoint_id)
        await bus.send(msg)
        response = await bus.await_parameter_response(node_id, value_type=endpoint_type)
        print(f"{node_id=}: {path} = {response.value if response else None}")


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Script to read the configured ODrive over CAN bus."
    )
    parser.add_argument(
        "node_id", nargs="*", type=int, help="CAN Node IDs of the ODrive."
    )
    parser.add_argument(
        "--path",
        type=str,
        required=True,
        help="""The path to the specified parameter to read. \n For example reading the
        CAN cyclic message showing power is 'axis0.config.can.powers_msg_rate_ms'
        """,
    )
    args = parser.parse_args()

    print("opening CAN bus...")
    bus = connection.CANSimple(enums.CANInterface.ODRIVE, enums.BusType.SOCKET_CAN)

    try:
        await _print_path(bus, args.node_id, args.path)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
