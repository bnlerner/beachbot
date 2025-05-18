import argparse
import asyncio
import os
import sys

from odrive import enums as odrive_enums  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers import can


async def _set_state(
    bus: can.CANSimple, node_id: int, axis_state: odrive_enums.AxisState
) -> None:
    msg = can.SetAxisStateMessage(node_id=node_id, axis_state=axis_state)
    await bus.send(msg)


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Script to read the configured ODrive over CAN bus."
    )
    parser.add_argument(
        "--node-id", type=int, required=True, help="CAN Node ID of the ODrive."
    )
    parser.add_argument(
        "--state",
        type=str,
        required=True,
        choices=["idle", "running"],
        help="""The axis state the motor should be in""",
    )
    args = parser.parse_args()
    if args.state == "idle":
        axis_state = odrive_enums.AxisState.IDLE
    elif args.state == "running":
        axis_state = odrive_enums.AxisState.CLOSED_LOOP_CONTROL
    else:
        raise ValueError("Unknown state.")

    print("opening CAN bus...")
    bus = can.CANSimple(can.CANInterface.ODRIVE, can.BusType.SOCKET_CAN)

    try:
        await _set_state(bus, args.node_id, axis_state)
    finally:
        bus.shutdown()


if __name__ == "__main__":
    asyncio.run(main())
