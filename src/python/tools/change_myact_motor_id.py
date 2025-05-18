#!/usr/bin/env python3
"""Tool to change MyActuator V3 controller motor ID."""

import argparse
import asyncio
import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers import can
from drivers.can import enums


async def change_motor_id(current_id: int, new_id: int) -> bool:
    """Change the motor ID of a MyActuator V3 controller.

    Args:
        current_id: Current motor ID
        new_id: New motor ID to set

    Returns:
        True if successful, False otherwise
    """
    print("Connecting to CAN interface can0...")
    can_bus = can.CANSimple(
        can_interface=can.CANInterface.ODRIVE, bustype=can.BusType.SOCKET_CAN
    )

    # Flag to track successful response
    success = False

    # Define callback for status messages
    async def on_status_msg(msg: can.CanMessage) -> None:
        nonlocal success
        if msg.node_id == new_id:
            print(f"✓ Motor ID successfully changed to {new_id}, {msg=}")
            success = True
        else:
            print(f"Motor ID not changed to {new_id}, {msg=}")

    # Register callback
    can_bus.register_callbacks((can.ReadMotorStatus1Message, on_status_msg))

    # Start the listener task
    listen_task = asyncio.create_task(can_bus.listen())

    try:
        print(f"Attempting to change motor ID from {current_id} to {new_id}...")

        # Send ID change command
        await can_bus.send(
            can.FunctionControlCommand(
                node_id=current_id,
                function=enums.MyActuatorFunctionControlIndex.SET_CANID,
                function_value=new_id,
            )
        )

        # Wait briefly for command to take effect
        await asyncio.sleep(2.5)

        # Send a system reset command to apply the change
        print("Sending system reset command to apply the change...")
        await can_bus.send(can.SystemResetCommand(node_id=current_id))

        # Wait for the motor to restart
        await asyncio.sleep(2.0)

        # Check if motor responds with new ID
        print(f"Checking if motor responds with new ID {new_id}...")
        await can_bus.send(can.ReadMotorStatus1Message(node_id=new_id))
        await asyncio.sleep(2.5)
        print(f"Checking if motor responds with old ID {current_id}...")
        await can_bus.send(can.ReadMotorStatus1Message(node_id=current_id))

        # Wait for response
        timeout = 3.0
        print(f"Waiting up to {timeout} seconds for confirmation...")
        start_time = asyncio.get_event_loop().time()

        while not success and (asyncio.get_event_loop().time() - start_time) < timeout:
            await asyncio.sleep(0.1)

        if not success:
            print("⚠ Failed to detect motor with new ID")
            print(
                "Try checking with both the old and new IDs to verify the change status."
            )

    except Exception as e:
        print(f"Error: {e}")
        return False
    finally:
        # Clean up
        listen_task.cancel()
        can_bus.shutdown()

    return success


async def main(current_id: int, new_id: int) -> int:
    """Run the main program."""
    try:
        if current_id == new_id:
            print("Error: Current and new IDs must be different")
            return 1

        if not (1 <= current_id <= 32) or not (1 <= new_id <= 32):
            print("Error: Motor IDs must be between 1 and 32")
            return 1

        success = await change_motor_id(current_id, new_id)

        return 0 if success else 1

    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        return 130
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Change MyActuator V3 controller motor ID"
    )
    parser.add_argument(
        "--current-id", "-c", type=int, required=True, help="Current motor ID"
    )
    parser.add_argument(
        "--new-id", "-n", type=int, required=True, help="New motor ID to set"
    )

    args = parser.parse_args()
    sys.exit(asyncio.run(main(args.current_id, args.new_id)))
