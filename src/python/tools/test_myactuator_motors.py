#!/usr/bin/env python3

import argparse
import asyncio
import os
import sys
from typing import Dict

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers import can


async def _async_print_msg(msg: can.QAReturnMessage) -> None:
    print(msg)


async def discover_motors(
    can_interface: str, scan_range: range = range(1, 8)
) -> Dict[int, str]:
    """Discover MyActuator motors on the CAN bus."""
    discovered_motors: Dict[int, str] = {}

    # Initialize the CAN bus
    can_bus = can.CANSimple(
        can_interface=can.CANInterface(can_interface), bustype=can.BusType.SOCKET_CAN
    )

    # Define a callback for Controller V3 status messages
    async def on_controller_v3_status(msg: can.ReadMotorStatus1Message) -> None:
        motor_id = msg.node_id
        if motor_id not in discovered_motors:
            discovered_motors[motor_id] = "Controller V3"
            print(f"Discovered Controller V3 motor with ID {motor_id}")

    async def on_x4_status(msg: can.QueryCANCommunicationIDMessage) -> None:
        motor_id = msg.node_id
        print(f"Received X4-24 motor with ID {motor_id}")
        if motor_id not in discovered_motors:
            discovered_motors[motor_id] = "X4-24"
            print(f"Discovered X4-24 motor with ID {motor_id}")

    can_bus.register_callbacks(
        (can.ReadMotorStatus1Message, on_controller_v3_status),
        (can.QueryCANCommunicationIDMessage, on_x4_status),
    )

    print(f"Scanning CAN interface {can_interface} for motors...")

    listen_task = asyncio.create_task(can_bus.listen())
    try:
        print("Probing for motors...")
        # Try X4-24 protocol
        print("Probing for any X4-24 motors...")
        await can_bus.send(can.QueryCANCommunicationIDMessage(node_id=-1))
        await asyncio.sleep(0.5)

        # Send query messages to discover both types of motors
        for node_id in scan_range:
            # Try V3 controller protocol
            print(f"Probing for V3 controller motor with ID {node_id}...")
            await can_bus.send(can.ReadMotorStatus1Message(node_id=node_id))
            await asyncio.sleep(0.5)

    finally:
        # Cancel tasks
        listen_task.cancel()
        can_bus.shutdown()

    return discovered_motors


async def test_x4_motor(can_interface: str, node_id: int) -> None:
    """Test basic commands for X4-24 motor protocol."""
    # Initialize the CAN bus
    can_bus = can.CANSimple(
        can_interface=can.CANInterface(can_interface), bustype=can.BusType.SOCKET_CAN
    )

    print(f"Connected to CAN interface: {can_interface}")
    print(f"Testing X4-24 motor with ID: {node_id}")
    can_bus.register_callbacks(
        (can.QAReturnMessageType1, _async_print_msg),
        (can.QAReturnMessageType2, _async_print_msg),
        (can.QAReturnMessageType3, _async_print_msg),
        (can.QAReturnMessageType4, _async_print_msg),
    )

    try:
        # Set to qa message mode
        await can_bus.send(can.SetCommunicationModeMessage(node_id=node_id, mode="qa"))
        await asyncio.sleep(0.5)

        print("Testing position control (-90° → 90° → 0°)...")

        # Ensure at -90 degrees
        await can_bus.send(
            can.X424ServoPositionControlMessage(
                node_id=node_id, position=-90.0, speed=300, current_limit=5
            )
        )
        await asyncio.sleep(2.5)
        # Move to 90 degrees
        await can_bus.send(
            can.X424ServoPositionControlMessage(
                node_id=node_id, position=90.0, speed=300, current_limit=5
            )
        )
        await asyncio.sleep(2.0)

        # Return to 0 degrees
        await can_bus.send(
            can.X424ServoPositionControlMessage(
                node_id=node_id, position=0.0, speed=300, current_limit=5
            )
        )
        await asyncio.sleep(2.0)

        # print("Testing speed control (-100 -> 0 -> 100 -> 0)...")

        # # Rotate at -300 RPM
        # await can_bus.send(
        #     x4_24_messages.X424ServoSpeedControlMessage(
        #         node_id=node_id,
        #         speed=-100,
        #         current_limit=5,
        #     )
        # )
        # await asyncio.sleep(2.0)

        # # Stop the motor
        # await can_bus.send(
        #     x4_24_messages.X424ServoSpeedControlMessage(
        #         node_id=node_id,
        #         speed=0,
        #         current_limit=5,
        #     )
        # )
        # await asyncio.sleep(0.5)

        # # Rotate at 300 RPM
        # await can_bus.send(
        #     x4_24_messages.X424ServoSpeedControlMessage(
        #         node_id=node_id,
        #         speed=100,
        #         current_limit=5,
        #     )
        # )
        # await asyncio.sleep(2.0)

        # # Stop the motor
        # await can_bus.send(
        #     x4_24_messages.X424ServoSpeedControlMessage(
        #         node_id=node_id,
        #         speed=0,
        #         current_limit=5,
        #     )
        # )
        # await asyncio.sleep(0.5)

    finally:
        can_bus.shutdown()


async def test_controller_v3_motor(can_interface: str, node_id: int) -> None:
    """Test basic commands for Controller V3 motor protocol."""
    # Initialize the CAN bus
    can_bus = can.CANSimple(
        can_interface=can.CANInterface(can_interface), bustype=can.BusType.SOCKET_CAN
    )

    print(f"Connected to CAN interface: {can_interface}")
    print(f"Testing Controller V3 motor with ID: {node_id}")

    # Define callbacks for Controller V3 protocol
    async def on_status_msg(msg: can.ReadMotorStatus1Message) -> None:
        print(
            f"Status: Temp={msg.temperature}°C, Voltage={msg.voltage:.1f}V, Error=0x{msg.error_state:04x}"
        )

    async def on_angle_msg(msg: can.ReadMultiTurnAngleMessage) -> None:
        print(f"Angle: {msg.angle:.2f}°")

    # Register callbacks for specific message types
    can_bus.register_callbacks(
        (can.ReadMotorStatus1Message, on_status_msg),
        (can.ReadMultiTurnAngleMessage, on_angle_msg),
    )

    # Start the listener task
    listen_task = asyncio.create_task(can_bus.listen())

    try:
        # Basic Controller V3 test sequence
        print("Testing Controller V3 commands...")

        # Position control test (0° → 90° → 0°)
        print("Testing position control (0° → 90° → 0°)...")

        # Release brake
        await can_bus.send(can.SystemBrakeReleaseCommand(node_id=node_id))
        await asyncio.sleep(0.5)

        # Move to 90 degrees
        await can_bus.send(
            can.PositionControlCommand(node_id=node_id, position=90.0, max_speed=500)
        )
        await asyncio.sleep(2.0)

        # Read current angle
        await can_bus.send(can.ReadMultiTurnAngleMessage(node_id=node_id))
        await asyncio.sleep(0.1)

        # Return to 0 degrees
        await can_bus.send(
            can.PositionControlCommand(node_id=node_id, position=0.0, max_speed=500)
        )
        await asyncio.sleep(2.0)

        await can_bus.send(can.MotorShutdownCommand(node_id=node_id))
        await asyncio.sleep(0.5)

    finally:
        listen_task.cancel()
        can_bus.shutdown()


async def test_motor(can_interface: str, node_id: int, motor_type: str) -> None:
    """Test basic motor commands using either X4-24 or Controller V3 protocol."""
    if motor_type == "X4-24":
        await test_x4_motor(can_interface, node_id)
    else:
        await test_controller_v3_motor(can_interface, node_id)


async def main(discover: bool, test: bool) -> None:
    """Main function to parse arguments and run the appropriate test."""

    # Discover motors if requested or needed
    discovered_motors = {}
    if discover or test:
        discovered_motors = await discover_motors("can0")

    if len(discovered_motors) != 0 and test:
        for motor_id, motor_type in discovered_motors.items():
            await test_motor("can0", motor_id, motor_type)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test MyActuator motors via CAN")
    parser.add_argument(
        "--discover", "-d", action="store_true", help="Discover connected motors"
    )
    parser.add_argument(
        "--test", "-a", action="store_true", help="Discover and test all motors"
    )
    args = parser.parse_args()

    asyncio.run(main(args.discover, args.test))
