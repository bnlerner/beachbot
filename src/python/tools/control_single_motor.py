"""
Minimal example for controlling an ODrive via the CANSimple protocol.

Puts the ODrive into closed loop control mode, sends a velocity setpoint of 1.0
and then prints the encoder feedback.

Assumes that the ODrive is already configured for velocity control.

See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html for protocol
documentation.
"""
from typing import Tuple
import can
import struct

def _control_motor(bus: can.interface.Bus, node_id:int) -> None:
    """Node ID must match `<odrv>.axis0.config.can.node_id`. The default is 0."""
    # Put axis into closed loop control state
    _set_control_loop_state(bus, node_id)
    _set_velocity(bus, node_id, 1.0)
    # Print encoder feedback
    _print_encoder_feedback(bus, node_id)


def _flush_bus(bus: can.interface.Bus) -> None:
    # Flush CAN RX buffer so there are no more old pending messages
    while not (bus.recv(timeout=0) is None):
        pass


def _set_control_loop_state(bus: can.interface.Bus, node_id:int) -> None:
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x07), # 0x07: Set_Axis_State
        data=struct.pack('<I', 8), # 8: AxisState.CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    # Wait for axis to enter closed loop control by scanning heartbeat messages
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x01): # 0x01: Heartbeat
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if state == 8: # 8: AxisState.CLOSED_LOOP_CONTROL
                break


def _set_velocity(bus: can.interface.Bus, node_id:int, velocity: float) -> None:
    """Sets velocity in turns/s"""
    bus.send(can.Message(
        arbitration_id=(node_id << 5 | 0x0d), # 0x0d: Set_Input_Vel
        data=struct.pack('<ff', velocity, 0.0), # 1.0: velocity, 0.0: torque feedforward
        is_extended_id=False
    ))


def _print_encoder_feedback(bus: can.interface.Bus, node_id:int) -> None:
    for msg in bus:
        if msg.arbitration_id == (node_id << 5 | 0x09): # 0x09: Get_Encoder_Estimates
            pos, vel = struct.unpack('<ff', bytes(msg.data))
            print(f"pos: {pos:.3f} [turns], vel: {vel:.3f} [turns/s]")


if __name__ == "__main__":
    bus = can.interface.Bus("can0", bustype="socketcan")
    node_id = 1
    _flush_bus(bus)
    try:
        print("Starting motor control")
        _control_motor(bus, node_id)
    except KeyboardInterrupt:
        _set_velocity(bus, node_id, 0.0)
        bus.shutdown()

    print("Shutting down")