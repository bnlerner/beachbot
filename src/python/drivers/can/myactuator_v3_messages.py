from __future__ import annotations

import datetime
from typing import Literal, Type, TypeVar, cast

import can
from geometry import math_helpers

from drivers.can import enums, messages

MyActuatorCanMessageT = TypeVar("MyActuatorCanMessageT", bound="MyActuatorCanMessage")


class MyActuatorCanMessage(messages.CanMessage):
    """A class representing different types of MyActuator CAN messages that can be
    sent to the node and received from the node.

    Protocol specification from motor_protocol.md:
    - Single motor command sending: 0x140+ID (1~32)
    - Multi-motor command sending: 0x280 (not implemented yet)
    - Reply: 0x240+ID (1~32)
    - DLC: 8 bytes (always)

    Message format:
    DATA[0] = Command byte
    DATA[1] to DATA[7] = Command-specific data
    """

    @property
    def arbitration_id(self) -> messages.MyActuatorArbitrationID:
        return cast(messages.MyActuatorArbitrationID, self._arbitration_id)

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        # For MyActuator, we match based on command byte in the data
        if 0x140 <= msg.arbitration_id < 0x160 or 0x240 <= msg.arbitration_id < 0x260:
            return msg.data[0] == cls.cmd_id
        return False

    @classmethod
    def from_can_message(
        cls: Type[MyActuatorCanMessageT], msg: can.Message
    ) -> MyActuatorCanMessageT:
        if not cls.matches(msg):
            raise ValueError(
                f"CAN message does not match the class desired {cls.__name__}!"
            )

        # For MyActuator, extract node_id from arbitration_id
        if 0x140 <= msg.arbitration_id < 0x160:
            node_id = msg.arbitration_id - 0x140
        elif 0x240 <= msg.arbitration_id < 0x260:
            node_id = msg.arbitration_id - 0x240
        else:
            raise ValueError(f"Invalid MyActuator arbitration ID: {msg.arbitration_id}")

        message = cls(node_id=node_id)
        message._parse_can_msg_data(msg)
        return message

    def _gen_arbitration_id(self) -> messages.MyActuatorArbitrationID:
        return messages.MyActuatorArbitrationID(
            node_id=self.node_id, cmd_id=self.cmd_id
        )

    def _gen_can_msg_data(self) -> bytes:
        """Generate 8-byte data array for MyActuator message format."""
        # Default implementation - command byte followed by zeros
        # Override in subclasses for specific message types
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


########################################################################################
# MYACTUATOR MESSAGES #################################################################
########################################################################################


class MyactuatorReadMotorStatus1Message(MyActuatorCanMessage):
    """Reads the current motor temperature, voltage and error status flags."""

    cmd_id = 0x9A

    # Motor temperature (int8_t type, unit 1°C/LSB)
    temperature: int
    # Brake release command (1 = brake released, 0 = brake locked)
    brake_released: bool
    # Voltage (uint16_t type, unit 0.1V/LSB)
    voltage: float
    # Error flags (uint16_t type, bits represent different motor states)
    error_state: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.temperature = msg.data[1]
        self.brake_released = bool(msg.data[3])

        # Voltage in DATA[4] (low byte) and DATA[5] (high byte)
        voltage_raw = (msg.data[5] << 8) | msg.data[4]
        self.voltage = voltage_raw * 0.1  # 0.1V/LSB

        # Error state in DATA[6] (low byte) and DATA[7] (high byte)
        self.error_state = (msg.data[7] << 8) | msg.data[6]


class ReadMotorStatus2Message(MyActuatorCanMessage):
    """Reads the temperature, speed and encoder position of the current motor."""

    cmd_id = 0x9C

    # Motor temperature (int8_t type, 1°C/LSB)
    temperature: int
    # Torque current value (int16_t type, 0.01A/LSB)
    torque_current: float
    # Motor output shaft speed (int16_t type, 1dps/LSB)
    speed: int
    # Motor output shaft angle (int16_t type, 1degree/LSB, range ±32767 degree)
    angle: int

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.temperature = msg.data[1]

        # Torque current in DATA[2] (low byte) and DATA[3] (high byte)
        current_raw = (msg.data[3] << 8) | msg.data[2]
        if current_raw > 32767:  # Handle negative values (2's complement)
            current_raw -= 65536
        self.torque_current = current_raw * 0.01  # 0.01A/LSB

        # Speed in DATA[4] (low byte) and DATA[5] (high byte)
        speed_raw = (msg.data[5] << 8) | msg.data[4]
        if speed_raw > 32767:  # Handle negative values (2's complement)
            speed_raw -= 65536
        self.speed = speed_raw

        # Angle in DATA[6] (low byte) and DATA[7] (high byte)
        angle_raw = (msg.data[7] << 8) | msg.data[6]
        if angle_raw > 32767:  # Handle negative values (2's complement)
            angle_raw -= 65536
        self.angle = angle_raw


class WriteMotorZeroPositionMessage(MyActuatorCanMessage):
    """Write the current encoder position of the motor as the multi-turn encoder zero offset
    (initial position) into the ROM.

    Note: After writing the new zero point position,you need to send 0x76 (system reset
    command) to restart the system to be effective. Because of the change of the zero
    offset,the new zero offset (initial position) should be used as a reference when setting
    the target position.
    """

    cmd_id = 0x64

    def _gen_can_msg_data(self) -> bytes:
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


class TorqueControlCommand(MyActuatorCanMessage):
    """Command to control the torque and current output of the motor."""

    cmd_id = 0xA1

    # Torque current control value (int16_t type, 0.01A/LSB)
    torque_current: float

    def _gen_can_msg_data(self) -> bytes:
        # Convert torque_current to int16_t value
        torque_raw = int(self.torque_current * 100)  # Convert to 0.01A/LSB

        # Clamp to int16_t range
        torque_raw = max(-32768, min(32767, torque_raw))

        # Create data bytes - cmd_id followed by zeros, then torque value in bytes 4-5
        return bytes(
            [self.cmd_id, 0, 0, 0, torque_raw & 0xFF, (torque_raw >> 8) & 0xFF, 0, 0]
        )


class SpeedControlCommand(MyActuatorCanMessage):
    """Command to control the speed of the motor output shaft."""

    cmd_id = 0xA2

    # Speed control value (int32_t type, 0.01dps/LSB)
    speed: float

    def _gen_can_msg_data(self) -> bytes:
        # Convert speed to int32_t value (0.01dps/LSB)
        speed_raw = int(self.speed * 100)

        # Create data bytes - cmd_id followed by zeros, then speed value in bytes 4-7
        return bytes(
            [
                self.cmd_id,
                0,
                0,
                0,
                speed_raw & 0xFF,
                (speed_raw >> 8) & 0xFF,
                (speed_raw >> 16) & 0xFF,
                (speed_raw >> 24) & 0xFF,
            ]
        )


class PositionControlCommand(MyActuatorCanMessage):
    """Command to control the absolute position of the motor."""

    cmd_id = 0xA4

    # Position control value (int32_t type, 0.01degree/LSB)
    position: float
    # Max speed (uint16_t type, 1dps/LSB)
    max_speed: int

    def _gen_can_msg_data(self) -> bytes:
        # Convert position to int32_t value (0.01degree/LSB)
        position_raw = int(self.position * 100)

        # Create data bytes
        return bytes(
            [
                self.cmd_id,
                0,
                self.max_speed & 0xFF,  # Low byte of max_speed
                (self.max_speed >> 8) & 0xFF,  # High byte of max_speed
                position_raw & 0xFF,
                (position_raw >> 8) & 0xFF,
                (position_raw >> 16) & 0xFF,
                (position_raw >> 24) & 0xFF,
            ]
        )


class IncrementalPositionControlCommand(MyActuatorCanMessage):
    """Command to control the incremental position of the motor. The host sends this
    command to control the incremental position (multi-turn angle) of
    the motor,and run the input position increment with the current position as the starting
    point.
    """

    cmd_id = 0xA8

    # Max speed (uint16_t type, 1dps/LSB)
    max_speed: int
    # Incremental position control value (int32_t type, 0.01degree/LSB)
    # angleControl is of type int32_t, and the corresponding actual
    # position is 0.01degree/LSB, that is, 36000 represents 360°, and the rotation
    # direction of the motor is determined by the incremental position symbol
    position_increment: float

    def _gen_can_msg_data(self) -> bytes:
        """Data field Description Data
        DATA[0] Command byte 0xA8
        DATA[1] NULL 0x00
        DATA[2] Speed limit low byte DATA[2] = (uint8
        _
        t)(maxSpeed)
        DATA[3] Speed limit high byte DATA[3] = (uint8
        _
        t)(maxSpeed>>8)
        DATA[4] Position control low byte DATA[4] = (uint8
        _
        t)(angleControl)
        DATA[5] Position control DATA[5] = (uint8
        _
        t)(angleControl>>8)
        DATA[6] Position control DATA[6] = (uint8
        _
        t)(angleControl>>16)
        DATA[7] Position control high byte DATA[7] = (uint8
        t)(angleControl>>24)
        """
        speed_low_byte = self.max_speed & 0xFF
        speed_high_byte = (self.max_speed >> 8) & 0xFF

        position_raw = int(self.position_increment * 100)
        position_low_byte = position_raw & 0xFF
        position_byte_1 = (position_raw >> 8) & 0xFF
        position_byte_2 = (position_raw >> 16) & 0xFF
        position_byte_high_byte = (position_raw >> 24) & 0xFF

        # Create data bytes
        return bytes(
            [
                self.cmd_id,
                0,
                speed_low_byte,
                speed_high_byte,
                position_low_byte,
                position_byte_1,
                position_byte_2,
                position_byte_high_byte,
            ]
        )


class MotorShutdownCommand(MyActuatorCanMessage):
    """Turns off the motor output and clears the motor running state."""

    cmd_id = 0x80

    def _gen_can_msg_data(self) -> bytes:
        # Simply the command byte followed by zeros
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


class MotorStopCommand(MyActuatorCanMessage):
    """Stops the motor but maintains closed-loop control mode."""

    cmd_id = 0x81

    def _gen_can_msg_data(self) -> bytes:
        # Simply the command byte followed by zeros
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


class ReadMultiTurnAngleMessage(MyActuatorCanMessage):
    """Reads the current multi-turn absolute angle value of the motor."""

    cmd_id = 0x92

    # Motor angle (int32_t type, unit 0.01°/LSB)
    angle: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        # Angle in DATA[4] to DATA[7], unit 0.01°/LSB
        angle_raw = (
            msg.data[4] | (msg.data[5] << 8) | (msg.data[6] << 16) | (msg.data[7] << 24)
        )

        # Convert to signed if needed (32-bit two's complement)
        if angle_raw > 0x7FFFFFFF:
            angle_raw -= 0x100000000

        self.angle = angle_raw * 0.01  # Convert to degrees


class SystemBrakeReleaseCommand(MyActuatorCanMessage):
    """Releases the system brake, allowing the motor to be moved."""

    cmd_id = 0x77

    def _gen_can_msg_data(self) -> bytes:
        # Simply the command byte followed by zeros
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


class SystemBrakeLockCommand(MyActuatorCanMessage):
    """Locks the system brake, preventing the motor from moving."""

    cmd_id = 0x78

    def _gen_can_msg_data(self) -> bytes:
        # Simply the command byte followed by zeros
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


class SystemOperatingModeAcquisitionCommand(MyActuatorCanMessage):
    """Acquires the current operating mode of the system."""

    cmd_id = 0x70
    operating_mode: enums.MyActuatorV3OperatingMode

    def _gen_can_msg_data(self) -> bytes:
        # Simply the command byte followed by zeros
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        operating_mode_byte = msg.data[7]
        self.operating_mode = enums.MyActuatorV3OperatingMode(operating_mode_byte)


class SystemResetCommand(MyActuatorCanMessage):
    """Resets the system program."""

    cmd_id = 0x76

    def _gen_can_msg_data(self) -> bytes:
        # Simply the command byte followed by zeros
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])


class VersionAcquisitionCommand(MyActuatorCanMessage):
    cmd_id = 0xB2
    version_date: int

    def version_datetime(self) -> datetime.datetime:
        """Takes the version date in the format of year, month and day, such as an int
        20220206 to Feb 6, 2022 and returns a datetime object.
        """
        return datetime.datetime.strptime(str(self.version_date), "%Y%m%d")

    def _gen_can_msg_data(self) -> bytes:
        return bytes([self.cmd_id, 0, 0, 0, 0, 0, 0, 0])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """The motor will reply to the host after receiving the command. The driver
        reply data contains the latest version date of the system software, VersionDate,
        which is of type uint32_t. The date format is in the format of year, month and day,
        such as 20211126.

        DATA[4] VersionDate low byte 1 DATA[4] = (uint8_t)(&VersionDate)
        DATA[5] VersionDate byte 2 DATA[5] = (uint8_t)(VersionDate>>8)
        DATA[6] VersionDate byte 3 DATA[6] = (uint8_t)(VersionDate>>16)
        DATA[7] VersionDate byte 4 DATA[7] = (uint8_t)(VersionDate>>24)
        """
        version_date_raw = (
            (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4]
        )
        self.version_date = version_date_raw


class CANIDCommand(MyActuatorCanMessage):
    """This command is used to set and read CANID. The host sends this command to set and
    read the CAN ID, the parameters are as follows:
    1. The read and write flag bit wReadWriteFlag is bool type, 1 read 0 write;
    2. CANID, size range (#1~#32), uint16_t type (synchronized with the upper computer
    function), device identifier 0x140 + ID (1~32).

    Protocol format from manual:
    - ID: 0x300 (fixed arbitration ID for this command)
    - DATA[0]: 0x79 (command byte)
    - DATA[2]: 0x00 for write, 0x01 for read
    - DATA[7]: New CAN ID value (1-32)

    NOTE: This command could be affected by the CANID filter. Ensure the CANID filter is
    disabled before using this command.
    """

    cmd_id = 0x79

    # Read and write flag bit as an enum (actual used is of bool type, 1 read 0 write)
    read_write_flag: Literal["read", "write"]
    # CAN ID (uint16_t type, size range (#1~#32), synchronized with the upper computer function)
    can_id: int

    def _gen_arbitration_id(self) -> messages.MyActuatorArbitrationID:
        # Override to use fixed arbitration ID of 0x300 for this command
        # Use the standard construction but set a custom value
        arbitration_id = messages.MyActuatorArbitrationID(
            node_id=self.node_id, cmd_id=self.cmd_id, custom_value=0x300
        )
        return arbitration_id

    def _gen_can_msg_data(self) -> bytes:
        """Data field Description Data
        DATA[0] Command byte 0x79
        DATA[1] NULL 0x00
        DATA[2] Read and write flags DATA[2] = wReadWriteFlag (0 for write, 1 for read)
        DATA[3] NULL 0x00
        DATA[4] NULL 0x00
        DATA[5] NULL 0x00
        DATA[6] NULL 0x00
        DATA[7] CANID DATA[7] = CANID(1~32)
        """
        read_write_flag_byte = 0x01 if self.read_write_flag == "read" else 0x00
        clipped_can_id = int(math_helpers.clip(self.can_id, 1, 32))
        return bytes([self.cmd_id, 0, read_write_flag_byte, 0, 0, 0, 0, clipped_can_id])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self.read_write_flag = "read" if bool(msg.data[1]) else "write"


class FunctionControlCommand(MyActuatorCanMessage):
    """Function Control Command (0x20) for configuring controller functions.

    This command sets various controller operating parameters defined in section 2.34.4
    of the MyActuator Controller V4.2 protocol, such as:
    - Clear multi-turn value and update zero point
    - Enable/disable CANID filter
    - Enable/disable error status transmission
    - Configure multi-turn save on power off
    - Set motor CANID
    - Set maximum positive/negative angle limits

    Function indices are defined in enums.MyActuatorFunctionControlIndex.

    Data format:
    DATA[0] = Command byte (0x20)
    DATA[1] = Function index
    DATA[2-3] = Reserved (NULL)
    DATA[4-7] = Function value (4 bytes, little-endian)
    """

    cmd_id = 0x20

    # Function parameter to modify
    function: enums.MyActuatorFunctionControlIndex
    # Value to set for the specified function
    function_value: int

    def _gen_can_msg_data(self) -> bytes:
        """Generate the 8-byte data array for the function control command.

        According to the protocol document:
        DATA[0] = Command byte (0x20)
        DATA[1] = Function index (uint8_t)
        DATA[2-3] = NULL bytes
        DATA[4-7] = Function value (int32_t, little-endian)
        """
        return bytes(
            [
                self.cmd_id,  # Command byte
                self.function.value,  # Function index
                0x00,  # NULL
                0x00,  # NULL
                self.function_value & 0xFF,  # Value byte 0 (LSB)
                (self.function_value >> 8) & 0xFF,  # Value byte 1
                (self.function_value >> 16) & 0xFF,  # Value byte 2
                (self.function_value >> 24) & 0xFF,  # Value byte 3 (MSB)
            ]
        )
