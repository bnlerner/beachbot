import struct
from typing import Literal, Type, TypeVar, cast

import can

from drivers.can import messages

X424CanMessageT = TypeVar("X424CanMessageT", bound="X424CanMessage")


########################################################################################
# X4-24 SPECIFIC MESSAGES #############################################################
########################################################################################


class X424CanMessage(messages.CanMessage):
    """Base class for X4-24 protocol messages.
    X4-24 uses a different protocol than X6-S2 but shares basic structure.
    """

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        # For X4-24 special commands with ID is the node ID
        node_id = (msg.arbitration_id >> 8) & 0xFF
        if node_id == cls.node_id:
            return msg.data[0] == cls.cmd_id

        return False

    @classmethod
    def from_can_message(
        cls: Type[X424CanMessageT], msg: can.Message
    ) -> X424CanMessageT:
        if not cls.matches(msg):
            raise ValueError(
                f"CAN message does not match the class desired {cls.__name__}!"
            )

        # Control / Q&A return frames use the motor ID as the 11-bit arbitration
        # ID (1..32). Setup frames use 0x7FF (node filled in by parser).
        # Automatic cyclic feedback uses 0x204+ID (handled by subclass override).
        if msg.arbitration_id == 0x7FF:
            node_id = 0
        elif msg.arbitration_id <= 0xFF:
            node_id = msg.arbitration_id
        else:
            node_id = (msg.arbitration_id >> 8) & 0xFF

        message = cls(node_id=node_id)
        message._parse_can_msg_data(msg)
        return message

    @property
    def high_id(self) -> int:
        return (self.node_id >> 8) & 0xFF

    @property
    def low_id(self) -> int:
        return self.node_id & 0xFF

    @property
    def arbitration_id(self) -> messages.X424ArbitrationID:
        return cast(messages.X424ArbitrationID, self._arbitration_id)

    def _gen_arbitration_id(self) -> messages.X424ArbitrationID:
        # For standard control messages, use the node ID directly
        return messages.X424ArbitrationID(node_id=self.node_id, cmd_id=self.cmd_id)


class X424CanMessageSetAndQuery(X424CanMessage):
    """Base class for X4-24 set and query messages."""

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        # For X4-24 special commands with ID 0x7FF
        if msg.arbitration_id == 0x7FF:
            return msg.data[3] == cls.cmd_id

        return False

    def _gen_arbitration_id(self) -> messages.X424ArbitrationID:
        # Special case - always use 0x7FF
        return messages.X424ArbitrationID(node_id=0x7FF, cmd_id=self.cmd_id)


class QueryCommunicationModeMessage(X424CanMessageSetAndQuery):
    """Query the communication mode of an X4-24 motor (Q&A or automatic message mode)."""

    cmd_id = 0x81

    mode: Literal["auto", "qa"]

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        return msg.arbitration_id == 0x7FF and msg.data[2] == 0x01

    def _gen_can_msg_data(self) -> bytes:
        # Motor ID high 8 bits, low 8 bits, 0x00, 0x81
        return bytes([self.high_id, self.low_id, 0x00, self.cmd_id])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        if msg.data[3] != 0x80:
            self.mode = "auto" if msg.data[3] == 0x01 else "qa"


class QueryCANCommunicationIDMessage(X424CanMessageSetAndQuery):
    """Query the CAN ID of X4-24 motors on the bus."""

    cmd_id = 0x82

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        return msg.arbitration_id == 0x7FF and msg.data[2] == 0x01

    def _gen_can_msg_data(self) -> bytes:
        return bytes([0xFF, 0xFF, 0x00, self.cmd_id])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        if msg.data[0] == 0xFF and msg.data[2] == 0x01:
            data = msg.data[3:5]
            self._node_id = int.from_bytes(data, "big")


class SetCommunicationModeMessage(X424CanMessageSetAndQuery):
    """Set the communication mode of an X4-24 motor (Q&A or automatic message mode)."""

    cmd_id = 0x00

    mode: Literal["auto", "qa"]

    def _gen_can_msg_data(self) -> bytes:
        # Motor ID high 8 bits, low 8 bits, 0x00, 0x01, mode
        # 0x01 for auto, 0x02 for Q&A
        mode_val = 0x01 if self.mode == "auto" else 0x02
        return bytes([self.high_id, self.low_id, self.cmd_id, mode_val])

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        self._node_id = msg.arbitration_id
        self.mode = "auto" if msg.data[3] == 0x01 else "qa"


class SetZeroPositionMessage(X424CanMessageSetAndQuery):
    """Set the current position as zero position for an X4-24 motor."""

    cmd_id = 0x03

    def _gen_can_msg_data(self) -> bytes:
        # Motor ID high 8 bits, low 8 bits, 0x00, 0x03
        return bytes([self.high_id, self.low_id, 0x00, self.cmd_id])


class SetMotorIDMessage(X424CanMessageSetAndQuery):
    """Set a new ID for an X4-24 motor."""

    cmd_id = 0x04

    cur_node_id: int
    new_node_id: int

    def _gen_can_msg_data(self) -> bytes:
        # Old ID high/low, 0x00, 0x04, New ID high/low
        cur_high, cur_low = (self.cur_node_id >> 8) & 0xFF, self.cur_node_id & 0xFF
        new_high, new_low = (self.new_node_id >> 8) & 0xFF, self.new_node_id & 0xFF
        return bytes([cur_high, cur_low, 0x00, self.cmd_id, new_high, new_low])


class ResetMotorIDMessage(X424CanMessageSetAndQuery):
    """Reset the motor ID to 0x01 (special command)."""

    cmd_id = 0x05

    def _gen_can_msg_data(self) -> bytes:
        # Special format: 0x7F, 0x7F, 0x00, 0x05, 0x7F, 0x7F
        return bytes([0x7F, 0x7F, 0x00, self.cmd_id, 0x7F, 0x7F])


class X424ServoPositionControlMessage(X424CanMessage):
    """Servo position control mode for X4-24 motors (mode 0x01).

    Format:
    - First byte: Control byte containing motor mode, direction, and message type
    - Bytes 1-4: Position value (IEEE 754 float32)
    - Bytes 5-6: Speed value (0x0032 = 50 = 20 RPM)
    - Byte 7: Current threshold (0xCA = 202 ≈ 5A)

    Example for 90° position:
    [0x28, 0x56, 0x80, 0x00, 0x00, 0x00, 0x32, 0xCA]
    """

    cmd_id = 0x01

    # in degrees
    position: float
    # in rpm, range 0-3276.7
    speed: float
    # in A, range 0-10
    current_limit: float
    # 0-3, controls what response is sent
    # Message return status:
    # 0: No return
    # 1: Return message type 1 (lower fidelity position and speed w current)
    # 2: Return message type 2 (higher fidelity position w current)
    # 3: Return message type 3 (higher fidelity speed w current)
    message_type: int = 2

    def _gen_can_msg_data(self) -> bytes:
        """Generate a properly formatted servo position control message using bit-level packing.

        Protocol specification:
        1. Motor mode (uint3): 3 bits - position mode (0x01)
        2. Expected position (float32): 32 bits
        3. Expected speed (uint15): 15 bits
        4. Current threshold (uint12): 12 bits
        5. Message return status (uint2): 2 bits

        Example for 90° position (from x4-24-example.md):
        [0x28, 0x56, 0x80, 0x00, 0x00, 0x32, 0x00, 0xCA]
        """
        # 1. Prepare the values
        speed_value = int(min(self.speed, 32767))
        current_value = int(min(self.current_limit * 10, 4095))

        # 2. Pack position as float32
        position_bytes = struct.pack("<f", self.position)
        position_int = int.from_bytes(position_bytes, byteorder="little")

        # 3. Create a 64-bit integer to hold all fields
        result = 0

        # 4. Insert fields in order from MSB to LSB
        result |= (self.cmd_id & 0x07) << 61  # Motor mode 0x01: bits 61-63
        result |= (position_int & 0xFFFFFFFF) << 29  # position: bits 29-60
        result |= (speed_value & 0x7FFF) << 14  # speed uint15 (0-32767): bits 14-28
        result |= (current_value & 0xFFF) << 2  # current uint12 (0-4095): bits 2-13
        result |= self.message_type & 0x03  # message_type uint2 (0-3): bits 0-1

        # 5. Convert to bytes (big-endian to maintain correct bit order)
        return result.to_bytes(8, byteorder="big")


class X424ServoSpeedControlMessage(X424CanMessage):
    """Servo speed control mode for X4-24 motors (mode 0x02).

    Format:
    - First byte: Control byte containing motor mode and message type
    - Bytes 1-4: Speed value (IEEE 754 float32)
    - Bytes 5-6: Current threshold (0x0032 = 50 = 5A)

    Example for 20 RPM:
    [0x43, 0x41, 0xA0, 0x00, 0x00, 0x00, 0x32]
    """

    cmd_id = 0x02

    # in rpm
    speed: float
    # in A
    current_limit: float
    # 0-3, controls what response is sent
    message_type: int = 3

    def _gen_can_msg_data(self) -> bytes:
        """Generate a properly formatted servo speed control message using bit-level packing.

        Protocol specification:
        1. Motor mode uint3: this is 0x02 for speed mode (3 bits)
        2. Reserved control status uint3: Always 0 (3 bits)
        3. Message return status uint2: Set by message_type (2 bits)
        4. Expected output end speed float32 (32 bits)
        5. Current threshold uint16: 0-65536 with ratio 10 (16 bits)

        Example for 20 RPM:
        [0x43, 0x41, 0xA0, 0x00, 0x00, 0x00, 0x32]
        """
        # 1. Prepare the values
        # motor_mode is cmd_id (0x02)
        reserved_status = 0  # Always 0
        message_type = self.message_type & 0x03  # uint2 (0-3)
        speed_value = self.speed  # float32
        current_value = int(min(self.current_limit * 10, 65535))  # uint16 (0-65535)

        # 2. Pack speed as float32
        speed_bytes = struct.pack("<f", speed_value)
        speed_int = int.from_bytes(speed_bytes, byteorder="little")

        # 3. Create an integer to hold all fields (56 bits total)
        result = 0

        # 4. Insert fields in order from MSB to LSB
        result |= (self.cmd_id & 0x07) << 53  # Motor mode: bits 53-55
        result |= (reserved_status & 0x07) << 50  # Reserved status: bits 50-52
        result |= (message_type & 0x03) << 48  # Message type: bits 48-49
        result |= (speed_int & 0xFFFFFFFF) << 16  # Speed value: bits 16-47
        result |= current_value & 0xFFFF  # Current threshold: bits 0-15

        # 5. Convert to bytes (big-endian to maintain correct bit order)
        # Speed control uses 7 bytes (56 bits)
        return result.to_bytes(7, byteorder="big")


class X424CurrentControlMessage(X424CanMessage):
    """Current/torque control mode for X4-24 motors (mode 0x03).

    Format:
    - First byte: Control byte containing motor mode
    - Bytes 1-2: Current value

    Example for 5A:
    [0x62, 0x01, 0xF4]
    """

    cmd_id = 0x03
    # in A, range -327.68 to 327.67
    current: float
    # 0-7, different control modes
    # 0: Normal current control
    # 1: Torque control mode (Torque = current * torque constant, the torque constant of this product is 1.4Nm/A)
    # 2: Variable damping braking control mode, currently automatically
    # ignoring the actual current data
    # 3: Energy consumption braking control mode, currently automatically
    # ignoring the expected current data
    # 4: Regenerative braking control mode, currently the expected current
    # value is the set braking current threshold
    # 5: Reserved (currently invalid)
    # 6: Reserved (currently invalid)
    # 7: Reserved (currently invalid)
    control_type: int = 0
    # 0-3, controls what response is sent
    message_type: int = 1

    def _gen_can_msg_data(self) -> bytes:
        """Generate a properly formatted current control message using bit-level packing.

        Protocol specification:
        1. Motor mode uint3: 0x03 for current control (3 bits)
        2. Reserved control status uint3: values 0-7 based on control_type (3 bits)
        3. Message return status uint2: 0~3 from message_type (2 bits)
        4. Expected current/expected torque int16: -32768~32767 (16 bits)
           Current in 0.01A units, torque = current * torque constant (1.4Nm/A)

        Total: 24 bits = 3 bytes

        Example for 5A:
        [0x62, 0x01, 0xF4]
        """
        # 1. Prepare the values
        motor_mode = self.cmd_id  # 0x03 for current mode
        control_status = self.control_type & 0x07  # uint3 (0-7)
        message_type = self.message_type & 0x03  # uint2 (0-3)

        # Scale current value (0.01A per unit)
        # 5A = 0x01F4 = 500 integer value
        current_int = int(self.current * 100)

        # Handle signed value in 16-bit range
        if current_int < 0:
            # Convert to two's complement
            current_int = (abs(current_int) ^ 0xFFFF) + 1

        # Ensure value fits in int16 range
        current_int &= 0xFFFF

        # 2. Create a 24-bit integer to hold all fields
        result = 0

        # 3. Insert fields in order from MSB to LSB
        result |= (motor_mode & 0x07) << 21  # Motor mode: bits 21-23
        result |= (control_status & 0x07) << 18  # Control status: bits 18-20
        result |= (message_type & 0x03) << 16  # Message type: bits 16-17
        result |= current_int & 0xFFFF  # Current value: bits 0-15

        # 4. Convert to bytes (big-endian to maintain correct bit order)
        return result.to_bytes(3, byteorder="big")


################################################################################
## X4-24 Cyclic Messages
################################################################################


class QAReturnMessage(X424CanMessage):
    """Q&A return message for X4-24 motors.

    Multi-byte fields on the wire are big-endian (IEEE 754 floats included),
    per RMD-X4-P36-24 / Appendix 1 protocol notes.
    """

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        """Match on message class in bits 5-7 of data[0]."""
        if not msg.data:
            return False
        message_type = (msg.data[0] >> 5) & 0x07
        return message_type == cls.cmd_id


class QAReturnMessageType1(QAReturnMessage):
    """Q&A return message type 1 for X4-24 motors.

    Packed bitfields (big-endian bit layout across the 8-byte frame):
    - Byte0[5:7]: Message type (0x01)
    - Byte0[0:4]: Motor error message
    - Position (uint16): 0~65535 → -12.5..+12.5 rad
    - Speed (uint12): 0-4095 → -18.0..+18.0 rad/s
    - Current (uint12): 0-4095 → -30..+30 A
    - Motor / MOS temperature (uint8): °C = (raw - 50) / 2
    """

    cmd_id = 0x01

    # Error code, from 0-7
    motor_error: int
    # position in radians
    position: float
    # speed in rad/s
    speed: float
    # current in amperes
    current: float
    # temperature in °C
    motor_temp: float
    # temperature in °C
    mos_temp: float

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """Parse the CAN message data into fields."""
        # Extract message type and error from first byte
        self.motor_error = msg.data[0] & 0x1F  # Lower 5 bits

        # Convert the message data bytes to a 64-bit integer for extraction
        data_int = int.from_bytes(msg.data, byteorder="big")

        # Extract position (uint16): bits 40-55
        position_raw = (data_int >> 40) & 0xFFFF
        # Map 0~65536 to -12.5~12.5 rad
        self.position = (position_raw / 65536.0 * 25.0) - 12.5

        # Extract speed (uint12): bits 28-39
        speed_raw = (data_int >> 28) & 0xFFF
        # Map 0-4095 to -18.0~18.0 rad/s
        self.speed = (speed_raw / 4095.0 * 36.0) - 18.0

        # Extract current (uint12): bits 16-27
        current_raw = (data_int >> 16) & 0xFFF
        # Map 0-4095 to -30~30A
        self.current = (current_raw / 4095.0 * 60.0) - 30.0

        # Extract motor temperature (uint8): bits 8-15
        temp_raw = (data_int >> 8) & 0xFF
        self.motor_temp = (temp_raw - 50) / 2.0

        # Extract MOS temperature (uint8): bits 0-7
        mos_temp_raw = data_int & 0xFF
        self.mos_temp = (mos_temp_raw - 50) / 2.0

    def __repr__(self) -> str:
        return (
            f"QAReturnMessageType1(node_id={self.node_id}, error={self.motor_error}, "
            f"position={self.position:.2f} rad, speed={self.speed:.2f} rad/s, "
            f"current={self.current:.2f}A, motor_temp={self.motor_temp:.1f}°C, "
            f"mos_temp={self.mos_temp:.1f}°C)"
        )


class QAReturnMessageType2(QAReturnMessage):
    """Q&A return message type 2 for X4-24 motors.

    Format (all multi-byte fields big-endian):
    - Byte0[5:7]: Message type (0x02)
    - Byte0[0:4]: Motor error message
    - Bytes 1-4: float32 position in **degrees** (IEEE 754 BE)
    - Bytes 5-6: int16 current (0.01 A/LSB)
    - Byte 7: motor temperature, °C = (raw - 50) / 2
    """

    cmd_id = 0x02

    # Fields
    motor_error: int = 0  # 0-7
    position: float = 0.0  # position in degrees
    current: float = 0.0  # current in amperes
    motor_temp: float = 0.0  # temperature in °C

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """Parse the CAN message data into fields."""
        self.motor_error = msg.data[0] & 0x1F  # Lower 5 bits

        # Position: IEEE 754 float32, big-endian (manual Appendix 1)
        self.position = struct.unpack(">f", bytes(msg.data[1:5]))[0]

        # Current: big-endian int16, 0.01 A/LSB
        current_raw = (msg.data[5] << 8) | msg.data[6]
        if current_raw > 32767:
            current_raw -= 65536
        self.current = current_raw / 100.0

        self.motor_temp = (msg.data[7] - 50) / 2.0

    def __repr__(self) -> str:
        return (
            f"QAReturnMessageType2(node_id={self.node_id}, error={self.motor_error}, "
            f"position={self.position:.2f} deg, current={self.current:.2f}A, "
            f"motor_temp={self.motor_temp:.1f}°C)"
        )


class QAReturnMessageType3(QAReturnMessage):
    """Q&A return message type 3 for X4-24 motors.

    Format (all multi-byte fields big-endian):
    - Byte0[5:7]: Message type (0x03)
    - Byte0[0:4]: Motor error message
    - Bytes 1-4: float32 speed in RPM (IEEE 754 BE)
    - Bytes 5-6: int16 current (0.01 A/LSB)
    - Byte 7: motor temperature, °C = (raw - 50) / 2
    """

    cmd_id = 0x03
    # Fields
    motor_error: int = 0  # 0-7
    speed: float = 0.0  # speed in RPM
    current: float = 0.0  # current in amperes
    motor_temp: float = 0.0  # temperature in °C

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """Parse the CAN message data into fields."""
        self.motor_error = msg.data[0] & 0x1F  # Lower 5 bits

        # Speed: IEEE 754 float32, big-endian (verified on-bus at ±50 rpm)
        self.speed = struct.unpack(">f", bytes(msg.data[1:5]))[0]

        # Current: big-endian int16, 0.01 A/LSB
        current_raw = (msg.data[5] << 8) | msg.data[6]
        if current_raw > 32767:
            current_raw -= 65536
        self.current = current_raw / 100.0

        self.motor_temp = (msg.data[7] - 50) / 2.0

    def __repr__(self) -> str:
        return (
            f"QAReturnMessageType3(node_id={self.node_id}, error={self.motor_error}, "
            f"speed={self.speed:.2f} RPM, current={self.current:.2f}A, "
            f"motor_temp={self.motor_temp:.1f}°C)"
        )


class QAReturnMessageType4(X424CanMessage):
    """Q&A return message type 4 for X4-24 motors.

    Response to configuration commands.

    Format:
    - Byte0[5:7]: Message type (0x04)
    - Byte0[0:4]: Motor error message
    - Byte1: Configuration code (0-255)
    - Byte2: Configuration status (0 = failure, 1 = success)
    """

    cmd_id = 0x04
    # Fields
    motor_error: int = 0  # 0-7
    config_code: int = 0  # Configuration code
    config_status: bool = False  # False = failure, True = success

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """Parse the CAN message data into fields."""
        # Extract message type and error from first byte
        self.motor_error = msg.data[0] & 0x1F  # Lower 5 bits

        # Extract configuration code (byte 1)
        self.config_code = msg.data[1]

        # Extract configuration status (byte 2)
        self.config_status = msg.data[2] == 1

    def __repr__(self) -> str:
        status_str = "SUCCESS" if self.config_status else "FAILURE"
        return (
            f"QAReturnMessageType4(node_id={self.node_id}, error={self.motor_error}, "
            f"config_code={self.config_code}, status={status_str})"
        )


class QAReturnMessageType5(X424CanMessage):
    """Q&A return message type 5 for X4-24 motors.

    Response to query commands.

    Format:
    - Byte0[5:7]: Message type (0x05)
    - Byte0[0:4]: Motor error message
    - Byte1: Query code (1-9)
    - Byte2~: Query returned data (variable length, depends on query code)
      1: Position (float32 BE, degrees)
      2: Speed (float32 BE, rpm)
      3: Current (float32 BE)
      4: Power (float32 BE)
      5: Acceleration (uint16 BE, rad/s²)
      6-9: uint16 parameters (2 bytes)
    """

    cmd_id = 0x05

    # Fields
    motor_error: int = 0  # 0-7
    query_code: int = 0  # Query code 1-9

    # Result fields (only one will be populated based on query_code)
    position: float = 0.0  # For query_code 1
    speed: float = 0.0  # For query_code 2
    current: float = 0.0  # For query_code 3
    power: float = 0.0  # For query_code 4
    uint16_value: int = 0  # For query_code 5-9

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        """Check if the CAN message matches QAReturnMessageType5 format."""
        if len(msg.data) < 3:
            return False

        # Check message type (bits 5-7 of first byte)
        message_type = (msg.data[0] >> 5) & 0x07
        return message_type == 0x05

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        """Parse the CAN message data into fields."""
        # Extract message type and error from first byte
        self.motor_error = msg.data[0] & 0x1F  # Lower 5 bits

        # Extract query code (byte 1)
        self.query_code = msg.data[1]

        # Parse the returned data based on query code (big-endian)
        if self.query_code in [1, 2, 3, 4]:
            # Float32 values (position, speed, current, power)
            if len(msg.data) >= 6:  # Need at least 6 bytes
                value = struct.unpack(">f", bytes(msg.data[2:6]))[0]

                if self.query_code == 1:
                    # Query 1: position (degrees per type-2 / guide)
                    self.position = value
                elif self.query_code == 2:
                    self.speed = value
                elif self.query_code == 3:
                    self.current = value
                elif self.query_code == 4:
                    self.power = value

        elif self.query_code in [5, 6, 7, 8, 9]:
            # uint16 values (acceleration rad/s² for code 5, gains, coefficients)
            if len(msg.data) >= 4:  # Need at least 4 bytes
                self.uint16_value = (msg.data[2] << 8) | msg.data[3]

    def __repr__(self) -> str:
        value_str = ""
        if self.query_code == 1:
            value_str = f"position={self.position:.2f} deg"
        elif self.query_code == 2:
            value_str = f"speed={self.speed:.2f} rpm"
        elif self.query_code == 3:
            value_str = f"current={self.current:.2f} A"
        elif self.query_code == 4:
            value_str = f"power={self.power:.2f} W"
        elif self.query_code == 5:
            value_str = f"acceleration={self.uint16_value} rad/s²"
        elif self.query_code in [6, 7, 8, 9]:
            value_str = f"value={self.uint16_value}"

        return (
            f"QAReturnMessageType5(node_id={self.node_id}, error={self.motor_error}, "
            f"query_code={self.query_code}, {value_str})"
        )


class X424AutomaticFeedbackMessage(X424CanMessage):
    """Automatic-mode cyclic feedback (RMD-X4 Appendix 1).

    When communication mode is Automatic (0x01), the motor pushes this frame
    at ~1000 Hz.

    - CAN ID: ``0x204 + motor_id`` (e.g. motor 1 → 0x205)
    - Bytes 0-1: int16 position, degrees = raw / 100.0  (−180..+180)
    - Bytes 2-3: int16 speed, rpm = raw / 10.0
    - Bytes 4-5: int16 torque current, A = raw / 10.0
    - Byte 6: motor temp, °C = (raw - 50) / 2
    - Byte 7: error code
    """

    # Base arbitration ID; actual ID is 0x204 + node_id.
    _AUTO_ID_BASE = 0x204

    position_deg: float = 0.0
    speed_rpm: float = 0.0
    current_a: float = 0.0
    motor_temp: float = 0.0
    error_code: int = 0

    @classmethod
    def matches(cls, msg: can.Message) -> bool:
        if msg.dlc < 8 and len(msg.data) < 8:
            return False
        # Motor IDs 1..32 → auto IDs 0x205..0x224
        return cls._AUTO_ID_BASE + 1 <= msg.arbitration_id <= cls._AUTO_ID_BASE + 32

    @classmethod
    def from_can_message(
        cls: Type[X424CanMessageT], msg: can.Message
    ) -> X424CanMessageT:
        if not cls.matches(msg):
            raise ValueError(
                f"CAN message does not match the class desired {cls.__name__}!"
            )
        node_id = msg.arbitration_id - cls._AUTO_ID_BASE
        message = cls(node_id=node_id)
        message._parse_can_msg_data(msg)
        return message

    def _gen_arbitration_id(self) -> messages.X424ArbitrationID:
        return messages.X424ArbitrationID(
            node_id=self._AUTO_ID_BASE + self.node_id, cmd_id=0
        )

    def _parse_can_msg_data(self, msg: can.Message) -> None:
        pos_raw = struct.unpack(">h", bytes(msg.data[0:2]))[0]
        spd_raw = struct.unpack(">h", bytes(msg.data[2:4]))[0]
        cur_raw = struct.unpack(">h", bytes(msg.data[4:6]))[0]
        self.position_deg = pos_raw / 100.0
        self.speed_rpm = spd_raw / 10.0
        self.current_a = cur_raw / 10.0
        self.motor_temp = (msg.data[6] - 50) / 2.0
        self.error_code = msg.data[7]

    def __repr__(self) -> str:
        return (
            f"X424AutomaticFeedbackMessage(node_id={self.node_id}, "
            f"position={self.position_deg:.2f} deg, speed={self.speed_rpm:.1f} rpm, "
            f"current={self.current_a:.2f}A, temp={self.motor_temp:.1f}°C, "
            f"error={self.error_code})"
        )
