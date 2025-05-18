"""Common enums for the CAN bus protocols"""
import enum


class BusType(enum.Enum):
    SOCKET_CAN = "socketcan"
    VIRTUAL = "virtual"


class CANInterface(enum.Enum):
    """Specifies the CAN interfaces."""

    ODRIVE = "can0"
    MYACTUATOR = "can0"
    VIRTUAL = "vcan"


class X424MotorError(enum.Enum):
    """Specifies the error codes for the X4-24 motor."""

    NO_ERROR = 0
    MOTOR_OVERHEATING = 1
    MOTOR_OVERCURRENT = 2
    MOTOR_VOLTAGE_TOO_LOW = 3
    MOTOR_ENCODER_ERROR = 4
    MOTOR_BRAKE_VOLTAGE_TOO_HIGH = 6
    DRV_DRIVE_ERROR = 7


class MyActuatorV3OperatingMode(enum.Enum):
    """Specifies the operating modes for the MyActuator controller V3."""

    CURRENT_LOOP_CONTROL = 0x01
    SPEED_LOOP_CONTROL = 0x02
    POSITION_LOOP_CONTROL = 0x03


class MyActuatorFunctionControlIndex(enum.Enum):
    """Function indices for the MyActuator V3 controller Function Control Command (0x20).

    These values correspond to section 2.34.4 of the MyActuator Controller V4.2 protocol.
    Each function controls a specific aspect of the motor controller's behavior.
    """

    # Clear motor multi-turn value, update zero point and save. It will take effect after restarting.
    CLEAR_MULTI_TURN_VALUE = 0x01
    # Enable/disable CANID filter for communication efficiency.
    # The value "1" means that the CANID filter is enabled, which can improve the
    # efficiency of motor sending and receiving in CAN communication.
    # The value "0" means the disabled CANID filter, which needs to be disabled when the
    # multi-motor control command 0x280, 0x300 is required.
    # This value will be saved in FLASH, and the written value will be recorded after
    # power off.
    CANID_FILTER_ENABLE = 0x02
    # Enable/disable automatic error status reporting.
    # The value "1" means that this function is enabled. After the motor appears in an
    # error state, it actively sends the status command 0x9A to the bus with a sending
    # cycle of 100ms. Stop sending after the error status disappears.
    # The value "0" means the function is disabled.
    ERROR_STATUS_TRANSMISSION = 0x03
    # Enable/disable saving multi-turn value on power off.
    # The value "1" means that this function is enabled, and the motor will save the
    # current multi-turn value before powering off. The system defaults to single lap
    # mode. It will take effect after restarting.
    MULTI_TURN_SAVE_ON_POWER_OFF = 0x04
    # Set the CANID for the motor.
    # The value means the CANID number that is going to be modified, which will be saved
    # to ROM and take effect after a reboot.
    SET_CANID = 0x05
    # Set the maximum positive angle for position mode.
    # The value represents the maximum positive angle value for the position operation
    # mode, which is set and saved to ROM to take effect immediately.
    SET_MAX_POSITIVE_ANGLE = 0x06
    # Set the maximum negative angle for position mode.
    # The value represents the maximum negative angle value for the position operation
    # mode, which is set and saved to ROM to take effect immediately.
    SET_MAX_NEGATIVE_ANGLE = 0x07
