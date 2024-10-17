"""Common enums for the CAN bus protocols"""
import enum


class BusType(enum.Enum):
    SOCKET_CAN = "socketcan"
    VIRTUAL = "virtual"


class CANInterface(enum.Enum):
    """Specifies the CAN interfaces."""

    ODRIVE = "can0"
    VIRTUAL = "vcan"
