from config import motor_config

from ipc import core


class GPSMessage(core.BaseMessage):
    """The GPS data from the body mounted GPS receiver."""

    latitude: float
    longitude: float


class VehicleDynamicsMessage(core.BaseMessage):
    """The vehicle dynamics data that comes from the body mounted IMU."""

    roll: float
    pitch: float
    heading: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    roll_acceleration: float
    pitch_acceleration: float
    heading_acceleration: float


class MotorCommandMessage(core.BaseMessage):
    """The target motor commands to use for control of each motor."""

    motor: motor_config.MotorConfig
    velocity: float
