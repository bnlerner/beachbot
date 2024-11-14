from config import robot_config
from planning import primitives

from ipc import core


class GPSMessage(core.BaseMessage):
    """The GPS data from the body mounted GPS receiver."""

    latitude: float
    longitude: float
    ellipsoid_height: float


class VehicleDynamicsMessage(core.BaseMessage):
    """The vehicle dynamics data that comes from the vehicle mounted IMU.
    Data is expressed in BODY frame. See frames for more details.
    """

    # RPH is in degrees.
    roll: float
    pitch: float
    heading: float
    # angular speed rates in deg/s
    roll_rate: float
    pitch_rate: float
    yaw_rate: float


class MotorCommandMessage(core.BaseMessage):
    """The target motor commands to use for control of each motor."""

    motor: robot_config.Motor
    velocity: float


class StopMotorsMessage(core.BaseMessage):
    """Stops all motors via the software E-stop."""

    pass


class MotorVelocityMessage(core.BaseMessage):
    motor: robot_config.Motor
    estimated_velocity: float


class NavigateRequest(core.Request):
    """A location to navigate to."""

    target: primitives.GPSPoint
