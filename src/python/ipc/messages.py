from drivers import primitives as hw_primitives
from planning import primitives as pl_primitives

from ipc import core


class GPSMessage(core.BaseMessage):
    """The GPS data from the body mounted GPS receiver."""

    latitude: float
    longitude: float


class VehicleDynamicsMessage(core.BaseMessage):
    """The vehicle dynamics data that comes from the body mounted IMU."""

    roll: float
    pitch: float
    # TODO: Verify if this heading is in degrees or radians.
    heading: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    roll_acceleration: float
    pitch_acceleration: float
    heading_acceleration: float


class MotorCommandMessage(core.BaseMessage):
    """The target motor commands to use for control of each motor."""

    motor: hw_primitives.Motor
    velocity: float


class NavigateRequest(core.Request):
    """A location to navigate to."""

    target: pl_primitives.NavigationPoint
