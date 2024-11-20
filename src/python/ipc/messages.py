import geometry
from config import robot_config

from ipc import core


class VehicleKinematicsMessage(core.BaseMessage):
    """The vehicle kinematics data expresses the vehicles state including position,
    orientation, velocity and angular velocity. Data comes from the vehicle mounted IMU
    and GPS. Data is expressed in BODY frame. See frames for more details.
    """

    pose: geometry.Pose
    twist: geometry.Twist


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

    target: geometry.Position
