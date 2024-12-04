import geometry
from config import robot_config

from ipc import core


class GNSSMessage(core.BaseMessage):
    latitude: float
    longitude: float
    ellipsoid_height: float
    ned_velocity: geometry.Velocity


class ESFMessage(core.BaseMessage):
    roll: float
    pitch: float
    heading: float
    angular_velocity: geometry.AngularVelocity
    angular_acceleration: geometry.AngularAcceleration


class MotorCommandMessage(core.BaseMessage):
    """Commands a velocity setpoint for the motor, along with a feedforward torque in
    Nm.
    """

    motor: robot_config.Motor
    velocity: float
    feedforward_torque: float = 0.0


class MotorVelocityMessage(core.BaseMessage):
    motor: robot_config.Motor
    estimated_velocity: float


class NavigateRequest(core.Request):
    """A location to navigate to."""

    target: geometry.Position


class StopMotorsMessage(core.BaseMessage):
    """Stops all motors via the software E-stop."""

    pass


class VehicleKinematicsMessage(core.BaseMessage):
    """The vehicle kinematics data expresses the vehicles state including position,
    orientation, velocity and angular velocity. Data comes from the vehicle mounted IMU
    and GPS. Data is expressed in BODY frame. See frames for more details.
    """

    pose: geometry.Pose
    twist: geometry.Twist
