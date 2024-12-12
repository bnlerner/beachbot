import geometry
from config import robot_config

from ipc import core


class GNSSMessage(core.BaseMessage):
    """Message containing the current location of the robot."""

    latitude: float
    longitude: float
    ellipsoid_height: float
    heading: float
    ned_velocity: geometry.Velocity


class IMUMessage(core.BaseMessage):
    """A message containing IMU data like RPY along with angular velocity and
    acceleration.
    """

    roll: float
    pitch: float
    heading: float
    angular_velocity: geometry.AngularVelocity
    acceleration: geometry.Acceleration
    is_calibrated: bool


class MotorCommandMessage(core.BaseMessage):
    """Commands a velocity setpoint for the motor Allows setting a feedforward torque
    (Nm) along with a custom integral reset directly in the motor controller.
    """

    motor: robot_config.Motor
    velocity: float
    feedforward_torque: float = 0.0
    # Ensures there is no transient integral torque in the motor
    reset_integral: bool = False


class MotorVelocityMessage(core.BaseMessage):
    """Estimated motor velocity according to the motor controller."""

    motor: robot_config.Motor
    estimated_velocity: float


class NavigateRequest(core.Request):
    """A requres to navigate to a specific target location."""

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
