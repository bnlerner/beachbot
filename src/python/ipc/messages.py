from typing import List

import geometry
from config import robot_config
from drivers.camera import primitives as camera_primitives

from ipc import core


class CameraImageMessage(core.BaseMessage):
    """Message containing a camera image."""

    image: camera_primitives.Image

    class Config:
        arbitrary_types_allowed = True


class GNSSMessage(core.BaseMessage):
    """Message containing the current location of the robot, its velocity and covariance."""

    latitude: float
    longitude: float
    ellipsoid_height: float
    heading_of_motion: float
    ned_velocity: geometry.Velocity
    position_covariance: List[List[float]]
    velocity_covariance: List[List[float]]

    def is_in_motion(self) -> bool:
        return self.ned_velocity.magnitude > 0.3


class IMUMessage(core.BaseMessage):
    """A message containing IMU data like RPY along with angular velocity and
    acceleration.
    """

    roll: float
    pitch: float
    true_compass_heading: float
    angular_velocity: geometry.AngularVelocity
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
    """A requires to navigate to a specific target location."""

    target: geometry.Position


class TrackedObjectsMessage(core.BaseMessage):
    """The tracked objects detected in front of or in the rear of the robot."""

    frame: geometry.ReferenceFrame
    objects: List[camera_primitives.TrackedObjects]


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
