import collections
from typing import DefaultDict, Optional

import geometry
from config import robot_config
from ipc import messages

from localization import gps_transformer, primitives

# UTM zone for Florida is zone number 17N.
_DEFAULT_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)


class Localizer:
    """Calculates the kinematics of the robot."""

    def __init__(self, utm_zone: primitives.UTMZone = _DEFAULT_UTM_ZONE):
        self._esf_message: Optional[messages.ESFMessage] = None
        self._gnss_message: Optional[messages.GNSSMessage] = None
        self._motor_velocities: DefaultDict[
            robot_config.DrivetrainLocation, float
        ] = collections.defaultdict(lambda: 0.0)

        self._gps_transformer = gps_transformer.GPSTransformer(utm_zone)

    def input_gnss_msg(self, msg: messages.GNSSMessage) -> None:
        self._gnss_message = msg

    def input_esf_msg(self, msg: messages.ESFMessage) -> None:
        self._esf_message = msg

    def input_motor_vel_msg(self, msg: messages.MotorVelocityMessage) -> None:
        self._motor_velocities[msg.motor.location] = msg.estimated_velocity

    def reset(self) -> None:
        self._esf_message = None
        self._gnss_message = None

    def vehicle_kin_msg(self) -> Optional[messages.VehicleKinematicsMessage]:
        if self._gnss_message and self._esf_message:
            position = self._gps_transformer.transform_position(
                self._gnss_message.longitude,
                self._gnss_message.latitude,
                ellipsoid_height=self._gnss_message.ellipsoid_height,
            )
            yaw = self._gps_transformer.transform_heading(
                self._gnss_message.longitude,
                self._gnss_message.latitude,
                self._esf_message.heading,
            )
            orientation = self._calc_orientation(
                self._esf_message.roll, self._esf_message.pitch, yaw
            )
            pose = geometry.Pose(position, orientation)
            velocity = self._calc_velocity(self._gnss_message.ned_velocity, orientation)
            angular_velocity = self._esf_message.angular_velocity
            return messages.VehicleKinematicsMessage(
                pose=pose, twist=geometry.Twist(velocity, angular_velocity)
            )

        return None

    def _calc_orientation(
        self, roll: float, pitch: float, yaw: float
    ) -> geometry.Orientation:
        veh_ori = geometry.Orientation.from_intrinsic_rpy(
            geometry.UTM, roll, pitch, 0.0
        )
        roll, pitch = veh_ori.roll, veh_ori.pitch
        return geometry.Orientation(geometry.UTM, roll, pitch, yaw)

    # TODO: Integrate for sensor fusion.
    def _calc_velocity(
        self, utm_velocity: geometry.Velocity, veh_ori: geometry.Orientation
    ) -> geometry.Velocity:
        """Calculates the vehicle velocity."""
        veh_velocity = utm_velocity.rotated(veh_ori.as_rotation().inverted())
        veh_velocity.frame = geometry.VEHICLE

        return veh_velocity

    def _motor_wheel_twist(self) -> geometry.Twist:
        """The twist in the robots body frame expressed as two floats of a linear and
        angular velocity.
        """
        # TODO: Break this out into a helper file with unit tests.
        front_left_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.FRONT_LEFT
        ]
        front_right_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.FRONT_RIGHT
        ]
        rear_left_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.REAR_LEFT
        ]
        rear_right_vel = self._motor_velocities[
            robot_config.DrivetrainLocation.REAR_RIGHT
        ]

        left_vel = (front_left_vel + rear_left_vel) / 2
        right_vel = (front_right_vel + rear_right_vel) / 2

        speed = (right_vel - left_vel) / 2
        linear_velocity = geometry.Velocity(geometry.BODY, speed, 0, 0)
        angular_speed = (right_vel + left_vel) / 2
        angular_velocity = geometry.AngularVelocity(geometry.BODY, 0, 0, angular_speed)

        return geometry.Twist(linear_velocity, angular_velocity)
