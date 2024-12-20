from typing import Optional

import geometry
from ipc import messages, session
from models import twist_estimator
from typing_helpers import req

from localization import gps_transformer, primitives

# UTM zone for Florida is zone number 17N.
_DEFAULT_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)


class Localizer:
    """Localizes the robot in the UTM frame using GNSS and INS data, calculating the
    vehicle kinematics such as position, orientation, velocity and angular velocity.
    """

    def __init__(self, utm_zone: primitives.UTMZone = _DEFAULT_UTM_ZONE):
        self._imu_message: Optional[messages.IMUMessage] = None
        self._gnss_message: Optional[messages.GNSSMessage] = None

        self._gps_transformer = gps_transformer.GPSTransformer(utm_zone)
        config = session.get_robot_config()
        self._twist_estimator = twist_estimator.TwistEstimator(config)

    def input_gnss_msg(self, msg: messages.GNSSMessage) -> None:
        self._gnss_message = msg

    def input_imu_msg(self, msg: messages.IMUMessage) -> None:
        self._imu_message = msg

    def input_motor_vel_msg(self, msg: messages.MotorVelocityMessage) -> None:
        self._twist_estimator.update(msg.motor, msg.estimated_velocity)

    def reset(self) -> None:
        self._imu_message = None
        self._gnss_message = None

    def vehicle_kin_msg(self) -> Optional[messages.VehicleKinematicsMessage]:
        if self._gnss_message is not None and self._imu_message is not None:
            pose = self._calc_pose()
            twist = self._calc_twist(pose.orientation)
            return messages.VehicleKinematicsMessage(pose=pose, twist=twist)
        else:
            return None

    def _calc_pose(self) -> geometry.Pose:
        gnss_msg = req(self._gnss_message)
        imu_msg = req(self._imu_message)
        position = self._gps_transformer.transform_position(
            gnss_msg.longitude,
            gnss_msg.latitude,
            ellipsoid_height=gnss_msg.ellipsoid_height,
        )
        # Don't provide heading of motion if GNSS is not moving as its not a valid measure.
        heading_of_motion = (
            gnss_msg.heading_of_motion if gnss_msg.is_in_motion() else None
        )
        heading = self._calc_heading(imu_msg.true_compass_heading, heading_of_motion)
        yaw = self._gps_transformer.transform_heading(
            gnss_msg.longitude, gnss_msg.latitude, heading
        )
        orientation = self._calc_orientation(imu_msg.roll, imu_msg.pitch, yaw)
        return geometry.Pose(position, orientation)

    def _calc_heading(
        self, true_compass_heading: float, heading_of_motion: Optional[float]
    ) -> float:
        """Determines the heading of the robot using either the heading of motion or the
        true compass heading depending on whether the robot is in motion or stationary.
        """
        if heading_of_motion:
            wheel_est_twist = self._twist_estimator.twist()
            if geometry.sign(wheel_est_twist.velocity.x) == -1:
                # Traveling in reverse
                return (heading_of_motion + 180) % 360
            else:
                return heading_of_motion
        else:
            return true_compass_heading

    def _calc_orientation(
        self, roll: float, pitch: float, yaw: float
    ) -> geometry.Orientation:
        body_ori = geometry.Orientation.from_intrinsic_rpy(
            geometry.UTM, roll, pitch, 0.0
        )
        roll, pitch = body_ori.roll, body_ori.pitch
        return geometry.Orientation(geometry.UTM, roll, pitch, yaw)

    def _calc_twist(self, body_ori: geometry.Orientation) -> geometry.Twist:
        """Calculates the current vehicle twist based on wheel velocity and data coming
        from the ublox module.
        """
        velocity = self._calc_velocity(req(self._gnss_message).ned_velocity, body_ori)
        angular_velocity = req(self._imu_message).angular_velocity

        return geometry.Twist(velocity, angular_velocity)

    def _calc_velocity(
        self, utm_velocity: geometry.Velocity, body_ori: geometry.Orientation
    ) -> geometry.Velocity:
        """Calculates the robots's velocity in BODY frame."""
        body_velocity = utm_velocity.rotated(
            body_ori.as_rotation().inverted(), intrinsic=False
        )
        body_velocity.frame = geometry.BODY

        return body_velocity
