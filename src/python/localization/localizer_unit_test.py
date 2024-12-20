import math

import geometry
import pytest
from config import robot_config
from ipc import messages, session

from localization import localizer as localizer_module

_ANGULAR_VELOCITY = geometry.AngularVelocity(geometry.BODY, 0, 0, 10)
# Velocity is due east
_NED_VELOCITY = geometry.Velocity(geometry.UTM, 1.0, 0.0, 0.0)


@pytest.fixture
def localizer() -> localizer_module.Localizer:
    return localizer_module.Localizer()


def _gen_gnss_msg() -> messages.GNSSMessage:
    return messages.GNSSMessage(
        latitude=26.3353346,
        longitude=-80.128041,
        ellipsoid_height=-19.261,
        heading_of_motion=90.3868463628129317,
        ned_velocity=_NED_VELOCITY,
        position_covariance=[[0] * 3] * 3,
        velocity_covariance=[[0] * 3] * 3,
    )


def _gen_imu_msg() -> messages.IMUMessage:
    return messages.IMUMessage(
        roll=0,
        pitch=0,
        true_compass_heading=0,
        angular_velocity=_ANGULAR_VELOCITY,
        is_calibrated=True,
    )


def _gen_motor_vel_msg() -> messages.MotorVelocityMessage:
    motor = session.get_motor("beachbot-1", robot_config.DrivetrainLocation.FRONT_LEFT)
    return messages.MotorVelocityMessage(motor=motor, estimated_velocity=-2)


def test_localizer(localizer: localizer_module.Localizer) -> None:
    gnss_msg = _gen_gnss_msg()
    imu_msg = _gen_imu_msg()
    motor_vel_msg = _gen_motor_vel_msg()

    localizer.input_gnss_msg(gnss_msg)
    localizer.input_imu_msg(imu_msg)
    localizer.input_motor_vel_msg(motor_vel_msg)

    veh_kin_msg = localizer.vehicle_kin_msg()
    assert veh_kin_msg is not None

    assert math.isclose(
        veh_kin_msg.twist.velocity.magnitude, gnss_msg.ned_velocity.magnitude
    )
    ned_utm_vel = gnss_msg.ned_velocity
    ned_utm_vel.frame = geometry.BODY
    assert veh_kin_msg.twist.velocity == ned_utm_vel
    assert veh_kin_msg.twist.spin == imu_msg.angular_velocity
