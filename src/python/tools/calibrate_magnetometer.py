"""A script to calibrate the magnetometer to ensure that hard and soft iron distortions
are taken into account. Hard Iron Offset Correction is an object that distorts the
magnetic field around the magnetometer. Script finds the center of the distorted sphere
by computing the mean of each axis then subtract these offsets from the raw data. A soft
iron scale correction is a distortion based on where the magnetometer is pointed. Script
computes the scaling factor for each axis.
"""
import math
import os
import sys
import time
from typing import Tuple

import adafruit_bno08x  # type:ignore[import-untyped]
import board  # type:ignore[import-untyped]
import busio  # type:ignore[import-untyped]
import numpy as np
from adafruit_bno08x.i2c import BNO08X_I2C  # type:ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry

_MAG_X_OFFSET = -13.875
_MAG_Y_OFFSET = 12.59375
_MAG_Z_OFFSET = -19.15625
_MAG_X_SCALAR = 26.25
_MAG_Y_SCALAR = 25.71875
_MAG_Z_SCALAR = 6.28125
_DECLINATION_OFFSET = -7.36


def tilt_compensated_heading(
    pitch: float, roll: float, mag_x: float, mag_y: float, mag_z: float
) -> float:
    # Tilt compensation
    pitch = math.radians(pitch)
    roll = math.radians(roll)
    mag_x_comp = mag_x * math.cos(pitch) + mag_z * math.sin(pitch)
    mag_y_comp = (
        mag_x * math.sin(roll) * math.sin(pitch)
        + mag_y * math.cos(roll)
        - mag_z * math.sin(roll) * math.cos(pitch)
    )

    # Compute heading
    heading = math.atan2(mag_y_comp, mag_x_comp)
    heading_degrees = math.degrees(heading) + _DECLINATION_OFFSET

    # Ensure heading is in [0, 360] degrees
    if heading_degrees < 0:
        heading_degrees += 360
    return heading_degrees


def _calc_true_compass_heading(
    body_ori: geometry.Orientation, mag_north_vec: geometry.Direction
) -> float:
    """Calculates compass heading of true north in the vehicle's BODY frame using
    the magnetometer.
    """
    mag_x_comp = mag_north_vec.x * body_ori.cos(
        "pitch"
    ) + mag_north_vec.z * body_ori.sin("pitch")
    mag_y_comp = (
        mag_north_vec.x * body_ori.sin("roll") * body_ori.sin("pitch")
        + mag_north_vec.y * body_ori.cos("roll")
        - mag_north_vec.z * body_ori.sin("roll") * body_ori.cos("pitch")
    )

    # Compute heading
    heading = math.degrees(math.atan2(mag_y_comp, mag_x_comp))

    # Azimuth of the vector is 0 deg when east and is a yaw based angle.
    # Add the declination per our location for the heading offset to get true north.
    true_compass_heading = (heading + _DECLINATION_OFFSET + 360) % 360

    return true_compass_heading


def _calibrated_magnetometer_data(
    mag_x: float, mag_y: float, mag_z: float
) -> Tuple[float, float, float]:
    # An offset calibration from hard iron in the data such as permanent magnetic
    # fields near the magnetometer (e.g., speakers, motors, or magnets). Shifts the
    # magnetic readings by adding a constant offset.
    offset_x = mag_x - _MAG_X_OFFSET
    offset_y = mag_y - _MAG_Y_OFFSET
    offset_z = mag_z - _MAG_Z_OFFSET

    # Calibration from soft iron distortion caused by nearby ferromagnetic materials
    # (e.g., steel). Distorts the shape of the magnetic field (scaling and skewing).
    scale_factor = max(_MAG_X_SCALAR, _MAG_Y_SCALAR, _MAG_Z_SCALAR)

    cal_mag_x = offset_x * scale_factor / _MAG_X_SCALAR
    cal_mag_y = offset_y * scale_factor / _MAG_Y_SCALAR
    cal_mag_z = offset_z * scale_factor / _MAG_Z_SCALAR

    return cal_mag_x, cal_mag_y, cal_mag_z


def main() -> None:
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

    data = []
    try:
        while True:
            mag_x, mag_y, mag_z = bno.magnetic
            cal_mag_x, cal_mag_y, cal_mag_z = _calibrated_magnetometer_data(
                mag_x, mag_y, mag_z
            )
            body_mag_vec = geometry.Direction(
                geometry.BODY, cal_mag_x, -cal_mag_y, -cal_mag_z
            )
            data.append([mag_x, mag_y, mag_z])

            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            if sum([quat_i, quat_j, quat_k, quat_real]) == 0:
                time.sleep(1)
                continue

            roll, pitch, yaw = geometry.quaternion_to_euler(
                quat_real, quat_i, quat_j, quat_k
            )
            imu_roll_mount_offset = -0.4
            imu_pitch_mount_offset = 2.65
            veh_ori = geometry.Orientation(
                geometry.UTM,
                roll + imu_roll_mount_offset,
                pitch + imu_pitch_mount_offset,
                yaw,
            )
            body_ori = veh_ori.rotated(geometry.VEH_TO_BODY_ROT)
            print(f"{mag_x=}, {mag_y=}, {mag_z=}")
            print(
                f"mag heading {tilt_compensated_heading(body_ori.pitch, body_ori.roll, body_mag_vec.x, body_mag_vec.y, body_mag_vec.z)}"
            )
            print(f"other: {_calc_true_compass_heading(body_ori, body_mag_vec)}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        mag_data = np.array(data)
        offset_x = (mag_data[:, 0].max() + mag_data[:, 0].min()) / 2
        offset_y = (mag_data[:, 1].max() + mag_data[:, 1].min()) / 2
        offset_z = (mag_data[:, 2].max() + mag_data[:, 2].min()) / 2

        scale_x = (mag_data[:, 0].max() - mag_data[:, 0].min()) / 2
        scale_y = (mag_data[:, 1].max() - mag_data[:, 1].min()) / 2
        scale_z = (mag_data[:, 2].max() - mag_data[:, 2].min()) / 2
        print(
            f"{offset_x=}, {offset_y=}, {offset_z=}, {scale_x=}, {scale_y=}, {scale_z=}"
        )

    finally:
        i2c.unlock()


if __name__ == "__main__":
    main()
