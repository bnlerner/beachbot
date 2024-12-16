import math
import os
import sys
import time

import adafruit_bno08x  # type:ignore[import-untyped]
import board  # type:ignore[import-untyped]
import busio  # type:ignore[import-untyped]
from adafruit_bno08x.i2c import BNO08X_I2C  # type:ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry


def find_heading(dqw: float, dqx: float, dqy: float, dqz: float) -> float:
    norm = math.sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
    dqw = dqw / norm
    dqx = dqx / norm
    dqy = dqy / norm
    dqz = dqz / norm

    ysqr = dqy * dqy

    t3 = +2.0 * (dqw * dqz + dqx * dqy)
    t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
    yaw_raw = math.atan2(t3, t4)
    yaw = yaw_raw * 180.0 / math.pi
    if yaw > 0:
        yaw = 360 - yaw
    else:
        yaw = abs(yaw)
    return yaw  # heading in 360 clockwise


def main() -> None:
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_STABILITY_CLASSIFIER)

    try:
        while True:
            accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
            print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
            print("")
            print("Magnetometer:")
            mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
            print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
            print("")
            print("Gyro:")
            gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
            print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
            print("")
            quat_i, quat_j, quat_k, quat_real = bno.quaternion
            roll, pitch, yaw = geometry.quaternion_to_euler(
                quat_real, quat_i, quat_j, quat_k
            )
            imu_roll_mount_offset: float = -0.4
            imu_pitch_mount_offset: float = 2.65
            veh_ori = geometry.Orientation(
                geometry.VEHICLE,
                roll + imu_roll_mount_offset,
                pitch + imu_pitch_mount_offset,
                yaw,
            )
            body_ori = veh_ori.rotated(geometry.VEH_TO_BODY_ROT)
            print(f"RPY: {body_ori=}")
            heading = find_heading(quat_real, quat_i, quat_j, quat_k)
            print("Heading using rotation vector:", heading)

            # the geomagnetic sensor is unstable
            # Heading is calculated using geomagnetic vector
            (
                geo_quat_i,
                geo_quat_j,
                geo_quat_k,
                geo_quat_real,
            ) = bno.geomagnetic_quaternion
            heading_geo = find_heading(
                geo_quat_real, geo_quat_i, geo_quat_j, geo_quat_k
            )
            print("Heading using geomagnetic rotation vector:", heading_geo)
            print("")

            stability = bno.stability_classification
            print(f"Stability: {stability=}\n")

            time.sleep(0.5)

    finally:
        i2c.unlock()


if __name__ == "__main__":
    main()
