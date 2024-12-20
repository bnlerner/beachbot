import asyncio
import dataclasses
import math
import os
import sys
from typing import Tuple

import adafruit_bno08x  # type: ignore[import-untyped]
import board  # type: ignore[import-untyped]
import busio  # type: ignore[import-untyped]
from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
import log
from drivers.imu import enums
from ipc import messages as ipc_messages
from ipc import registry, session

from node import base_node

_PUB_FREQUENCY = 50
_CALIBRATE_STATUS_SLEEP = 1.0
# The amount of offset between magnetic north and true north when using a magnetometer.
# Found on NOAA's website. Points a bit west in boca raton which is considered negative
# when added to the reading using the magnetometer readings.
# https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml?utm_source=chatgpt.com#declination
_DECLINATION_OFFSET = -7.36


@dataclasses.dataclass
class _IMUDataCache:
    # Gyro rotation measurement on the XYZ axis in rad/s
    angular_speed_x: float
    angular_speed_y: float
    angular_speed_z: float

    # The roll and pitch of the IMU (degrees) in the BODY frame.
    roll: float
    pitch: float
    # Heading as found using the magnetometer, indicates true north.
    true_compass_heading: float

    # Indicates if the IMU is calibrated.
    is_calibrated: bool

    # An indication of the stability of the system, whether moving or stationary.
    stability: enums.StabilityStatus

    def __repr__(self) -> str:
        data = ", ".join(f"{key}={val!r}" for key, val in self.__dict__.items())
        return f"{self.__class__.__name__}({data})"

    def angular_velocity(self) -> geometry.AngularVelocity:
        """Calculates the BODY frame angular velocity. Since the sensor is rolled 180
        relative to the BODY frame, the y and z angular speed's signs are flipped.
        """
        return geometry.AngularVelocity(
            geometry.BODY,
            math.degrees(self.angular_speed_x),
            math.degrees(-self.angular_speed_y),
            math.degrees(-self.angular_speed_z),
        )


class IMUNode(base_node.BaseNode):
    """Gathers the IMU sensor data from the module and publishes it over IPC."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.IMU)
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._bno = BNO08X_I2C(self._i2c)
        self._robot_config = session.get_robot_config()
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_STABILITY_CLASSIFIER)

        self._imu_data_cache: _IMUDataCache

        self.add_publishers(registry.Channels.IMU)
        self.add_tasks(self._calibrate)
        self.add_looped_tasks({self._publish_imu_reading: _PUB_FREQUENCY})

    async def _calibrate(self) -> None:
        """Starts the calibration process, if needed."""
        if not self._is_calibrated():
            self._bno.begin_calibration()
            while not self._is_calibrated():
                await asyncio.sleep(_CALIBRATE_STATUS_SLEEP)

            self._bno.save_calibration_data()
            log.info("IMU is now calibrated!")

    def _is_calibrated(self) -> bool:
        return enums.CalibrationStatus(self._bno.calibration_status).is_calibrated

    def _publish_imu_reading(self) -> None:
        self._update_data_cache()
        msg = ipc_messages.IMUMessage(
            roll=self._imu_data_cache.roll,
            pitch=self._imu_data_cache.pitch,
            true_compass_heading=self._imu_data_cache.true_compass_heading,
            angular_velocity=self._imu_data_cache.angular_velocity(),
            is_calibrated=self._imu_data_cache.is_calibrated,
        )
        self.publish(registry.Channels.IMU, msg)

    def _update_data_cache(self) -> None:
        gyro_x, gyro_y, gyro_z = self._bno.gyro
        body_ori = self._calc_body_ori()
        self._imu_data_cache = _IMUDataCache(
            angular_speed_x=gyro_x,
            angular_speed_y=gyro_y,
            angular_speed_z=gyro_z,
            roll=body_ori.roll,
            pitch=body_ori.pitch,
            true_compass_heading=self._calc_true_compass_heading(body_ori),
            is_calibrated=self._is_calibrated(),
            stability=enums.StabilityStatus.from_stability_classification(
                self._bno.stability_classification
            ),
        )

    def _calc_body_ori(self) -> geometry.Orientation:
        """Converts the sensors quaternion measurement into an euler RPY orientation in
        the BODY frame. IMU rotation vector quaternion is optimized for accuracy and
        referenced to magnetic north and gravity from accelerometer, gyro, and
        magnetometer data. The data is presented as a four point quaternion output for
        accurate data manipulation.

        NOTE: Yaw is not accurate and should not be used.
        """
        quat_i, quat_j, quat_k, quat_real = self._bno.quaternion
        roll, pitch, yaw = geometry.quaternion_to_euler(
            quat_real, quat_i, quat_j, quat_k
        )
        roll_offset = self._robot_config.imu_roll_mount_offset
        pitch_offset = self._robot_config.imu_pitch_mount_offset
        veh_ori = geometry.Orientation(
            geometry.UTM, roll + roll_offset, pitch + pitch_offset, yaw
        )
        body_ori = veh_ori.rotated(geometry.VEH_TO_BODY_ROT)

        return body_ori

    def _calc_true_compass_heading(self, body_ori: geometry.Orientation) -> float:
        """Calculates compass heading of true north in the vehicle's BODY frame using
        the magnetometer.
        """
        mag_x, mag_y, mag_z = self._calibrated_magnetometer_data()
        mag_north_vec = geometry.Direction(geometry.BODY, mag_x, -mag_y, -mag_z)

        mag_x_comp = mag_north_vec.x * body_ori.cos(
            "pitch"
        ) + mag_north_vec.z * body_ori.sin("pitch")
        mag_y_comp = (
            mag_north_vec.x * body_ori.sin("roll") * body_ori.sin("pitch")
            + mag_north_vec.y * body_ori.cos("roll")
            - mag_north_vec.z * body_ori.sin("roll") * body_ori.cos("pitch")
        )

        heading = math.degrees(math.atan2(mag_y_comp, mag_x_comp))

        # Add the declination per our location for the heading offset to get true north.
        # Ensure value is positiive per convention on heading between [0, 360).
        true_compass_heading = (heading + _DECLINATION_OFFSET + 360) % 360

        return true_compass_heading

    def _calibrated_magnetometer_data(self) -> Tuple[float, float, float]:
        """Applies offsets and scaling factors to the magnetometer data to ensure an
        accurate and proportional reading given the mounting location of the IMU.
        """
        mag_x, mag_y, mag_z = self._bno.magnetic
        # An offset calibration from hard iron in the data such as permanent magnetic
        # fields near the magnetometer (e.g., speakers, motors, or magnets). Shifts the
        # magnetic readings by adding a constant offset.
        offset_x = mag_x - self._robot_config.mag_x_offset
        offset_y = mag_y - self._robot_config.mag_y_offset
        offset_z = mag_z - self._robot_config.mag_z_offset

        # Calibration from soft iron distortion caused by nearby ferromagnetic materials
        # (e.g., steel). Distorts the shape of the magnetic field (scaling and skewing).
        scale_factor = max(
            self._robot_config.mag_x_scalar,
            self._robot_config.mag_y_scalar,
            self._robot_config.mag_z_scalar,
        )

        cal_mag_x = offset_x * scale_factor / self._robot_config.mag_x_scalar
        cal_mag_y = offset_y * scale_factor / self._robot_config.mag_y_scalar
        cal_mag_z = offset_z * scale_factor / self._robot_config.mag_z_scalar

        return cal_mag_x, cal_mag_y, cal_mag_z

    async def shutdown_hook(self) -> None:
        self._i2c.unlock()


if __name__ == "__main__":
    node = IMUNode()
    node.start()
