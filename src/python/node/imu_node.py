import asyncio
import dataclasses
import math
import os
import sys
from typing import Tuple

import adafruit_bno08x  # type:ignore[import-untyped]
import board  # type:ignore[import-untyped]
import busio  # type:ignore[import-untyped]
from adafruit_bno08x.i2c import BNO08X_I2C  # type:ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
from drivers.imu import enums
from ipc import messages as ipc_messages
from ipc import registry

from node import base_node

_PUB_FREQUENCY = 50
_CALIBRATE_STATUS_SLEEP = 1.0


@dataclasses.dataclass
class _IMUDataCache:
    # Linear acceleration in m/s^2
    acceleration_x: float
    acceleration_y: float
    acceleration_z: float

    # Magnetic field in each axis in uT
    magnetic_field_x: float
    magnetic_field_y: float
    magnetic_field_z: float

    # Gyro rotation measurement on the XYZ axis in rad/s
    angular_speed_x: float
    angular_speed_y: float
    angular_speed_z: float

    # The RPY of the IMU (degrees) in its own frame.
    roll: float
    pitch: float
    yaw: float

    # Indicates if the IMU is calibrated.
    is_calibrated: bool

    # An indication of the stability of the system, whether moving or stationary.
    stability: enums.StabilityStatus

    def __repr__(self) -> str:
        data = ", ".join(f"{key}={val!r}" for key, val in self.__dict__.items())
        return f"{self.__class__.__name__}({data})"

    def angular_velocity(self) -> geometry.AngularVelocity:
        return geometry.AngularVelocity(
            geometry.VEHICLE,
            math.degrees(self.angular_speed_x),
            math.degrees(self.angular_speed_y),
            math.degrees(self.angular_speed_z),
        )

    def acceleration(self) -> geometry.Acceleration:
        return geometry.Acceleration(
            geometry.VEHICLE,
            self.acceleration_x,
            self.acceleration_y,
            self.acceleration_z,
        )


class IMUNode(base_node.BaseNode):
    """Gathers the IMU sensor data from the module and publishes it over IPC."""

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.IMU)
        self._i2c = busio.I2C(board.SCL, board.SDA)
        self._bno = BNO08X_I2C(self._i2c)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        self._bno.enable_feature(adafruit_bno08x.BNO_REPORT_GAME_ROTATION_VECTOR)
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

    def _is_calibrated(self) -> bool:
        return enums.CalibrationStatus(self._bno.calibration_status).is_calibrated

    def _publish_imu_reading(self) -> None:
        self._update_data_cache()
        msg = ipc_messages.IMUMessage(
            roll=self._imu_data_cache.roll,
            pitch=self._imu_data_cache.pitch,
            heading=self._imu_data_cache.yaw,
            angular_velocity=self._imu_data_cache.angular_velocity(),
            acceleration=self._imu_data_cache.acceleration(),
            is_calibrated=self._imu_data_cache.is_calibrated,
        )
        self.publish(registry.Channels.IMU, msg)

    def _update_data_cache(self) -> None:
        accel_x, accel_y, accel_z = self._bno.acceleration
        mag_x, mag_y, mag_z = self._bno.magnetic
        gyro_x, gyro_y, gyro_z = self._bno.gyro
        roll, pitch, yaw = self._calc_rpy()
        stability = enums.StabilityStatus.from_stability_classification(
            self._bno.stability_classification
        )

        self._imu_data_cache = _IMUDataCache(
            acceleration_x=accel_x,
            acceleration_y=accel_y,
            acceleration_z=accel_z,
            magnetic_field_x=mag_x,
            magnetic_field_y=mag_y,
            magnetic_field_z=mag_z,
            angular_speed_x=gyro_x,
            angular_speed_y=gyro_y,
            angular_speed_z=gyro_z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            is_calibrated=self._is_calibrated(),
            stability=stability,
        )

    async def shutdown_hook(self) -> None:
        self._i2c.unlock()

    def _calc_rpy(self) -> Tuple[float, float, float]:
        """Converts the sensors quaternion measurement into an euler RPY."""
        quat_i, quat_j, quat_k, quat_real = self._bno.quaternion
        roll, pitch, yaw = geometry.quaternion_to_euler(
            quat_real, quat_i, quat_j, quat_k
        )
        return roll, pitch, yaw


if __name__ == "__main__":
    node = IMUNode()
    node.start()
