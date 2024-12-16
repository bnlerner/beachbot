import asyncio
import dataclasses
import math
import os
import sys

import adafruit_bno08x  # type:ignore[import-untyped]
import board  # type:ignore[import-untyped]
import busio  # type:ignore[import-untyped]
from adafruit_bno08x.i2c import BNO08X_I2C  # type:ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
from drivers.imu import enums
from ipc import messages as ipc_messages
from ipc import registry, session

from node import base_node

_PUB_FREQUENCY = 50
_CALIBRATE_STATUS_SLEEP = 1.0


@dataclasses.dataclass
class _IMUDataCache:
    # Gyro rotation measurement on the XYZ axis in rad/s
    angular_speed_x: float
    angular_speed_y: float
    angular_speed_z: float

    # The roll and pitch of the IMU (degrees) in the BODY frame.
    roll: float
    pitch: float

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
            angular_velocity=self._imu_data_cache.angular_velocity(),
            is_calibrated=self._imu_data_cache.is_calibrated,
        )
        self.publish(registry.Channels.IMU, msg)

    def _update_data_cache(self) -> None:
        gyro_x, gyro_y, gyro_z = self._bno.gyro
        body_ori = self._calc_body_ori()
        stability = enums.StabilityStatus.from_stability_classification(
            self._bno.stability_classification
        )

        self._imu_data_cache = _IMUDataCache(
            angular_speed_x=gyro_x,
            angular_speed_y=gyro_y,
            angular_speed_z=gyro_z,
            roll=body_ori.roll,
            pitch=body_ori.pitch,
            is_calibrated=self._is_calibrated(),
            stability=stability,
        )

    async def shutdown_hook(self) -> None:
        self._i2c.unlock()

    def _calc_body_ori(self) -> geometry.Orientation:
        """Converts the sensors quaternion measurement into an euler RPY orientation in
        the BODY frame.
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


if __name__ == "__main__":
    node = IMUNode()
    node.start()
