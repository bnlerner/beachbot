from __future__ import annotations

import geometry
import pydantic
from ublox_gps import core as ublox_core  # type: ignore[import-untyped]

from drivers.gps import enums


class UbloxPVTMessage(pydantic.BaseModel):
    """Ublox PVT (position, velocity, time) message as described in page 390 of
    https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
    """

    # Type of fix from the Ublox.
    fix_type: enums.FixType
    # Number of satellites used in fix
    num_satellites: int
    # Both in degrees with scaling 1e-7
    longitude: float
    latitude: float
    # height above the ellipsoid in mm
    ellipsoid_height: float
    # Height above mean sea level in mm
    height_above_sea_level: float
    # horizontal accuracy in mm
    h_acc: float
    # Vertical accuracy in mm
    v_acc: float
    # North velocity (mm/s) in NED convention
    north_velocity: float
    # East velocity (mm/s) in NED convention
    east_velocity: float
    # Down velocity (mm/s) in NED convention
    down_velocity: float
    # Ground speed (mm/s) in 2D
    ground_speed: float
    # Heading of the vehicle. Set to the same value as heading of motion if receiver is
    # not in sensor fusion mode.
    heading_of_vehicle: float
    # Direction the GPS is moving in degrees
    heading_of_motion: float
    # Accuracy of heading in degrees
    heading_acc: float
    # Carrier solution
    gps_quality: enums.GPSQuality

    @classmethod
    def from_ublox_message(cls, msg: ublox_core.Message) -> UbloxPVTMessage:
        return UbloxPVTMessage(
            fix_type=enums.FixType(msg.fixType),
            num_satellites=msg.numSV,
            longitude=msg.lon,
            latitude=msg.lat,
            ellipsoid_height=msg.height,
            height_above_sea_level=msg.hMSL,
            h_acc=msg.hAcc,
            v_acc=msg.vAcc,
            north_velocity=msg.velN,
            east_velocity=msg.velE,
            down_velocity=msg.velD,
            ground_speed=msg.gSpeed,
            heading_of_vehicle=msg.headVeh,
            heading_of_motion=msg.headMot,
            heading_acc=msg.hAcc,
            gps_quality=enums.GPSQuality(msg.flags.carrSoln),
        )

    def is_valid(self) -> bool:
        return self.num_satellites > 0 and self.fix_type.value != 0


class UbloxATTMessage(pydantic.BaseModel):
    """This message outputs the attitude solution as RPH angles."""

    # Vehicle RPH in degrees
    roll: float
    pitch: float
    heading: float

    @classmethod
    def from_ublox_message(cls, msg: ublox_core.Message) -> UbloxATTMessage:
        return UbloxATTMessage(roll=msg.roll, pitch=msg.pitch, heading=msg.heading)


class UbloxINSMessage(pydantic.BaseModel):
    """Ublox INS (inertial navigation solution) message which is primarily concerned
    with vehicle dynamics. All ADR product (this one) are relative to the VEHICLE frame.
    """

    # Angular rate of procession around the XYZ axis of the vehicle. (deg/s)
    x_axis_angular_rate: float
    y_axis_angular_rate: float
    z_axis_angular_rate: float
    # Acceleration along XYZ axis's removing the effect of gravity. (m/s^2)
    x_axis_acceleration: float
    y_axis_acceleration: float
    z_axis_acceleration: float
    is_valid: float

    @classmethod
    def from_ublox_message(cls, msg: ublox_core.Message) -> UbloxINSMessage:
        val_fields = msg.biltfield0
        return UbloxINSMessage(
            x_axis_angular_rate=msg.xAngRate,
            y_axis_angular_rate=msg.yAngRate,
            z_axis_angular_rate=msg.zAngRate,
            x_axis_acceleration=msg.xAccel,
            y_axis_acceleration=msg.yAccel,
            z_axis_acceleration=msg.zAccel,
            is_valid=not all(
                [
                    val_fields.xAngRateValid,
                    val_fields.yAngRateValid,
                    val_fields.zAngRateValid,
                    val_fields.xAccelValid,
                    val_fields.yAccelValid,
                    val_fields.zAccelValid,
                ]
            ),
        )

    def get_spin(self) -> geometry.AngularVelocity:
        return geometry.AngularVelocity(
            geometry.VEHICLE,
            self.x_axis_angular_rate,
            self.y_axis_angular_rate,
            self.z_axis_angular_rate,
        )

    def get_angular_acceleration(self) -> geometry.AngularAcceleration:
        return geometry.AngularAcceleration(
            geometry.VEHICLE,
            self.x_axis_acceleration,
            self.y_axis_acceleration,
            self.z_axis_acceleration,
        )


class UbloxESFStatusMessage(pydantic.BaseModel):
    """Ublox Status of the external sensor fusion"""

    fusion_mode: enums.SensorFusionMode
    wheel_tick: enums.SensorStatus
    auto_imu_mount_alignment: enums.SensorStatus
    ins: enums.SensorStatus
    imu: enums.SensorStatus

    @classmethod
    def from_ublox_message(cls, msg: ublox_core.Message) -> UbloxESFStatusMessage:
        return UbloxESFStatusMessage(
            fusion_mode=enums.SensorFusionMode(msg.fusionMode),
            wheel_tick=enums.SensorStatus(msg.initStatus1.wtInitStatus),
            auto_imu_mount_alignment=enums.SensorStatus(msg.initStatus1.mntAlgStatus),
            ins=enums.SensorStatus(msg.initStatus1.insInitStatus),
            imu=enums.SensorStatus(msg.initStatus2.imuInitStatus),
        )
