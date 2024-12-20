from __future__ import annotations

import abc
import time
from typing import Any, ClassVar, Dict, List, Tuple, Type, TypeVar

import geometry
import pydantic
from ublox_gps import core as ublox_core  # type: ignore[import-untyped]

from drivers.gps import enums

UbloxBaseMessageT = TypeVar("UbloxBaseMessageT", bound="UbloxBaseMessage")


class UbloxBaseMessage(pydantic.BaseModel, abc.ABC):
    """A base message for Ublox which implements some helpful features all messages use."""

    cls_name: ClassVar[str]
    msg_name: ClassVar[str]
    creation: float = pydantic.Field(default_factory=time.perf_counter)
    lifetime: float = 0.5

    class Config:
        validate_assignment = True

    @classmethod
    @abc.abstractmethod
    def from_ublox_message(
        cls: Type[UbloxBaseMessageT], msg: ublox_core.Message
    ) -> UbloxBaseMessageT:
        ...

    @pydantic.model_validator(mode="before")
    @classmethod
    def ensure_timestamp(cls, values: Dict[str, Any]) -> Dict[str, Any]:
        if "creation" not in values or values["creation"] is None:
            values["creation"] = time.perf_counter()

        return values

    def is_expired(self) -> bool:
        return self.creation + self.lifetime < time.perf_counter()


class UbloxPVTMessage(UbloxBaseMessage):
    """Ublox PVT (position, velocity, time) message as described in page 390 of
    https://content.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_UBX-13003221.pdf
    """

    cls_name: ClassVar[str] = "NAV"
    msg_name: ClassVar[str] = "PVT"

    # Type of fix from the Ublox.
    fix_type: enums.FixType
    # Number of satellites used in fix
    num_satellites: int
    # Both in degrees with scaling 1e-7
    longitude: float
    latitude: float
    # height above the ellipsoid in m
    ellipsoid_height: float
    # Height above mean sea level in m
    height_above_sea_level: float
    # horizontal accuracy in m
    h_acc: float
    # Vertical accuracy in m
    v_acc: float
    # North velocity (m/s) in NED convention
    north_velocity: float
    # East velocity (m/s) in NED convention
    east_velocity: float
    # Down velocity (m/s) in NED convention
    down_velocity: float
    # Ground speed (m/s) in 2D
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
            # Convert from mm to m as these are reported in mm in the ublox module.
            ellipsoid_height=msg.height / 1000,
            height_above_sea_level=msg.hMSL / 1000,
            h_acc=msg.hAcc / 1000,
            v_acc=msg.vAcc / 1000,
            north_velocity=msg.velN / 1000,
            east_velocity=msg.velE / 1000,
            down_velocity=msg.velD / 1000,
            ground_speed=msg.gSpeed / 1000,
            heading_of_vehicle=msg.headVeh,
            heading_of_motion=msg.headMot,
            heading_acc=msg.hAcc,
            gps_quality=enums.GPSQuality(msg.flags.carrSoln),
        )

    def is_valid(self) -> bool:
        return self.num_satellites > 0 and self.fix_type.value != 0


class UbloxATTMessage(UbloxBaseMessage):
    """This message outputs the attitude solution as RPH angles."""

    cls_name: ClassVar[str] = "NAV"
    msg_name: ClassVar[str] = "ATT"

    # Vehicle RPH in degrees
    roll: float
    pitch: float
    heading: float

    @classmethod
    def from_ublox_message(cls, msg: ublox_core.Message) -> UbloxATTMessage:
        return UbloxATTMessage(roll=msg.roll, pitch=msg.pitch, heading=msg.heading)


class UbloxINSMessage(UbloxBaseMessage):
    """Ublox INS (inertial navigation solution) message which is primarily concerned
    with vehicle dynamics. All ADR product (this one) are relative to the VEHICLE frame.
    """

    cls_name: ClassVar[str] = "ESF"
    msg_name: ClassVar[str] = "INS"

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
            is_valid=all(
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


class UbloxESFStatusMessage(UbloxBaseMessage):
    """Ublox Status of the external sensor fusion"""

    cls_name: ClassVar[str] = "ESF"
    msg_name: ClassVar[str] = "STATUS"

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


class UbloxCOVMessage(UbloxBaseMessage):
    """Ublox COV (covariance matrices) message which contains information on correlation
    between axes of the position and velocity solutions in the topocentric coordinate
    system defined as the local-level North (N), East (E), Down (D) frame.
    """

    cls_name: ClassVar[str] = "NAV"
    msg_name: ClassVar[str] = "COV"
    # Longer lifetime because this message takes a while to get from the ublox module.
    lifetime: float = 60.0

    position_covariance: List[List[float]]
    velocity_covariance: List[List[float]]
    # Whether the covariance matrice is considered valid by the ublox module.
    is_data_valid: bool

    @classmethod
    def from_ublox_message(cls, msg: ublox_core.Message) -> UbloxCOVMessage:
        """Generates the covariance message from the ublox message. As the covariance
        matrices are symmetric, only the upper triangular part is output.

        Position Standard Deviations:
            posCovNN: Standard deviation along the North axis in m^2.
            posCovEE: Standard deviation along the East axis in m^2.
            posCovDD: Standard deviation along the Down axis in m^2.
        Position Correlation Coefficients:
            posCovNE: Correlation between North and East in m^2.
            posCovND: Correlation between North and Down in m^2.
            posCovED: Correlation between East and Down in m^2.
        Velocity Standard Deviations:
            velCovNN, velCovEE, velCovDD in m^2/s^2.
        Velocity Correlation Coefficients:
            velCovNE, velCovND, velCovED in m^2/s^2.
        """
        return UbloxCOVMessage(
            position_covariance=_gen_covariance_matrix(
                (msg.posCovNN, msg.posCovEE, msg.posCovDD),
                (msg.posCovNE, msg.posCovND, msg.posCovED),
            ),
            velocity_covariance=_gen_covariance_matrix(
                (msg.velCovNN, msg.velCovEE, msg.velCovDD),
                (msg.velCovNE, msg.velCovND, msg.velCovED),
            ),
            is_data_valid=all([msg.posCovValid, msg.velCovValid]),
        )

    def is_valid(self) -> bool:
        return self.is_data_valid and not self.is_expired()


def _gen_covariance_matrix(
    std_dev: Tuple[float, float, float], correlations: Tuple[float, float, float]
) -> List[List[float]]:
    """Create a 3x3 covariance matrix from the standard deviation of (sigma_x,
    sigma_y, sigma_z) and correlation coefficients (rho_xy, rho_xz, rho_yz).
    """
    sigma_x, sigma_y, sigma_z = std_dev
    rho_xy, rho_xz, rho_yz = correlations

    cov_matrix = [
        [sigma_x**2, rho_xy * sigma_x * sigma_y, rho_xz * sigma_x * sigma_z],
        [rho_xy * sigma_x * sigma_y, sigma_y**2, rho_yz * sigma_y * sigma_z],
        [rho_xz * sigma_x * sigma_z, rho_yz * sigma_y * sigma_z, sigma_z**2],
    ]

    return cov_matrix
