import asyncio
import os
import sys
from typing import Optional

import serial  # type: ignore[import-untyped]
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import geometry
import log
from drivers.gps import messages as gps_messages
from ipc import messages as ipc_messages
from ipc import registry
from localization import gps_transformer, primitives

from node import base_node

_TIMEOUT = 1.0
_BAUDRATE = 38400
_PORT = "/dev/ttyACM0"
# UTM zone for Florida is zone number 17N.
_DEFAULT_UTM_ZONE = primitives.UTMZone(
    zone_number=17, hemisphere="north", epsg_code="EPSG:32617"
)


class UbloxDataNode(base_node.BaseNode):
    """Gathers GPS and IMU data from the Ublox data module and publishes this
    data for all to use.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.UBLOX_DATA)
        self._serial_conn = serial.Serial(_PORT, baudrate=_BAUDRATE, timeout=_TIMEOUT)
        self._ublox_gps = UbloxGps(self._serial_conn)
        self._gps_transformer = gps_transformer.GPSTransformer(_DEFAULT_UTM_ZONE)

        self._att_msg: Optional[gps_messages.UbloxATTMessage] = None
        self._pvt_msg: Optional[gps_messages.UbloxPVTMessage] = None
        self._ins_msg: Optional[gps_messages.UbloxINSMessage] = None

        self.add_tasks(self._query_ublox, self._publish)
        self.add_publishers(registry.Channels.BODY_KINEMATICS)

    async def _query_ublox(self) -> None:
        """Continuously queries the Ublox module for data. Note that its update rate is
        limited by the ublox u-center software and can be configured there.
        """
        while True:
            try:
                self._update_vehicle_attitude_data()
                self._update_gps_data()
                self._update_vehicle_ins_data()
            except IOError as err:
                # Can happen occasionally. Opt to just log and continue if this happens.
                log.error(f"{err=}")
            finally:
                await asyncio.sleep(0)

    async def _publish(self) -> None:
        """Publishes as fast as data can update."""
        while True:
            if veh_msg := self._gen_vehicle_kin_msg():
                self.publish(registry.Channels.BODY_KINEMATICS, veh_msg)
                self._clear_data_cache()

            await asyncio.sleep(0)

    def _update_vehicle_attitude_data(self) -> None:
        if veh_att := self._ublox_gps.veh_attitude():
            self._att_msg = gps_messages.UbloxATTMessage.from_ublox_message(veh_att)
            log.data(**self._att_msg.__dict__)
        else:
            self._att_msg = None

    def _update_vehicle_ins_data(self) -> None:
        if veh_dyn := self._ublox_gps.vehicle_dynamics():
            self._ins_msg = gps_messages.UbloxINSMessage.from_ublox_message(veh_dyn)
            log.data(**self._ins_msg.__dict__)
        else:
            self._ins_msg = None

    def _update_gps_data(self) -> None:
        if pvt_msg := self._ublox_gps.geo_coords():
            self._pvt_msg = gps_messages.UbloxPVTMessage.from_ublox_message(pvt_msg)
            log.data(**self._pvt_msg.__dict__)
        else:
            self._pvt_msg = None

    def _clear_data_cache(self) -> None:
        self._pvt_msg = None
        self._att_msg = None
        self._ins_msg = None

    def _gen_vehicle_kin_msg(self) -> Optional[ipc_messages.VehicleKinematicsMessage]:
        if self._att_msg and self._ins_msg and self._pvt_msg:
            position = self._gps_transformer.transform_position(
                self._pvt_msg.longitude,
                self._pvt_msg.latitude,
                ellipsoid_height=self._pvt_msg.ellipsoid_height,
            )
            yaw = self._gps_transformer.transform_heading(
                self._pvt_msg.longitude, self._pvt_msg.latitude, self._att_msg.heading
            )
            orientation = self._calc_orientation(self._att_msg, yaw)
            pose = geometry.Pose(position, orientation)
            velocity = self._calc_velocity(self._pvt_msg, orientation)
            angular_velocity = self._calc_angular_velocity(self._ins_msg)
            return ipc_messages.VehicleKinematicsMessage(
                pose=pose, twist=geometry.Twist(velocity, angular_velocity)
            )

        return None

    def _calc_angular_velocity(
        self, ins_msg: gps_messages.UbloxINSMessage
    ) -> geometry.AngularVelocity:
        roll_rate = ins_msg.x_axis_angular_rate
        # Negative because the pitch axis (y-axis) and the yaw axis (z-axis) are
        # both pointed in the opposite direction from the VEHICLE frame
        # (sensor reported) vs BODY frame (desired).
        # angular speed rates in deg/s
        pitch_rate = -ins_msg.y_axis_angular_rate
        yaw_rate = -ins_msg.z_axis_angular_rate
        return geometry.AngularVelocity(
            geometry.VEHICLE,
            ins_msg.x_axis_angular_rate,
            ins_msg.y_axis_angular_rate,
            ins_msg.z_axis_angular_rate,
        )

    def _calc_orientation(
        self, att_msg: gps_messages.UbloxATTMessage, yaw: float
    ) -> geometry.Orientation:
        veh_ori = geometry.Orientation.from_intrinsic_rpy(
            geometry.UTM, att_msg.roll, att_msg.pitch, 0.0
        )
        roll, pitch = veh_ori.roll, veh_ori.pitch
        return geometry.Orientation(geometry.UTM, roll, pitch, yaw)

    def _calc_velocity(
        self, pvt_msg: gps_messages.UbloxPVTMessage, veh_ori: geometry.Orientation
    ) -> geometry.Velocity:
        """Calculates the vehicle velocity."""
        utm_velocity = geometry.Velocity(
            geometry.UTM,
            pvt_msg.east_velocity,
            pvt_msg.north_velocity,
            -pvt_msg.down_velocity,
        )
        veh_velocity = utm_velocity.rotated(veh_ori.as_rotation().inverted())
        veh_velocity.frame = geometry.VEHICLE

        return veh_velocity

    async def shutdown_hook(self) -> None:
        self._serial_conn.close()


if __name__ == "__main__":
    node = UbloxDataNode()
    node.start()
