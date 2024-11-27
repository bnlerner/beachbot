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
import system_info
from drivers.gps import messages as gps_messages
from ipc import messages as ipc_messages
from ipc import registry

from node import base_node

_TIMEOUT = 1.0
_BAUDRATE = 38400
_PUB_FREQUENCY = 5


class UbloxDataNode(base_node.BaseNode):
    """Gathers GPS and IMU data from the Ublox data module and publishes this
    data for all to use.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.UBLOX_DATA)
        self._serial_conn = serial.Serial(
            system_info.UBLOX_SERIAL, baudrate=_BAUDRATE, timeout=_TIMEOUT
        )
        self._ublox_gps = UbloxGps(self._serial_conn)

        self._att_msg: Optional[gps_messages.UbloxATTMessage] = None
        self._pvt_msg: Optional[gps_messages.UbloxPVTMessage] = None
        self._ins_msg: Optional[gps_messages.UbloxINSMessage] = None

        self.add_tasks(self._query_ublox)
        self.add_looped_tasks(
            {self._publish_gnss: _PUB_FREQUENCY, self._publish_esf: _PUB_FREQUENCY}
        )
        self.add_publishers(registry.Channels.GNSS, registry.Channels.ESF)

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

    def _publish_gnss(self) -> None:
        if gnss_msg := self._gen_gnss_msg():
            self.publish(registry.Channels.GNSS, gnss_msg)
            self._pvt_msg = None

    def _publish_esf(self) -> None:
        if esf_msg := self._gen_esf_msg():
            self.publish(registry.Channels.ESF, esf_msg)
            self._att_msg = None
            self._ins_msg = None

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

    def _gen_gnss_msg(self) -> Optional[ipc_messages.GNSSMessage]:
        if self._pvt_msg and self._pvt_msg.is_valid():
            ned_velocity = geometry.Velocity(
                geometry.UTM,
                self._pvt_msg.east_velocity,
                self._pvt_msg.north_velocity,
                -self._pvt_msg.down_velocity,
            )
            return ipc_messages.GNSSMessage(
                latitude=self._pvt_msg.latitude,
                longitude=self._pvt_msg.longitude,
                ellipsoid_height=self._pvt_msg.ellipsoid_height,
                ned_velocity=ned_velocity,
            )
        else:
            return None

    def _gen_esf_msg(self) -> Optional[ipc_messages.ESFMessage]:
        if self._ins_msg and self._att_msg:
            spin = geometry.AngularVelocity(
                geometry.VEHICLE,
                self._ins_msg.x_axis_angular_rate,
                self._ins_msg.y_axis_angular_rate,
                self._ins_msg.z_axis_angular_rate,
            )
            angular_accel = geometry.AngularAcceleration(
                geometry.VEHICLE,
                self._ins_msg.x_axis_acceleration,
                self._ins_msg.y_axis_acceleration,
                self._ins_msg.z_axis_acceleration,
            )
            return ipc_messages.ESFMessage(
                roll=self._att_msg.roll,
                pitch=self._att_msg.pitch,
                heading=self._att_msg.heading,
                angular_velocity=spin,
                angular_acceleration=angular_accel,
            )
        else:
            return None

    async def shutdown_hook(self) -> None:
        self._serial_conn.close()



if __name__ == "__main__":
    node = UbloxDataNode()
    node.start()
