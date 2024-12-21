import asyncio
import os
import sys
import time
from typing import Optional, Type

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
# NOTE: Updates at a maximum of 20hz. Configurable on the ublox's u-center software.
_PUB_FREQUENCY = 20
_WAIT_TIME = 60


class GNSSNode(base_node.BaseNode):
    """Gathers GNSS data from the Ublox data module and publishes this data for all to
    use. Specifically does not use IMU / ESF data from the Ublox module because wheel
    ticks seem to be required to enable ESF data to be published.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.GNSS)
        self._serial_conn = serial.Serial(
            system_info.UBLOX_SERIAL, baudrate=_BAUDRATE, timeout=_TIMEOUT
        )
        self._ublox_gps = UbloxGps(self._serial_conn)
        self._cov_msg: Optional[gps_messages.UbloxCOVMessage] = None
        self._pvt_msg: Optional[gps_messages.UbloxPVTMessage] = None

        self.add_tasks(self._update_cov_data, self._update_pvt_data)
        self.add_looped_tasks({self._publish_gnss: _PUB_FREQUENCY})
        self.add_publishers(registry.Channels.GNSS)

    async def _update_cov_data(self) -> None:
        """Continuously polls the Ublox module for data. Note that its update rate is
        limited in the module and can be configured in the Ublox u-center software.
        """
        while True:
            if cov_msg := await self._poll_for_message(gps_messages.UbloxCOVMessage):
                self._cov_msg = cov_msg
                log.data(**self._cov_msg.__dict__)
            else:
                self._cov_msg = None

    async def _update_pvt_data(self) -> None:
        while True:
            if pvt_msg := await self._poll_for_message(gps_messages.UbloxPVTMessage):
                self._pvt_msg = pvt_msg
                log.data(**self._pvt_msg.__dict__)
            else:
                self._pvt_msg = None

    async def _poll_for_message(
        self, msg: Type[gps_messages.UbloxBaseMessageT]
    ) -> Optional[gps_messages.UbloxBaseMessageT]:
        """Reimplementation of the ublox gps package 'request_standard_packet' in an
        async way to allow for efficient message polling between messages with different
        update frequencies.
        """
        self._ublox_gps.set_packet(msg.cls_name, msg.msg_name, None)
        self._ublox_gps.send_message(msg.cls_name, msg.msg_name, None)

        start = time.perf_counter()
        while msg.msg_name not in self._ublox_gps.packets[msg.cls_name]:
            await asyncio.sleep(0.01)

            if time.perf_counter() - start > _WAIT_TIME:
                break

        if ublox_msg := self._ublox_gps.packets[msg.cls_name].get(msg.msg_name, None):
            return msg.from_ublox_message(self._ublox_gps.scale_packet(ublox_msg))
        else:
            return None

    def _publish_gnss(self) -> None:
        if gnss_msg := self._gen_gnss_msg():
            self.publish(registry.Channels.GNSS, gnss_msg)
            self._pvt_msg = None

    def _gen_gnss_msg(self) -> Optional[ipc_messages.GNSSMessage]:
        if (
            self._pvt_msg
            and self._cov_msg
            and self._pvt_msg.is_valid()
            and self._cov_msg.is_valid()
        ):
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
                heading_of_motion=self._pvt_msg.heading_of_motion,
                ned_velocity=ned_velocity,
                position_covariance=self._cov_msg.position_covariance,
                velocity_covariance=self._cov_msg.velocity_covariance,
            )
        else:
            return None

    async def shutdown_hook(self) -> None:
        self._serial_conn.close()


if __name__ == "__main__":
    node = GNSSNode()
    node.start()
