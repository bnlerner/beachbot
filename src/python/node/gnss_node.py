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
# NOTE: Updates at a maximum of 20hz. Configurable on the ublox's u-center software.
_PUB_FREQUENCY = 20


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
        self._pvt_msg: Optional[gps_messages.UbloxPVTMessage] = None

        self.add_tasks(self._poll_ublox_module)
        self.add_looped_tasks({self._publish_gnss: _PUB_FREQUENCY})
        self.add_publishers(registry.Channels.GNSS)

    async def _poll_ublox_module(self) -> None:
        """Continuously polls the Ublox module for data. Note that its update rate is
        limited in the module and can be configured in the Ublox u-center software.
        """
        while True:
            try:
                self._update_pvt_data()
            except IOError as err:
                # Can happen occasionally. Opt to just log and continue if this happens.
                log.error(f"{err=}")
            finally:
                await asyncio.sleep(0)

    def _update_pvt_data(self) -> None:
        if pvt_msg := self._ublox_gps.geo_coords():
            self._pvt_msg = gps_messages.UbloxPVTMessage.from_ublox_message(pvt_msg)
            log.data(**self._pvt_msg.__dict__)
        else:
            self._pvt_msg = None

    def _publish_gnss(self) -> None:
        if gnss_msg := self._gen_gnss_msg():
            self.publish(registry.Channels.GNSS, gnss_msg)
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
                heading=self._pvt_msg.heading_of_vehicle,
                ned_velocity=ned_velocity,
            )
        else:
            return None

    async def shutdown_hook(self) -> None:
        self._serial_conn.close()


if __name__ == "__main__":
    node = GNSSNode()
    node.start()
