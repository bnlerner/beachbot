import asyncio
import dataclasses
import os
import sys
import time
from typing import Optional

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
import serial  # type: ignore[import-untyped]
from ipc import messages, registry
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

from node import base_node

_TIMEOUT = 1.0
_BAUDRATE = 38400
_PORT = "/dev/ttyACM0"
_PUBLISH_RATE = 2  # In Hz


@dataclasses.dataclass
class _RPYDynamicsData:
    """Information from the IMU on the Ublox module."""

    roll: float
    pitch: float
    heading: float
    roll_rate: float
    pitch_rate: float
    yaw_rate: float
    roll_acceleration: float
    pitch_acceleration: float
    heading_acceleration: float


@dataclasses.dataclass
class _GPSData:
    """Information directly from the GPS on the Ublox module."""

    latitude: float
    longitude: float


class UbloxDataNode(base_node.BaseNode):
    """Gathers GPS and IMU data from the Ublox data module and publishes this
    data for all to use.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.UBLOX_DATA)
        self._serial_conn = serial.Serial(_PORT, baudrate=_BAUDRATE, timeout=_TIMEOUT)
        self._ublox_gps = UbloxGps(self._serial_conn)

        self._rpy_dyn_data: Optional[_RPYDynamicsData] = None
        self._gps_data: Optional[_GPSData] = None

        self.add_tasks(self._listen, self._publish)
        self.add_publishers(registry.Channels.BODY_DYNAMICS, registry.Channels.BODY_GPS)

    async def _listen(self) -> None:
        while True:
            try:
                self._update_rpy_dyn_data()
                await asyncio.sleep(0)
                self._update_gps_data()
                await asyncio.sleep(0)
            except IOError as err:
                log.error(f"{err=}")

    async def _publish(self) -> None:
        total_time = 1 / _PUBLISH_RATE
        while True:
            start_time = time.perf_counter()
            if gps_msg := self._gen_gps_message():
                self.publish(registry.Channels.BODY_GPS, gps_msg)

            if veh_msg := self._gen_vehicle_message():
                self.publish(registry.Channels.BODY_DYNAMICS, veh_msg)

            write_time = time.perf_counter() - start_time
            sleep_time = total_time - write_time
            await asyncio.sleep(sleep_time)

    def _update_rpy_dyn_data(self) -> None:
        veh_attitude = self._ublox_gps.veh_attitude()
        veh_dyn = self._ublox_gps.vehicle_dynamics()
        if veh_attitude and veh_dyn:
            self._rpy_dyn_data = _RPYDynamicsData(
                roll=veh_attitude.roll,
                pitch=veh_attitude.pitch,
                heading=veh_attitude.heading,
                roll_rate=veh_dyn.xAngRate,
                pitch_rate=veh_dyn.yAngRate,
                yaw_rate=veh_dyn.zAngRate,
                roll_acceleration=veh_attitude.accRoll,
                pitch_acceleration=veh_attitude.accPitch,
                heading_acceleration=veh_attitude.accHeading,
            )
        else:
            self._rpy_dyn_data = None

    def _update_gps_data(self) -> None:
        if coords := self._ublox_gps.geo_coords():
            self._gps_data = _GPSData(latitude=coords.lat, longitude=coords.lon)
        else:
            self._gps_data = None

    def _gen_vehicle_message(self) -> Optional[messages.VehicleDynamicsMessage]:
        if self._rpy_dyn_data:
            return messages.VehicleDynamicsMessage(
                roll=self._rpy_dyn_data.roll,
                pitch=self._rpy_dyn_data.pitch,
                heading=self._rpy_dyn_data.heading,
                roll_rate=self._rpy_dyn_data.roll_rate,
                pitch_rate=self._rpy_dyn_data.pitch_rate,
                yaw_rate=self._rpy_dyn_data.yaw_rate,
                roll_acceleration=self._rpy_dyn_data.roll_acceleration,
                pitch_acceleration=self._rpy_dyn_data.pitch_acceleration,
                heading_acceleration=self._rpy_dyn_data.heading_acceleration,
            )

        return None

    def _gen_gps_message(self) -> Optional[messages.GPSMessage]:
        if self._gps_data:
            return messages.GPSMessage(
                latitude=self._gps_data.latitude, longitude=self._gps_data.longitude
            )

        return None

    async def shutdown_hook(self) -> None:
        self._serial_conn.close()


if __name__ == "__main__":
    node = UbloxDataNode()
    node.start()
