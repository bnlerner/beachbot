import asyncio
import os
import sys
import time
from typing import Optional

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import log
import serial  # type: ignore[import-untyped]
from drivers.gps import messages as gps_messages
from ipc import messages as ipc_messages
from ipc import registry
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

from node import base_node

_TIMEOUT = 1.0
_BAUDRATE = 38400
_PORT = "/dev/ttyACM0"
_PUBLISH_RATE = 2  # In Hz


class UbloxDataNode(base_node.BaseNode):
    """Gathers GPS and IMU data from the Ublox data module and publishes this
    data for all to use.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.UBLOX_DATA)
        self._serial_conn = serial.Serial(_PORT, baudrate=_BAUDRATE, timeout=_TIMEOUT)
        self._ublox_gps = UbloxGps(self._serial_conn)

        self._att_data: Optional[gps_messages.UbloxATTMessage] = None
        self._pvt_data: Optional[gps_messages.UbloxPVTMessage] = None
        self._ins_data: Optional[gps_messages.UbloxINSMessage] = None

        self.add_tasks(self._listen, self._publish)
        self.add_publishers(registry.Channels.BODY_DYNAMICS, registry.Channels.BODY_GPS)

    async def _listen(self) -> None:
        while True:
            try:
                self._update_vehicle_attitude_data()
                await asyncio.sleep(0)
                self._update_gps_data()
                await asyncio.sleep(0)
                self._update_vehicle_ins_data()
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

    def _update_vehicle_attitude_data(self) -> None:
        if veh_att := self._ublox_gps.veh_attitude():
            self._att_data = gps_messages.UbloxATTMessage.from_ublox_message(veh_att)
            log.data(**self._att_data.__dict__)
        else:
            self._att_data = None

    def _update_vehicle_ins_data(self) -> None:
        if veh_dyn := self._ublox_gps.vehicle_dynamics():
            self._ins_data = gps_messages.UbloxINSMessage.from_ublox_message(veh_dyn)
            log.data(**self._ins_data.__dict__)
        else:
            self._ins_data = None

    def _update_gps_data(self) -> None:
        if pvt_msg := self._ublox_gps.geo_coords():
            self._pvt_data = gps_messages.UbloxPVTMessage.from_ublox_message(pvt_msg)
            log.data(**self._pvt_data.__dict__)
        else:
            self._pvt_data = None

    def _gen_vehicle_message(self) -> Optional[ipc_messages.VehicleDynamicsMessage]:
        if self._att_data and self._ins_data:
            return ipc_messages.VehicleDynamicsMessage(
                roll=self._att_data.roll,
                pitch=self._att_data.pitch,
                heading=self._att_data.heading,
                roll_rate=self._ins_data.x_axis_angular_rate,
                # Negative because the pitch axis (y-axis) and the yaw axis (z-axis) are
                # both pointed in the opposite direction from the VEHICLE frame
                # (sensor reported) vs BODY frame (desired).
                pitch_rate=-self._ins_data.y_axis_angular_rate,
                yaw_rate=-self._ins_data.z_axis_angular_rate,
            )

        return None

    def _gen_gps_message(self) -> Optional[ipc_messages.GPSMessage]:
        if self._pvt_data:
            return ipc_messages.GPSMessage(
                latitude=self._pvt_data.latitude,
                longitude=self._pvt_data.longitude,
                ellipsoid_height=self._pvt_data.height,
            )

        return None

    async def shutdown_hook(self) -> None:
        self._serial_conn.close()


if __name__ == "__main__":
    node = UbloxDataNode()
    node.start()
