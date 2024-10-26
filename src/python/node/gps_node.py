from ublox_gps import UbloxGps
import serial
from typing import Optional

from node import base_node
from drivers.gps import messages
import logging

_TIMEOUT = 1.0
_BAUDRATE = 38400
_PORT = "/dev/ttyACM0"


class GPSNode(base_node.BaseNode):

    def __init__(self):
        self._port = serial.Serial(_PORT, baudrate=_BAUDRATE, timeout=_TIMEOUT)
        self._ublox_gps = UbloxGps(self._port)

        self.add_tasks(self._listen)

    def _listen(self) -> None:
        try:
            if gps_msg := self._gen_gps_message():
                gps_msg.write()

            if veh_msg := self._gen_vehicle_message():
               veh_msg.write()

        except IOError as err:
            logging.error(f"{err=}")
        
    def _gen_vehicle_message(self) -> Optional[messages.VehicleDynamicsMessage]:
        if (veh_attitude := self._ublox_gps.veh_attitude()) and (veh_dyn := self._ublox_gps.vehicle_dynamics()):
            return messages.VehicleDynamicsMessage(
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

        return None

    def _gen_gps_message(self) -> Optional[messages.GPSMessage]:
        if coords := self._ublox_gps.geo_coords():
            return messages.GPSMessage(latitude=coords.lat, longitude=coords.lon)

        return None

    def shutdown_hook(self):
        self._port.close()


if __name__ == "__main__":
    node = GPSNode()
    node.start()
