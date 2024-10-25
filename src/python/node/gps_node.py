from ublox_gps import UbloxGps
import serial
from typing import Optional

from node import base_node
from drivers.gps import messages
from ipc import session

_TIMEOUT = 1.0
_BAUDRATE = 38400
_PORT = "/dev/ttyACM0"
_MSG_DIR = "var/"


class GPSNode(base_node.BaseNode):

    def __init__(self):
        self._port = serial.Serial(_PORT, baudrate=_BAUDRATE, timeout=_TIMEOUT)
        self._ublox_gps = UbloxGps(self._port)

        self.add_tasks(self._listen)

    def _listen(self) -> None:
        try:
            gps_msg = self._gen_gps_message()

        except IOError as err:
            print(f"{err=}")
        
        
            
        if veh_attitude := self._ublox_gps.veh_attitude():
            print("Roll: ", veh.roll)
            print("Pitch: ", veh.pitch)
            print("Heading: ", veh.heading)
            print("Roll Acceleration: ", veh.accRoll)
            print("Pitch Acceleration: ", veh.accPitch)
            print("Heading Acceleration: ", veh.accHeading)

    def _gen_gps_message(self) -> Optional[messages.GPSMessage]:
        if coords := self._ublox_gps.geo_coords():
            return messages.GPSMessage(latitude=coords.lat, longitude=coords.lon)

        return None

    def shutdown_hook(self):
        self._port.close()

def run():
  
  try: 
    print("Listening for UBX Messages.")
    while True:
      try: 
        coords = gps.geo_coords()
        veh = gps.veh_attitude()
        print(coords.lon, coords.lat)
        print(f"{veh}")
        # print(f"{gps.vehicle_dynamics()=}")
        # print(f"{gps.esf_measures()=}")
        # print(f"{gps.esf_status()=}")
        # print(f"{gps.geo_cov()=}")
        # print(f"{gps.get_DOP()=}")
        # print(f"{gps.hp_geo_coords()=}")
        # print(f"{gps.rf_ant_status()=}")
        # print(f"{gps.imu_alignment()=}")
        # The payload length does not match the length implied by the message fields. Expected 20 actual 16
        # print(f"{gps.satellites()=}")
        # print(f"{gps.pin_settings()=}")
        # print(f"{gps.port_settings()=}")
        # Get NMEA Protocol Version
        # veh = gps.veh_attitude()
        # print("Roll: ", veh.roll)
        # print("Pitch: ", veh.pitch)
        # print("Heading: ", veh.heading)
        # print("Roll Acceleration: ", veh.accRoll)
        # print("Pitch Acceleration: ", veh.accPitch)
        # print("Heading Acceleration: ", veh.accHeading)
        # print(f"{gps.stream_nmea()=}")
      except (ValueError, IOError) as err:
        print(err)
  
  finally:
    port.close()

if __name__ == '__main__':
  run()



if __name__ == "__main__":
    node = GPSNode()
    node.start()
