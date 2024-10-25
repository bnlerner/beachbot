
from ublox_gps import UbloxGps
import serial
# Can also use SPI here - import spidev
# I2C is not supported

#port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
# port = serial.Serial('/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00', baudrate=38400, timeout=1)
gps = UbloxGps(port)

def run():
  
  try: 
    print("Listenting for UBX Messages.")
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

