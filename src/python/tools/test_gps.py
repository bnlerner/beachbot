import os
import sys

import serial  # type: ignore[import-untyped]
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers.gps import messages

# Can also use SPI here - import spidev, but we dont because USB is easier to setup.
port = serial.Serial("/dev/ttyACM0", baudrate=38400, timeout=1)
gps = UbloxGps(port)


def run() -> None:
    try:
        print("Listenting for UBX Messages.")
        while True:
            try:
                # Position, Velocity and Time message.
                if coords := gps.geo_coords():
                    msg = messages.UbloxPVTMessage.from_ublox_message(coords)
                    print(f"{msg=}\n")
                # Vehicle attitude
                if veh := gps.veh_attitude():
                    msg = messages.UbloxATTMessage.from_ublox_message(veh)
                    print(f"{msg=}\n")
                # Vehicle dynamics.
                if veh_dyn := gps.vehicle_dynamics():
                    msg = messages.UbloxINSMessage.from_ublox_message(veh_dyn)
                    print(f"{msg=}\n")
                # Status of the sensors being used for sensor fusion.
                if status := gps.esf_status():
                    msg = messages.UbloxESFStatusMessage.from_ublox_message(status)
                    print(f"{msg=}\n")
                # Status of the RF antenna.
                rf_status = gps.rf_ant_status()
                print(f"{rf_status=}\n")

            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == "__main__":
    run()
