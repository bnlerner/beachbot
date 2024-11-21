import os
import sys
import time

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
        print("Listening for UBX Messages.")
        while True:
            try:
                start = time.perf_counter()
                # Position, Velocity and Time message.
                if coords := gps.geo_coords():
                    pvt_msg = messages.UbloxPVTMessage.from_ublox_message(coords)
                    print(f"{pvt_msg=}\n")
                # Vehicle attitude
                if veh_att := gps.veh_attitude():
                    att_msg = messages.UbloxATTMessage.from_ublox_message(veh_att)
                    print(f"{att_msg=}\n")
                # Vehicle dynamics.
                if veh_dyn := gps.vehicle_dynamics():
                    ins_msg = messages.UbloxINSMessage.from_ublox_message(veh_dyn)
                    print(f"{ins_msg=}\n")
                # Status of the sensors being used for sensor fusion.
                if status := gps.esf_status():
                    status_msg = messages.UbloxESFStatusMessage.from_ublox_message(
                        status
                    )
                    print(f"{status_msg=}\n")
                # Status of the RF antenna.
                if rf_status := gps.rf_ant_status():
                    print(f"{rf_status=}\n")
                print(f"Run time: {time.perf_counter() - start} s")
                time.sleep(2)

            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == "__main__":
    run()
