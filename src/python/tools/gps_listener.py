import os
import sys
import time

import serial  # type: ignore[import-untyped]
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import system_info
from drivers.gps import messages

# Can also use SPI here - import spidev, but we dont because USB is easier to setup.
port = serial.Serial(system_info.UBLOX_SERIAL, baudrate=38400, timeout=1)
gps = UbloxGps(port)


def run() -> None:
    try:
        print("Listening for UBX Messages.")
        while True:
            start = time.perf_counter()
            # Position, Velocity and Time message.
            if coords := gps.geo_coords():
                pvt_msg = messages.UbloxPVTMessage.from_ublox_message(coords)
                print(f"{pvt_msg=}\n")
            # if veh_att := gps.veh_attitude():
            #     att_msg = messages.UbloxATTMessage.from_ublox_message(veh_att)
            #     print(f"{att_msg=}\n")
            # if veh_dyn := gps.vehicle_dynamics():
            #     print(f"{veh_dyn=}")
            #     ins_msg = messages.UbloxINSMessage.from_ublox_message(veh_dyn)
            #     print(f"{ins_msg=}\n")
            # Status of the sensors being used for sensor fusion.
            # if status := gps.esf_status():
            #     status_msg = messages.UbloxESFStatusMessage.from_ublox_message(
            #         status
            #     )
            #     print(f"{status_msg=}\n")

            # if esf_measures := gps.esf_measures():
            #     print(f"{esf_measures=}")
            # Status of the RF antenna.
            # if rf_status := gps.rf_ant_status():
            #     print(f"{rf_status=}\n")
            print(f"Run time: {time.perf_counter() - start} s")
            # time.sleep(1)
            # data = gps.wait_packet('ESF', 'INS', 2500)
            # print(data)

    finally:
        port.close()


if __name__ == "__main__":
    run()
