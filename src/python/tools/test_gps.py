import serial  # type: ignore[import-untyped]
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

# Can also use SPI here - import spidev, but we dont because USB is easier to setup.
port = serial.Serial("/dev/ttyACM0", baudrate=38400, timeout=1)
gps = UbloxGps(port)


def run() -> None:
    try:
        print("Listenting for UBX Messages.")
        while True:
            try:
                coords = gps.geo_coords()
                print("Latitude: ", coords.lat, " Longitude: ", coords.lon)
                veh = gps.veh_attitude()
                print("Roll: ", veh.roll)
                print("Pitch: ", veh.pitch)
                print("Heading: ", veh.heading)
                print("Roll Acceleration: ", veh.accRoll)
                print("Pitch Acceleration: ", veh.accPitch)
                print("Heading Acceleration: ", veh.accHeading)
                veh_dyn = gps.vehicle_dynamics()
                print(
                    f"{veh_dyn=},{veh_dyn.xAccel=}, {veh_dyn.xAngRate=}, {veh_dyn.yAccel=}"
                )
                print(f"{gps.imu_alignment()=}")

            except (ValueError, IOError) as err:
                print(err)

    finally:
        port.close()


if __name__ == "__main__":
    run()
