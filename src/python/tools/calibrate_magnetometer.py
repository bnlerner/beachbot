"""
Hard Iron Offset Correction:
Find the center of the distorted sphere by computing the mean of each axis:

Subtract these offsets from the raw data.
Soft Iron Scale Correction:
Compute the scaling factor for each axis:

"""
import time

import adafruit_bno08x  # type: ignore[import-untyped]
import board  # type: ignore[import-untyped]
import busio  # type: ignore[import-untyped]
import numpy as np
from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore[import-untyped]


def main() -> None:
    i2c = busio.I2C(board.SCL, board.SDA)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)

    data = []
    try:
        while True:
            mag_x, mag_y, mag_z = bno.magnetic
            data.append([mag_x, mag_y, mag_z])
            time.sleep(0.5)
    except KeyboardInterrupt:
        mag_data = np.array(data)
        offset_x = (mag_data[:, 0].max() + mag_data[:, 0].min()) / 2
        offset_y = (mag_data[:, 1].max() + mag_data[:, 1].min()) / 2
        offset_z = (mag_data[:, 2].max() + mag_data[:, 2].min()) / 2

        scale_x = (mag_data[:, 0].max() - mag_data[:, 0].min()) / 2
        scale_y = (mag_data[:, 1].max() - mag_data[:, 1].min()) / 2
        scale_z = (mag_data[:, 2].max() - mag_data[:, 2].min()) / 2
        print(
            f"{offset_x=}, {offset_y=}, {offset_z=}, {scale_x=}, {scale_y=}, {scale_z=}"
        )

    finally:
        i2c.unlock()


if __name__ == "__main__":
    main()
