import asyncio
import os
import sys
import time
from typing import Type

import serial  # type: ignore[import-untyped]
from ublox_gps import UbloxGps  # type: ignore[import-untyped]

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import system_info
from drivers.gps import messages

_WAIT_TIME = 60


class GPSMessageReader:
    def __init__(self) -> None:
        self._port = serial.Serial(system_info.UBLOX_SERIAL, baudrate=38400, timeout=1)
        self._ublox_gps = UbloxGps(self._port)

    async def listen(self) -> None:
        print("Listening for UBX Messages.")
        await asyncio.gather(
            self._listen_for_cov_message(), self._listen_for_pvt_message()
        )

    async def _listen_for_pvt_message(self) -> None:
        while True:
            start = time.perf_counter()
            msg = await self._poll_for_message(messages.UbloxPVTMessage)
            print(f"Run time: {time.perf_counter() - start} s")
            print(msg)

    async def _listen_for_cov_message(self) -> None:
        while True:
            start = time.perf_counter()
            msg = await self._poll_for_message(messages.UbloxCOVMessage)
            print(f"Run time: {time.perf_counter() - start} s")
            print(msg)

    async def _poll_for_message(
        self, msg: Type[messages.UbloxBaseMessage]
    ) -> messages.UbloxBaseMessage:
        self._ublox_gps.set_packet(msg.cls_name, msg.msg_name, None)
        self._ublox_gps.send_message(msg.cls_name, msg.msg_name, None)

        start = time.perf_counter()
        while msg.msg_name not in self._ublox_gps.packets[msg.cls_name]:
            await asyncio.sleep(0.05)

            if time.perf_counter() - start > _WAIT_TIME:
                break

        ublox_msg = self._ublox_gps.packets[msg.cls_name].get(msg.msg_name, None)
        return msg.from_ublox_message(ublox_msg)

    def close(self) -> None:
        self._port.close()


if __name__ == "__main__":
    reader = GPSMessageReader()
    try:
        asyncio.run(reader.listen())
    except KeyboardInterrupt:
        pass
    finally:
        reader.close()
