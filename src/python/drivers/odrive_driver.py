import odrive
import json


class MotorDriver:

    def __init__(self) -> None:
        odrive.connected_devices
