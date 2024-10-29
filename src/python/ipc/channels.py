"""A channel contains all information to send a message through from a
publisher to a subscriber.
"""
from types import SimpleNamespace

from ipc import core, messages


class Channels(SimpleNamespace):
    BODY_GPS = core.ChannelSpec[messages.GPSMessage](
        channel="body_gps", msg_class=messages.GPSMessage
    )
    BODY_DYNAMICS = core.ChannelSpec[messages.VehicleDynamicsMessage](
        channel="vehicle_dynamics", msg_class=messages.VehicleDynamicsMessage
    )
    FRONT_LEFT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="front_left_motor_command", msg_class=messages.MotorCommandMessage
    )
    FRONT_RIGHT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="front_right_motor_command", msg_class=messages.MotorCommandMessage
    )
    REAR_LEFT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="rear_left_motor_command", msg_class=messages.MotorCommandMessage
    )
    REAR_RIGHT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="rear_right_motor_command", msg_class=messages.MotorCommandMessage
    )
