"""A registry contains all information to send a message through from a
publisher to a subscriber.
"""
from types import SimpleNamespace

from ipc import core, messages


class Channels(SimpleNamespace):
    """A registry of all channels available to use."""

    BODY_GPS = core.ChannelSpec[messages.GPSMessage](channel="body_gps")
    BODY_DYNAMICS = core.ChannelSpec[messages.VehicleDynamicsMessage](
        channel="vehicle_dynamics"
    )
    FRONT_LEFT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="front_left_motor_command"
    )
    FRONT_RIGHT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="front_right_motor_command"
    )
    REAR_LEFT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="rear_left_motor_command"
    )
    REAR_RIGHT_MOTOR_CMD = core.ChannelSpec[messages.MotorCommandMessage](
        channel="rear_right_motor_command"
    )


class NodeIDs(SimpleNamespace):
    """A registry of Node IDs available to use."""

    MOTOR_CONTROL = core.NodeID(name="motor_control")
    RC = core.NodeID(name="rc")
    UBLOX_DATA = core.NodeID(name="ublox_data")
    UI = core.NodeID(name="ui_node")
