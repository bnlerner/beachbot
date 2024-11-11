"""A registry contains all information to send a message through from a
publisher to a subscriber.
"""
from types import SimpleNamespace

from config import robot_config
from drivers import primitives

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


class Requests(SimpleNamespace):
    NAVIGATE = core.RequestSpec("navigate", messages.NavigateRequest)


class NodeIDs(SimpleNamespace):
    """A registry of Node IDs available to use."""

    MOTOR_CONTROL = core.NodeID(name="motor_control")
    NAVIGATION = core.NodeID(name="navigation")
    RC = core.NodeID(name="rc")
    UBLOX_DATA = core.NodeID(name="ublox_data")


def motor_channel(motor: primitives.Motor) -> core.ChannelSpec:
    """Motor channel from the config."""
    if motor.location == robot_config.DrivetrainLocation.FRONT_LEFT:
        return Channels.FRONT_LEFT_MOTOR_CMD
    elif motor.location == robot_config.DrivetrainLocation.FRONT_RIGHT:
        return Channels.FRONT_RIGHT_MOTOR_CMD
    elif motor.location == robot_config.DrivetrainLocation.REAR_LEFT:
        return Channels.REAR_LEFT_MOTOR_CMD
    elif motor.location == robot_config.DrivetrainLocation.REAR_RIGHT:
        return Channels.REAR_RIGHT_MOTOR_CMD
    else:
        raise ValueError("unknown channel")
