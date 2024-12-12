"""A registry contains all information to send a message through from a
publisher to a subscriber.
"""
from types import SimpleNamespace

from config import robot_config

from ipc import core, messages


class Channels(SimpleNamespace):
    """A registry of all IPC channels available to use."""

    BODY_KINEMATICS = core.ChannelSpec[messages.VehicleKinematicsMessage](
        channel="vehicle_dynamics"
    )
    IMU = core.ChannelSpec[messages.IMUMessage](channel="imu")
    GNSS = core.ChannelSpec[messages.GNSSMessage](channel="gnss")
    MOTOR_CMD_FRONT_LEFT = core.ChannelSpec[messages.MotorCommandMessage](
        channel="front_left_motor_command"
    )
    MOTOR_CMD_FRONT_RIGHT = core.ChannelSpec[messages.MotorCommandMessage](
        channel="front_right_motor_command"
    )
    MOTOR_CMD_REAR_LEFT = core.ChannelSpec[messages.MotorCommandMessage](
        channel="rear_left_motor_command"
    )
    MOTOR_CMD_REAR_RIGHT = core.ChannelSpec[messages.MotorCommandMessage](
        channel="rear_right_motor_command"
    )
    MOTOR_VELOCITY_FRONT_LEFT = core.ChannelSpec[messages.MotorVelocityMessage](
        channel="front_left_motor_velocity"
    )
    MOTOR_VELOCITY_FRONT_RIGHT = core.ChannelSpec[messages.MotorVelocityMessage](
        channel="front_right_motor_velocity"
    )
    MOTOR_VELOCITY_REAR_LEFT = core.ChannelSpec[messages.MotorVelocityMessage](
        channel="rear_left_motor_velocity"
    )
    MOTOR_VELOCITY_REAR_RIGHT = core.ChannelSpec[messages.MotorVelocityMessage](
        channel="rear_right_motor_velocity"
    )
    STOP_MOTORS = core.ChannelSpec[messages.StopMotorsMessage](channel="stop_motors")


class Requests(SimpleNamespace):
    NAVIGATE = core.RequestSpec("navigate", messages.NavigateRequest)


class NodeIDs(SimpleNamespace):
    """A registry of Node IDs available to use."""

    GNSS = core.NodeID(name="gnss")
    IMU = core.NodeID(name="imu")
    LOCALIZER = core.NodeID(name="localizer")
    MOTOR_CONTROL = core.NodeID(name="motor_control")
    NAVIGATION = core.NodeID(name="navigation")
    RC = core.NodeID(name="rc")
    UI = core.NodeID(name="ui")


def motor_command_channel(motor: robot_config.Motor) -> core.ChannelSpec:
    """Motor command channel from the config."""
    if motor.location == robot_config.DrivetrainLocation.FRONT_LEFT:
        return Channels.MOTOR_CMD_FRONT_LEFT
    elif motor.location == robot_config.DrivetrainLocation.FRONT_RIGHT:
        return Channels.MOTOR_CMD_FRONT_RIGHT
    elif motor.location == robot_config.DrivetrainLocation.REAR_LEFT:
        return Channels.MOTOR_CMD_REAR_LEFT
    elif motor.location == robot_config.DrivetrainLocation.REAR_RIGHT:
        return Channels.MOTOR_CMD_REAR_RIGHT
    else:
        raise ValueError("unknown channel")


def motor_velocity_channel(motor: robot_config.Motor) -> core.ChannelSpec:
    """Motor velocity channel from the config."""
    if motor.location == robot_config.DrivetrainLocation.FRONT_LEFT:
        return Channels.MOTOR_VELOCITY_FRONT_LEFT
    elif motor.location == robot_config.DrivetrainLocation.FRONT_RIGHT:
        return Channels.MOTOR_VELOCITY_FRONT_RIGHT
    elif motor.location == robot_config.DrivetrainLocation.REAR_LEFT:
        return Channels.MOTOR_VELOCITY_REAR_LEFT
    elif motor.location == robot_config.DrivetrainLocation.REAR_RIGHT:
        return Channels.MOTOR_VELOCITY_REAR_RIGHT
    else:
        raise ValueError("unknown channel")
