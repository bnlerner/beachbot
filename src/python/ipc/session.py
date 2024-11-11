from typing import List, Optional

import system_info
from config import robot_config
from drivers import primitives

_MOTOR_CONFIG_PATH = system_info.get_root_project_directory() / "env" / "motor_configs"


def get_motor(
    robot_name: str, location: robot_config.DrivetrainLocation
) -> primitives.Motor:
    """Get a single motor for the robot."""
    file_path = _MOTOR_CONFIG_PATH / robot_name / f"{location.value.lower()}.json"
    return primitives.Motor.from_json(file_path)


def get_robot_motors(*, robot_name: Optional[str] = None) -> List[primitives.Motor]:
    """Get all motor for this robot."""
    robot_name = robot_name or _get_robot_name()
    return [
        get_motor(robot_name, location) for location in robot_config.DrivetrainLocation
    ]


def _get_robot_name() -> str:
    """The configured robot."""
    # TODO: Make this change based on which robot is running.
    return "beachbot-1"
