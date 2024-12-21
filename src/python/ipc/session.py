import json
from typing import Any, Dict, List, Optional

import system_info
from config import robot_config

_MOTOR_CONFIG_PATH = system_info.get_root_project_directory() / "env" / "motor_configs"
_FLAT_ENDPOINT_PATH = (
    system_info.get_root_project_directory() / "env/motor_configs/flat_endpoints.json"
)


def get_motor(
    robot_name: str, location: robot_config.DrivetrainLocation
) -> robot_config.Motor:
    """Get a single motor for the robot."""
    file_path = _MOTOR_CONFIG_PATH / robot_name / f"{location.value.lower()}.json"
    return robot_config.Motor.from_json(file_path)


def get_robot_config() -> robot_config.Beachbot:
    """The robots config. Eventually configurable by different beachbots but for now its
    only the one.
    """
    wheel = robot_config.Wheel()
    return robot_config.Beachbot(
        drivetrain=[
            robot_config.Drivetrain(location=location, wheel=wheel)
            for location in robot_config.DrivetrainLocation
        ]
    )


def get_robot_motors(*, robot_name: Optional[str] = None) -> List[robot_config.Motor]:
    """Get all motor for this robot."""
    robot_name = robot_name or get_robot_name()
    return [
        get_motor(robot_name, location) for location in robot_config.DrivetrainLocation
    ]


def get_robot_name() -> str:
    """The configured robot."""
    # TODO: Make this change based on which robot is running.
    return "beachbot-1"


def get_motor_endpoint_data() -> Dict[str, Any]:
    with open(_FLAT_ENDPOINT_PATH, "r") as fp:
        endpoint_data = json.load(fp)

    return endpoint_data


def get_motor_config(motor: robot_config.Motor) -> Dict[str, Any]:
    file_path = (
        _MOTOR_CONFIG_PATH / get_robot_name() / f"{motor.location.value.lower()}.json"
    )

    with open(file_path, "r") as f:
        motor_config_dict = json.load(f)

    return motor_config_dict
