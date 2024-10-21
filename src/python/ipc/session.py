from typing import List
import pathlib

import system_info
from config import motor_config

_MOTOR_CONFIG_PATH = system_info.get_root_project_directory() / "env" / "motor_configs"


def get_motor_config(robot_name: str, location: motor_config.MotorLocation) -> motor_config.MotorConfig:
    """Get a single motor config for the robot."""
    file_path = _MOTOR_CONFIG_PATH / robot_name / f"{location.value.lower()}.json"
    return motor_config.MotorConfig.from_json(file_path)


def get_robot_motor_configs(robot_name: str) -> List[motor_config.MotorConfig]:
    """Get all motor configs for this robot."""
    return [
        get_motor_config(robot_name, location) for location in motor_config.MotorLocation
    ]
