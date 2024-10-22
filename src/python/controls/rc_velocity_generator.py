from typing import DefaultDict, Dict, List
from pynput import keyboard  # type: ignore[import-untyped]
import collections

from config import motor_config


class RCVelocityGenerator:

    def __init__(self, motor_configs: List[motor_config.MotorConfig]) -> None:
        self._motor_configs = motor_configs
        self._pressed_keys: DefaultDict[keyboard.Key, bool] = collections.defaultdict(lambda: False)

    def update(self, key: keyboard.Key, *, pressed: bool) -> None:
        self._pressed_keys[key] = pressed

    def velocities(self) -> Dict[motor_config.MotorConfig, float]:
        return {
            motor: self._gen_motor_velocity(motor) for motor in self._motor_configs
        }

    def _gen_motor_velocity(self, motor: motor_config.MotorConfig) -> float:
        linear_vel = self._linear_velocity()
        angular_vel = self._angular_velocity()

        if motor.location in (motor_config.MotorLocation.FRONT_LEFT, motor_config.MotorLocation.REAR_LEFT):
            return (linear_vel - angular_vel) * motor.direction
        elif motor.location in (motor_config.MotorLocation.FRONT_RIGHT, motor_config.MotorLocation.REAR_RIGHT):
            return (linear_vel + angular_vel) * motor.direction
        else:
            raise ValueError(f"Unknown motor config location {motor=}")

    def _angular_velocity(self) -> float:
        vel = 0
        if self._right_key_pressed:
            vel += 1
        if self._left_key_pressed:
            vel -= 1
        
        return vel

    def _linear_velocity(self) -> float:
        vel = 0
        if self._up_key_pressed:
            vel += 1
        if self._down_key_pressed:
            vel -= 1
        
        return vel


    @property
    def _up_key_pressed(self) -> keyboard.Key:
        return self._pressed_keys[keyboard.Key.up]
    
    @property
    def _down_key_pressed(self) -> keyboard.Key:
        return self._pressed_keys[keyboard.Key.down]
        
    @property
    def _left_key_pressed(self) -> keyboard.Key:
        return self._pressed_keys[keyboard.Key.left]

    @property
    def _right_key_pressed(self) -> keyboard.Key:
        return self._pressed_keys[keyboard.Key.right]
