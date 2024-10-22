from typing import DefaultDict, Dict, List
from pynput import keyboard
import collections

from config import motor_config


class RCVelocityGenerator:
    """"""

    def __init__(self, velocity: float, motor_configs: List[motor_config.MotorConfig]) -> None:
        self._velocity_default = velocity
        self._motor_configs = motor_configs
        self._pressed_keys: DefaultDict[keyboard.Key, bool] = collections.defaultdict(lambda: False)

    def update(self, key: keyboard.Key, *, pressed: bool) -> None:
        self._pressed_keys[key] = pressed

    def velocities(self) -> Dict[motor_config.MotorConfig, float]:
        return {
            motor: self._calc_motor_velocity(motor) for motor in self._motor_configs
        }

    def _calc_motor_velocity(self, motor: motor_config.MotorConfig) -> float:
        linear_vel = self._linear_velocity()
        angular_vel = self._angular_velocity()

        if motor.location in (motor_config.MotorLocation.FRONT_LEFT, motor_config.MotorLocation.REAR_LEFT):
            return (linear_vel - angular_vel) * motor.direction
        elif motor.location in (motor_config.MotorLocation.FRONT_RIGHT, motor_config.MotorLocation.REAR_RIGHT):
            return (linear_vel + angular_vel) * motor.direction
        else:
            raise ValueError(f"Unknown motor config location {motor=}")

    def _angular_velocity(self) -> float:
        """Angular velocity assuming that the velocity rotates about the robots body axis which would be with 
        the x-axis pointed forward and the z-axis pointed skyward.
        """
        vel = 0.0
        if self._right_key_pressed:
            vel -= self._velocity_default
        # Left turn is a positive angular velocity.
        if self._left_key_pressed:
            vel += self._velocity_default
        
        return vel

    def _linear_velocity(self) -> float:
        vel = 0.0
        if self._up_key_pressed:
            vel += self._velocity_default
        if self._down_key_pressed:
            vel -= self._velocity_default
        
        return vel


    @property
    def _up_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.up]
    
    @property
    def _down_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.down]
        
    @property
    def _left_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.left]

    @property
    def _right_key_pressed(self) -> bool:
        return self._pressed_keys[keyboard.Key.right]
