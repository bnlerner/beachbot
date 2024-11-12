from config import motor_config


class NavVelocityGenerator:
    """Generates a navigation velocity from an input target forward velocity and spin."""

    def __init__(self) -> None:
        self._linear_velocity: float = 0.0
        self._angular_velocity: float = 0.0

    def update(self, linear_velocity: float, angular_velocity: float) -> None:
        self._linear_velocity = linear_velocity
        self._angular_velocity = angular_velocity

    def velocity(self, motor: motor_config.MotorConfig) -> float:
        if motor.location in (
            motor_config.MotorLocation.FRONT_LEFT,
            motor_config.MotorLocation.REAR_LEFT,
        ):
            return (self._linear_velocity - self._angular_velocity) * motor.direction
        elif motor.location in (
            motor_config.MotorLocation.FRONT_RIGHT,
            motor_config.MotorLocation.REAR_RIGHT,
        ):
            return (self._linear_velocity + self._angular_velocity) * motor.direction
        else:
            raise ValueError(f"Unknown motor config location {motor=}")
