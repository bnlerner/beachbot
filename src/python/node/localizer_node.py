import os
import sys

# Get the path to the root of the project
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from ipc import messages, registry
from localization import localizer

from node import base_node

# TODO: Increase once GPS is supplimented with sensor fusion from the motors.
_PUB_FREQUENCY = 20


class LocalizerNode(base_node.BaseNode):
    """Handles localization of the robot by inputing relevant sensor data and outputing
    the body kinematics.
    """

    def __init__(self) -> None:
        super().__init__(registry.NodeIDs.LOCALIZER)
        self._localizer = localizer.Localizer()

        self.add_subscribers(
            {
                registry.Channels.MOTOR_VELOCITY_FRONT_LEFT: self._update_motor_velocity,
                registry.Channels.MOTOR_VELOCITY_FRONT_RIGHT: self._update_motor_velocity,
                registry.Channels.MOTOR_VELOCITY_REAR_LEFT: self._update_motor_velocity,
                registry.Channels.MOTOR_VELOCITY_REAR_RIGHT: self._update_motor_velocity,
                registry.Channels.IMU: self._update_imu,
                registry.Channels.GNSS: self._update_gnss,
            }
        )
        self.add_publishers(registry.Channels.BODY_KINEMATICS)
        self.add_looped_tasks({self._publish_body_kinematics: _PUB_FREQUENCY})

    async def _publish_body_kinematics(self) -> None:
        if veh_msg := self._localizer.vehicle_kin_msg():
            self.publish(registry.Channels.BODY_KINEMATICS, veh_msg)

    def _update_imu(self, msg: messages.IMUMessage) -> None:
        self._localizer.input_imu_msg(msg)

    def _update_gnss(self, msg: messages.GNSSMessage) -> None:
        self._localizer.input_gnss_msg(msg)

    def _update_motor_velocity(self, msg: messages.MotorVelocityMessage) -> None:
        self._localizer.input_motor_vel_msg(msg)


if __name__ == "__main__":
    node = LocalizerNode()
    node.start()
