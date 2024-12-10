import geometry
from config import robot_config
from planning import nav_vel_target_generator, primitives

from controls import nav_velocity_controller


class NavCascadeController:
    """A cascade controller to handle navigating along the path."""

    def __init__(self, config: robot_config.Beachbot):
        self._robot_config = config
        self._target_generator = nav_vel_target_generator.NavVelTargetGenerator(config)
        self._controller = nav_velocity_controller.NavVelocityController(config)

    def update(
        self,
        navpoint: primitives.NavigationPoint,
        cur_pose: geometry.Pose,
        reference_speed: float,
        cur_twist: geometry.Twist,
    ) -> None:
        # Resets the controller if we reach a cusp so that we dont carry the integrated
        # control signal when switching directions.
        if navpoint.is_cusp_point:
            self._controller.reset()

        target_in_body = cur_pose.to_local(navpoint.pose_2d)
        target_in_body.update_frame(geometry.BODY)
        target_twist = self._target_generator.gen_twist(
            navpoint.signed_turn_radius, reference_speed, target_in_body
        )
        self._controller.update(target_twist, cur_twist)

    def velocity(self, motor: robot_config.Motor) -> float:
        """Motor velocity in turns/s to achieve the control inputs."""
        return self._controller.velocity(motor)
