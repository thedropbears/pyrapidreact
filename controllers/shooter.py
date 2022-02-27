from components.indexer import Indexer
import math
from components.shooter import Shooter
from components.turret import Turret
from magicbot import (
    StateMachine,
    tunable,
    default_state,
    timed_state,
    feedback,
    will_reset_to,
)
from components.chassis import Chassis
from numpy import interp


class ShooterController(StateMachine):
    shooter: Shooter
    turret: Turret
    indexer: Indexer
    chassis: Chassis

    # If set to true, flywheel speed is set from tunable
    # Otherwise it is calculated from the interpolation table
    interpolation_override = tunable(False)
    flywheel_speed = tunable(0.0)

    distance = 0.0
    ranges_lookup = (3.0, 4.0, 5.0, 6.0, 7.0, 8.0)
    flywheel_speed_lookup = (28.0, 30.0, 35.0, 39.0, 43.5, 48.0)

    MAX_DIST = 8
    MIN_DIST = 3

    MAX_SPEED = 0.1
    MAX_ROTATION = 0.1

    _wants_to_fire = will_reset_to(False)

    @default_state
    def tracking(self) -> None:
        # calculate angle and dist to target
        turret_pose = self.chassis.robot_to_world(self.shooter.turret_offset)
        field_angle = math.atan2(-turret_pose.Y(), -turret_pose.X())
        cur_pose = self.chassis.estimator.getEstimatedPosition()
        angle = field_angle - cur_pose.rotation().radians()
        self.distance = cur_pose.translation().norm()

        self.turret.slew_local(angle)

        if self.interpolation_override:
            self.shooter.motor_speed = self.flywheel_speed
        else:
            self.shooter.motor_speed = interp(
                self.distance, self.ranges_lookup, self.flywheel_speed_lookup
            )

        if (
            self._wants_to_fire
            and self.indexer.has_cargo_in_chimney()
            and self.shooter.is_at_speed()
            and self.turret.is_on_target()
            and self.distance > self.MIN_DIST
            and self.distance < self.MAX_DIST
            and self.chassis.translation_velocity < self.MAX_SPEED
            and self.chassis.rotation_velocity < self.MAX_ROTATION
        ):
            self.next_state("firing")

    @timed_state(duration=0.5, first=True, next_state="tracking", must_finish=True)
    def firing(self) -> None:
        self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)

    @feedback
    def distance_to_goal(self) -> float:
        return self.distance

    def fire(self) -> None:
        self._wants_to_fire = True
