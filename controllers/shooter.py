from components.indexer import Indexer
import math
from components.shooter import Shooter
from components.turret import Turret
from magicbot import StateMachine, tunable, default_state, timed_state, feedback
from components.chassis import Chassis
from magicbot import tunable, feedback
from wpimath.geometry import Translation2d
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
    ranges_lookup = (2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)
    flywheel_speed_lookup = (32.0, 30.0, 36.0, 39.0, 42.0, 46.0, 51.0, 56.0)

    turret_offset = Translation2d(-0.148, 0)  # From CAD
    _wants_to_fire = False

    @default_state
    def tracking(self) -> None:
        # calculate angle and dist to target
        turret_pose = self.chassis.robot_to_world(self.turret_offset)
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
        ):
            self.next_state("firing")
            # Reset each loop so that the call has to be made each control loop
            self._wants_to_fire = False

    @timed_state(duration=0.5, first=True, next_state="tracking", must_finish=True)
    def firing(self) -> None:
        self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)

    @feedback
    def distance_to_goal(self) -> float:
        return self.distance

    def fire(self) -> None:
        self._wants_to_fire = True
