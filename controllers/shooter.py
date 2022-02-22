from components.shooter import Shooter
from components.target_estimator import TargetEstimator
from components.turret import Turret
from controllers.indexer import IndexerController
from magicbot import tunable, feedback, will_reset_to
from numpy import interp


class ShooterController:
    indexer_control: IndexerController
    shooter: Shooter
    target_estimator: TargetEstimator
    turret: Turret

    # If set to true, flywheel speed is set from tunable
    # Otherwise it is calculated from the interpolation table
    interpolation_override = tunable(False)
    flywheel_speed = tunable(0.0)

    wants_to_fire = will_reset_to(False)

    allowed_flywheel_error = tunable(20.0)
    allowed_bearing_error = tunable(0.2)

    distance = 0.0
    ranges_lookup = (2.5, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0)
    flywheel_speed_lookup = (32.0, 30.0, 36.0, 39.0, 42.0, 46.0, 51.0, 56.0)

    def execute(self):
        angle, self.distance = self.target_estimator.to_target()
        if angle is not None:
            self.turret.slew_local(angle)

        if self.interpolation_override:
            self.shooter.motor_speed = self.flywheel_speed
        else:
            self.shooter.motor_speed = interp(
                self.distance, self.ranges_lookup, self.flywheel_speed_lookup
            )

        if (
            self.wants_to_fire
            and abs(self.shooter.flywheel_error()) < self.allowed_flywheel_error
            and abs(self.turret.get_error()) < self.allowed_bearing_error
        ):
            self.indexer_control.wants_to_fire = True

    @feedback
    def distance_to_goal(self) -> float:
        return self.distance
