from components.shooter import Shooter
from components.target_estimator import TargetEstimator
from components.turret import Turret
import magicbot
from numpy import interp


class ShooterController:
    shooter: Shooter
    target_estimator: TargetEstimator
    turret: Turret

    # If set to true, flywheel speed is set from tunable
    # Otherwise it is calculated from the interpolation table
    interpolation_override = magicbot.tunable(True)
    flywheel_speed = magicbot.tunable(0.0)

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

    @magicbot.feedback
    def distance_to_goal(self) -> float:
        return self.distance
