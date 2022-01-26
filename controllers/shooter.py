from components.indexer import Indexer
from components.intake import Intake
from components.target_estimator import TargetEstimator
from components.turret import Turret
import magicbot


class ShooterController:
    intake: Intake
    indexer: Indexer
    target_estimator: TargetEstimator
    turret: Turret
    
    ignore_colour = magicbot.tunable(False)

    fire_command = magicbot.will_reset_to(False)
    clear_command = magicbot.will_reset_to(False)

    def __init__(self):
        self.intaking = False

    def execute(self):
        if self.intaking:
            self.indexer.engage()
            self.intake.engage()
            if self.indexer.has_ball():
                self.intaking = False

        if (
            not self.ignore_colour
            and self.indexer.has_ball()
            and self.indexer.can_read()
            and not self.indexer.is_ball_ours()
        ):
            self.indexer.engage("clearing")
            self.intake.engage("clearing")

        if self.clear_command:
            self.indexer.engage("clearing")
            self.intake.engage("clearing")

        if (
            self.fire_command
            and self.indexer.has_ball()
            and (
                (self.indexer.can_read() and self.indexer.is_ball_ours())
                or self.ignore_colour
            )
        ):
            self.indexer.engage("firing")
            self.intaking = True
    
        angle = self.target_estimator.to_target()
        if angle is not None:
            self.turret.slew_relative(angle)

    def toggle_intaking(self):
        self.intaking = not self.intaking

    def fire_input(self):
        self.fire_command = True

    def clear_input(self):
        self.clear_command = True
