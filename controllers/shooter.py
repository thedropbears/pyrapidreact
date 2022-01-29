from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
import magicbot
from numpy import interp


class ShooterController:
    indexer: Indexer
    intake: Intake
    shooter: Shooter

    ignore_colour = magicbot.tunable(False)

    fire_command = magicbot.will_reset_to(False)
    clear_command = magicbot.will_reset_to(False)

    def __init__(self):
        self.intaking = False

    # If set to true, flywheel speed is set from tunable
    # Otherwise it is calculated from the interpolation table
    interpolation_override = magicbot.tunable(True)
    flywheel_speed = magicbot.tunable(0.0)

    dist = magicbot.tunable(2.0)
    ranges_xp = (2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0)
    flywheel_speed_fp = (45.0, 50.0, 52.5, 57.0, 60.0, 65.0, 70.0)

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

    def toggle_intaking(self):
        self.intaking = not self.intaking

        if self.interpolation_override:
            self.shooter.motor_speed = self.flywheel_speed
        else:
            self.shooter.motor_speed = interp(self.dist, self.ranges_xp, self.flywheel_speed_fp)

    def fire_input(self):
        self.fire_command = True

    def clear_input(self):
        self.clear_command = True
