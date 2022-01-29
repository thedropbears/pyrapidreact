from components.indexer import Indexer
from components.intake import Intake
import magicbot


class ShooterController:
    intake: Intake
    indexer: Indexer
    ignore_colour = magicbot.tunable(False)

    fire_command = magicbot.will_reset_to(False)

    def execute(self):
        if (
            not self.ignore_colour
            and self.indexer.has_ball()
            and self.indexer.has_read()
            and not self.indexer.is_ball_ours()
        ):
            self.indexer.clear()
            self.intake.clear()

        if (
            self.fire_command
            and self.indexer.has_ball()
            and (
                (self.indexer.has_read() and self.indexer.is_ball_ours())
                or self.ignore_colour
            )
        ):
            self.indexer.fire()

    # def toggle(self):
    #     self.indexer.

    def fire_input(self):
        self.fire_command = True
