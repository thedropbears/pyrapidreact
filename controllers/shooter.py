from components.indexer import Indexer
from components.intake import Intake
import magicbot


class ShooterController:
    intake: Intake
    indexer: Indexer
    ignore_colour = magicbot.tunable(False)

    fire_command = magicbot.will_reset_to(False)

    def execute(self) -> None:
        if (
            self.indexer.has_ball()
            and not self.indexer.is_ball_ours()
            and not self.ignore_colour
            and self.indexer.has_read()
        ):
            self.indexer.clear()
            self.intake.clear()

        if self.is_ready() and self.fire_command:
            self.indexer.fire()

    def fire_input(self):
        self.fire_command = True

    def is_ready(self):
        return self.indexer.has_ball() and (
            self.indexer.is_ball_ours() or self.ignore_colour
        )
