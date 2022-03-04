from magicbot.state_machine import StateMachine, state, timed_state
from components.hanger import Hanger
from components.turret import Turret
from components.intake import Intake
from controllers.shooter import ShooterController


class ClimbController(StateMachine):
    """High level hang controller"""

    shooter_control: ShooterController
    turret: Turret
    intake: Intake
    hang: Hanger

    @state(first=True, must_finish=True)
    def prepare(self, state_tm):
        self.shooter_control.track_target = False
        self.turret.slew_local(0)
        self.intake.auto_retract = False
        self.intake._deployed = True
        self.intake._running = False
        if state_tm > 1 and self.turret.get_error() < 0.2:
            self.next_state("")

    @timed_state(duration=1, must_finish=True)
    def raise_arms(self):
        self.hang.release()
