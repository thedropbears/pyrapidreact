import math
from magicbot.state_machine import StateMachine, state
from components.hanger import Hanger
from components.turret import Turret
from components.intake import Intake
from controllers.shooter import ShooterController


class ClimbController(StateMachine):
    """High level hang controller"""

    shooter_control: ShooterController
    turret: Turret
    intake: Intake
    hanger: Hanger

    @state(first=True, must_finish=True)
    def prepare(self, state_tm):
        self.shooter_control.track_target = False
        self.turret.slew_local(math.pi)
        self.intake.auto_retract = False
        self.intake.motor_enabled = False
        self.intake.deployed = True
        if state_tm > 5 and self.turret.is_on_target():
            self.next_state("raise_arms")

    @state(must_finish=True)
    def raise_arms(self):
        self.hanger.payout(1)
        if self.hanger.is_raised():
            self.done()