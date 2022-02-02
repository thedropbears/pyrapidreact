import magicbot
import rev
import wpilib


class Intake(magicbot.StateMachine):
    intake_speed = magicbot.tunable(0.9)

    intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax

    def setup(self):
        self.intake_motor.setInverted(False)

    @magicbot.state(first=True)
    def intaking(self):
        # lower intake
        self.intake_motor.set(self.intake_speed)
        if self.has_ball():
            self.next_state("stopped")

    @magicbot.default_state
    def stopped(self):
        # raise intake
        self.intake_motor.set(0)

    @magicbot.timed_state(duration=1, must_finish=True)
    def clearing(self):
        self.intake_motor.set(-self.intake_speed)

    @magicbot.feedback
    def has_ball(self) -> bool:
        return not self.intake_prox.get()