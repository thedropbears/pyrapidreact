import magicbot
import ctre
import wpilib


class Intake(magicbot.StateMachine):
    intake_speed = magicbot.tunable(0.9)

    intake_prox: wpilib.DigitalInput
    intake_motor: ctre.TalonSRX

    def setup(self):
        self.intake_motor.setInverted(True)

    @magicbot.state(first=True, must_finish=True)
    def intaking(self):
        # lower intake
        self.intake_motor.set(ctre.ControlMode.PercentOutput, self.intake_speed)
        if self.has_ball():
            self.next_state("stopped")

    @magicbot.state(must_finish=True)
    def stopped(self):
        # raise intake
        self.intake_motor.set(ctre.ControlMode.PercentOutput, 0)

    @magicbot.timed_state(duration=1, next_state="stopped", must_finish=True)
    def clearing(self):
        self.intake_motor.set(ctre.ControlMode.PercentOutput, -self.intake_speed)

    def toggle_intaking(self):
        if self.current_state == "intaking":
            self.next_state("stopped")
        else:
            self.next_state("intaking")

    def clear(self):
        self.next_state("clearing")

    def stop(self):
        self.next_state("stopped")

    @magicbot.feedback
    def has_ball(self) -> bool:
        return not self.intake_prox.get()
