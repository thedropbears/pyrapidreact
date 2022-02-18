import magicbot
import rev
import wpilib


class Intake(magicbot.StateMachine):
    intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax
    intake_piston: wpilib.Solenoid

    intake_speed = tunable(0.5)
    speed = 0.0

    def setup(self) -> None:
        self.intake_motor.setInverted(False)

    def execute(self) -> None:
        self.intake_motor.set(self.speed)

    def set(self, direction: int) -> None:
        if direction == 1:
            self.speed = direction * self.intake_speed
            self.intake_piston.set(True)
        else: self.intake_piston.set(False)
