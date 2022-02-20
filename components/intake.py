import magicbot
import rev


class Intake:
    # intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax
    # intake_piston: wpilib.Solenoid

    speed_mult = magicbot.tunable(1)
    speed = magicbot.will_reset_to(0.0)

    def setup(self) -> None:
        self.intake_motor.restoreFactoryDefaults()
        self.intake_motor.setInverted(True)

    def execute(self) -> None:
        self.intake_motor.set(self.speed)

    def set(self, direction: int) -> None:
        self.speed = direction * self.speed_mult
        # if direction == 1:
        #     self.intake_piston.set(True)
        # else:
        #     self.intake_piston.set(False)
