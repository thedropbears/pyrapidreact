import rev
import wpilib


class Intake:
    # intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax
    intake_piston: wpilib.DoubleSolenoid

    deployed = False

    def setup(self) -> None:
        self.intake_motor.setInverted(True)
        self._intake_limit = self.intake_motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )

    def execute(self) -> None:
        if self.has_cargo():
            # If the breakbeam has fired we have a ball and we should retract
            self.deployed = False
        if self.deployed:
            self.intake_motor.set(1.0)
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.intake_motor.set(0.0)
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kReverse)

    def has_cargo(self) -> bool:
        return self._intake_limit.get()
