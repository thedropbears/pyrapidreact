import rev
import wpilib
from magicbot import feedback


class Intake:
    # intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax
    intake_piston: wpilib.DoubleSolenoid

    auto_retract = True

    def __init__(self):
        self._last_cargo_presence = False
        self.deployed = False
        self.motor_enabled = True

    def setup(self) -> None:
        self.intake_motor.restoreFactoryDefaults()
        self.intake_motor.setInverted(True)
        self._intake_limit = self.intake_motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self._intake_limit.enableLimitSwitch(False)

    def execute(self) -> None:
        if self.has_cargo() and self.auto_retract:
            # If the breakbeam has fired we have a ball and we should retract
            self.deployed = False

        if self.deployed:
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kReverse)

        if self.deployed and self.motor_enabled:
            self.intake_motor.set(1.0)
        else:
            self.intake_motor.set(0.0)

    def has_cargo(self) -> bool:
        current = self._intake_limit.get()
        result = current and self._last_cargo_presence
        self._last_cargo_presence = current
        return result

    def deploy_without_running(self) -> None:
        self.auto_retract = False
        self.deployed = True
        self.motor_enabled = False

    @feedback
    def is_deployed(self):
        return self.deployed

    @feedback
    def is_motor_enabled(self):
        return self.motor_enabled
