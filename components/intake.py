import rev
import wpilib


class Intake:
    # intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax
    intake_piston: wpilib.DoubleSolenoid

    auto_retract = True

    def __init__(self) -> None:
        self._last_cargo_presence = False
        self._deployed = False
        self._running = False

    @property
    def deployed(self):
        return self._deployed

    @deployed.setter
    def deployed(self, value):
        self._deployed = value
        self._running = value

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
            self._deployed = False
            self._running = False

        if self._deployed:
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kReverse)

        if self._running:
            self.intake_motor.set(1.0)
        else:
            self.intake_motor.set(0.0)

    def has_cargo(self) -> bool:
        current = self._intake_limit.get()
        result = current and self._last_cargo_presence
        self._last_cargo_presence = current
        return result
