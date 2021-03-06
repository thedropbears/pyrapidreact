import rev
import wpilib
from magicbot import tunable, will_reset_to
from enum import Enum
from components.indexer import Indexer

import magicbot


class Intake:
    indexer: Indexer

    class Direction(Enum):
        FORWARDS = 1
        BACKWARDS = -1

    # intake_prox: wpilib.DigitalInput
    intake_motor: rev.CANSparkMax
    intake_piston: wpilib.DoubleSolenoid

    auto_retract = tunable(True)
    invert_direction = tunable(False)
    motor_direction = will_reset_to(Direction.FORWARDS)

    def __init__(self):
        self._last_cargo_presence = 0
        self.deployed = False
        self.motor_enabled = True

    def setup(self) -> None:
        self.intake_motor.setInverted(False)
        self._intake_limit = self.intake_motor.getForwardLimitSwitch(
            rev.SparkMaxLimitSwitch.Type.kNormallyOpen
        )
        self._intake_limit.enableLimitSwitch(False)

    def execute(self) -> None:
        # stop intake motor running if we have something in tunnel
        self.motor_enabled = self.indexer.ready_to_intake()

        if self.has_cargo() and self.auto_retract:
            # If the breakbeam has fired we have a ball and we should retract
            self.deployed = False

        if self.deployed:
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.intake_piston.set(wpilib.DoubleSolenoid.Value.kReverse)

        if self.deployed:
            if self.motor_direction is self.Direction.BACKWARDS:
                self.intake_motor.set(-0.4 if self.invert_direction else 0.4)
            elif self.motor_enabled:
                self.intake_motor.set(1.0 if self.invert_direction else -1.0)
        else:
            self.intake_motor.set(0.0)

    @magicbot.feedback
    def has_cargo(self) -> bool:
        if self._intake_limit.get():
            self._last_cargo_presence += 1
        else:
            self._last_cargo_presence = 0
        return self._last_cargo_presence > 3

    def deploy_without_running(self) -> None:
        self.auto_retract = False
        self.deployed = True
        self.motor_enabled = False

    def is_deployed(self):
        return self.deployed

    def is_motor_enabled(self):
        return self.motor_enabled
