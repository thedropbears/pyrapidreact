import magicbot
import time
import ctre
import wpilib
from enum import Enum


class IntakeState(Enum):
    clearing = 1
    intaking = 2
    stopped = 3


class Intake:
    intake_speed = magicbot.tunable(1)
    clearing_time = magicbot.tunable(1)

    intake_prox: wpilib.DigitalInput
    intake_motor: ctre.TalonSRX

    def setup(self):
        self.state = IntakeState.intaking
        self.clearing_since = time.monotonic()

        self.intake_motor.setInverted(True)

    def execute(self) -> None:
        if self.state == IntakeState.clearing:
            if time.monotonic() - self.clearing_since > self.clearing_time:
                self.state = IntakeState.intaking
            self.intake_motor.set(ctre.ControlMode.PercentOutput, -self.intake_speed)

        elif self.state == IntakeState.intaking:
            # lower intake
            self.intake_motor.set(ctre.ControlMode.PercentOutput, self.intake_speed)
            if self.has_ball():
                self.state = IntakeState.stopped

        elif self.state == IntakeState.stopped:
            # raise intake?
            self.intake_motor.set(ctre.ControlMode.PercentOutput, 0)

    def toggle_intaking(self):
        if self.state == IntakeState.stopped:
            self.state = IntakeState.intaking
        else:
            self.state = IntakeState.stopped
        # actuate intake up/down

    def clear(self):
        self.state = IntakeState.clearing
        self.clearing_since = time.monotonic()

    def stop_clearing(self):
        self.state = IntakeState.stopped

    @magicbot.feedback
    def has_ball(self) -> bool:
        return not self.intake_prox.get()
