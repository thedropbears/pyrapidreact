import ctre
from utilities.ctre import FALCON_CPR
from magicbot import tunable


class Hanger:

    climb_motor: ctre.TalonFX

    GEAR_RATIO = 1 / 35.0
    PULLEY_CIRCUMFERANCE = 0.04

    winch_dir = 0
    winch_power = tunable(0)

    def setup(self) -> None:
        # TODO: check
        self.climb_motor.setInverted(ctre.TalonFXInvertType.CounterClockwise)

        self.climb_motor.config_kF(0, 0.0, 10)
        self.climb_motor.config_kP(0, 2.0, 10)
        self.climb_motor.config_kI(0, 0.0, 10)
        self.climb_motor.config_kD(0, 0.0, 10)
        self.climb_motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )
        self.climb_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.climb_motor.configSelectedFeedbackCoefficient(
            self.GEAR_RATIO * self.PULLEY_CIRCUMFERANCE / FALCON_CPR, 0, 10
        )

    def on_disable(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Disabled, 0)

    def execute(self) -> None:
        self.climb_motor.set(
            ctre.ControlMode.PercentOutput, self.winch_dir * self.winch_power
        )
        self.winch_dir = 0

    def winch(self) -> None:
        self.winch_dir = -1

    def payout(self) -> None:
        self.winch_dir = 1

    # def release(self) -> None:
    #     self.climb_position = 1.8
