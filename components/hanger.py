import ctre
from utilities.ctre import FALCON_CPR
from magicbot import tunable, feedback


class Hanger:

    climb_motor: ctre.TalonFX

    GEAR_RATIO = 1 / 35.0
    PULLEY_CIRCUMFERENCE = 0.04

    winch_speed = tunable(0.2)  # max speed m/s
    target_position = tunable(0)  # positive releases rope, negative pulls

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
        self.climb_motor.setSelectedSensorPosition(0)
        self.climb_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.climb_motor.configSelectedFeedbackCoefficient(
            self.GEAR_RATIO * self.PULLEY_CIRCUMFERENCE / FALCON_CPR, 0, 10
        )

    def on_disable(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Disabled, 0)

    def execute(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Position, self.climb_position)

    def winch(self, speed: float) -> None:
        self.target_position -= speed * self.winch_speed / 50

    def payout(self, speed: float) -> None:
        self.target_position += speed * self.winch_speed / 50

    @feedback
    def get_position(self) -> float:
        return self.climb_motor.getSelectedSensorPosition()
