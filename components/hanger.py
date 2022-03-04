import ctre
from magicbot import tunable


class Hanger:

    climb_motor: ctre.TalonFX

    climb_position = tunable(0)

    GEAR_RATIO = 1 / 30.0

    def setup(self) -> None:
        # check if motor needs to be inverted irl
        self.climb_motor.setInverted(False)

        self.climb_motor.config_kF(0, 0.0, 10)
        self.climb_motor.config_kP(0, 2.0, 10)
        self.climb_motor.config_kI(0, 0.0, 10)
        self.climb_motor.config_kD(0, 0.0, 10)
        self.climb_motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )
        self.climb_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.climb_motor.configSelectedFeedbackCoefficient(self.GEAR_RATIO, 0, 10)

    def on_disable(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Disabled, 0)

    def execute(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Position, self.climb_position)

    def winch(self) -> None:
        self.climb_position += 100

    def payout(self) -> None:
        self.climb_position -= 100

    def release(self) -> None:
        # TODO put however climb releases its arms
        pass
