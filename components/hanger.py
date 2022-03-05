import ctre


class Hanger:

    climb_motor: ctre.TalonFX

    climb_position = 0

    GEAR_RATIO = 1 / 35.0
    PULLEY_CIRCUMFERANCE = 0.04

    def setup(self) -> None:
        # TODO: check
        self.climb_motor.setInverted(ctre.TalonFXInvertType.Clockwise)

        self.climb_motor.config_kF(0, 0.0, 10)
        self.climb_motor.config_kP(0, 2.0, 10)
        self.climb_motor.config_kI(0, 0.0, 10)
        self.climb_motor.config_kD(0, 0.0, 10)
        self.climb_motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )
        self.climb_motor.setNeutralMode(ctre.NeutralMode.Brake)
        self.climb_motor.configSelectedFeedbackCoefficient(
            2048 * self.GEAR_RATIO * self.PULLEY_CIRCUMFERANCE, 0, 10
        )

    def on_disable(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Disabled, 0)

    def execute(self) -> None:
        self.climb_motor.set(ctre.ControlMode.Position, self.climb_position)

    def winch(self) -> None:
        self.climb_position -= 0.1

    def payout(self) -> None:
        self.climb_position += 0.1

    def release(self) -> None:
        self.climb_position = 1.8
