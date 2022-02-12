import rev
from magicbot import tunable, will_reset_to

class Hanger:
    
    climb_motor: rev.CANSparkMax

    climb_position = tunable(0.1)

    GEAR_RATIO = 1 / 15.0

    def setup(self) -> None:
        # check if motor needs to be inverted irl
        self.climb_motor.setInverted(False)

        self.motor_pid = self.climb_motor.getPIDController()

        self.motor_pid.setP(1.0)
        self.motor_pid.setI(0.0)
        self.motor_pid.setD(0.0)
        self.motor_pid.setFF(0.0)

        self.encoder = self.climb_motor.getEncoder()
        self.encoder.setPositionConversionFactor(self.GEAR_RATIO)

    def on_disable(self) -> None:
        self.climb_motor.stopMotor()

    def execute(self) -> None:
        self.motor_pid.setReference(self.climb_position, rev.CANSparkMax.ControlType.kPosition)

    def winch(self) -> None:
        self.climb_position += 0.1

    def payout(self) -> None:
        self.climb_position -= 0.1
