import rev
from magicbot import tunable, will_reset_to

class Hanger:
    
    climb_motor: rev.CANSparkMax

    climb_speed = tunable(0.5)

    climb_target_speed = will_reset_to(0.0)

    def __init__(self) -> None:
        # check if motor needs to be inverted irl
        self.climb_motor.setInverted(False)

    def on_disable(self) -> None:
        self.climb_motor.stopMotor()

    def execute(self) -> None:
        self.climb_motor.set(self.climb_speed)

    def winch(self) -> None:
        self.climb_target_speed = self.climb_speed

    def payout(self) -> None:
        self.climb_target_speed = -self.climb_speed
