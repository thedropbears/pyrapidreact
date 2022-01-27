import magicbot
import rev
import time
import ctre


class Intake:
    """The intake, indexing and feeding of the ball"""

    # TODO: check for reversed
    indexer_speed = magicbot.tunable(0.8)
    intake_speed = magicbot.tunable(1)
    feeder_speed = magicbot.tunable(1)

    prox_limit = magicbot.tunable(300)

    colour_sensor: rev.ColorSensorV3
    intake_motor: ctre.TalonSRX
    indexer_motor: ctre.TalonSRX
    feed_motor: ctre.TalonSRX

    def setup(self):
        self.intaking = True
        self.feeding = False
        self.firing = False
        self.firing_since = time.monotonic()
        self.clearing = False
        self.clearing_since = time.monotonic()

    def execute(self) -> None:
        if self.clearing:
            if time.monotonic() - self.clearingSince > self.clearingTime:
                self.clearing = False
            self.intake_motor.set(ctre.ControlMode.PercentOutput, -self.intake_speed)
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, -self.indexer_speed)
            return

        if self.intaking:
            # lower intake
            self.intake_motor.set(ctre.ControlMode.PercentOutput, self.intake_speed)
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
            if self.has_ball():
                self.intaking = False
        else:
            # raise intake?
            self.intake.set(ctre.ControlMode.PercentOutput, 0)
            self.indexer.set(ctre.ControlMode.PercentOutput, 0)
            if self.has_ball() and not self.is_ball_ours():
                self.clearing = True
                self.clearingSince = time.monotonic()

        if self.firing:
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
            self.feed_motor.set(ctre.ControlMode.PercentOutput, self.feeder_speed)
            if time.monotonic() - self.firingSince > self.fireTime:
                self.firing = False
                self.intaking = True
                self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)

    def fire(self) -> None:
        self.firing = True
        self.firing_since = time.monotonic()

    def start_intaking(self):
        self.intaking = True

    def stop_intaking(self):
        self.intaking = False

    def clear(self):
        self.clearing = True
        self.clearing_since = time.monotonic()

    def stop_clearing(self):
        self.clearing = False

    @magicbot.feedback
    def has_ball(self) -> bool:
        return self.colour_sensor.proximity() > self.prox_limit

    @magicbot.feedback
    def is_ball_ours(self) -> bool:
        values = self.colour_sensor.getColors()
        return (values.red > values.blue) == self.is_red

    @magicbot.feedback
    def is_ready(self) -> bool:
        return self.has_ball() and self.is_ball_ours()
