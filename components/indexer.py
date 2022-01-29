import magicbot
import rev
import ctre

rev.ColorSensorV3.RawColor.__add__ = lambda a, b: rev.ColorSensorV3.RawColor(
    a.red + b.red, a.green + b.green, a.blue + b.blue, a.ir + b.ir
)


class Indexer(magicbot.StateMachine):

    is_red = magicbot.tunable(False)

    indexer_speed = magicbot.tunable(0.8)
    feeder_speed = magicbot.tunable(1)

    colour_sensor: rev.ColorSensorV3
    indexer_motor: ctre.TalonSRX
    feed_motor: ctre.TalonSRX

    def setup(self):
        self.read_colour = rev.ColorSensorV3.RawColor(0, 0, 0, 0)

        self.indexer_motor.setInverted(False)
        self.feed_motor.setInverted(False)

    @magicbot.state(first=True)
    def indexing(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
        self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)
        if self.has_ball():
            self.next_state("reading")

    @magicbot.default_state
    def stopped(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, 0)
        self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)
        if self.has_ball() and not self.is_ball_ours():
            self.next_state("clearing")

    @magicbot.timed_state(duration=1, next_state="stopped", must_finish=True)
    def clearing(self):
        self.feed_motor.set(ctre.ControlMode.PercentOutput, -self.feeder_speed)
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, -self.indexer_speed)

    @magicbot.timed_state(duration=0.5, next_state="stopped", must_finish=True)
    def firing(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
        self.feed_motor.set(ctre.ControlMode.PercentOutput, self.feeder_speed)

    def fire(self):
        self.next_state("firing")

    def clear(self):
        self.next_state("clearing")

    def stop(self):
        self.next_state("stopped")

    def start(self):
        self.next_state("indexing")

    @magicbot.feedback
    def has_ball(self) -> bool:
        return self.colour_sensor.getProximity() > 400

    @magicbot.feedback
    def can_read_ball(self) -> bool:
        # or if can see red or blue
        return self.colour_sensor.getProximity() > 500

    @magicbot.feedback
    def is_ball_ours(self) -> bool:
        """Assumes we have a ball we can read"""
        return (self.read_colour.red > self.read_colour.blue) == self.is_red

    @magicbot.feedback
    def colour_values(self):
        values = self.colour_sensor.getRawColor()
        return f"R: {values.red}, G: {values.green}, B: {values.green}"

    @magicbot.feedback
    def colour_prox(self):
        return self.colour_sensor.getProximity()
