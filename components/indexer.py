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
            self.done()

    @magicbot.default_state
    def stopped(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, 0)
        self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)

    @magicbot.timed_state(duration=1, must_finish=True)
    def clearing(self):
        self.feed_motor.set(ctre.ControlMode.PercentOutput, -self.feeder_speed)
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, -self.indexer_speed)

    @magicbot.timed_state(duration=0.5, must_finish=True)
    def firing(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
        self.feed_motor.set(ctre.ControlMode.PercentOutput, self.feeder_speed)

    @magicbot.feedback
    def has_ball(self) -> bool:
        return self.colour_sensor.getProximity() > 400

    def can_read(self) -> bool:
        return self.colour_sensor.getProximity() > 500

    @magicbot.feedback
    def is_ball_ours(self) -> bool:
        """Assumes we have a ball we can read"""
        values = self.colour_sensor.getRawColor()
        return (values.red > values.blue) == self.is_red

    @magicbot.feedback
    def colour_values(self):
        values = self.colour_sensor.getRawColor()
        return f"R: {values.red}, G: {values.green}, B: {values.green}"

    @magicbot.feedback
    def colour_prox(self):
        return self.colour_sensor.getProximity()
