import magicbot
import rev
import ctre


class Indexer(magicbot.StateMachine):

    is_red = magicbot.tunable(False)

    indexer_speed = magicbot.tunable(0.8)

    colour_sensor: rev.ColorSensorV3
    indexer_motor: ctre.TalonSRX

    def setup(self):
        self.indexer_motor.setInverted(False)

    @magicbot.state(first=True)
    def indexing(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
        if self.has_ball():
            self.done()

    @magicbot.default_state
    def stopped(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, 0)

    @magicbot.timed_state(duration=1, must_finish=True)
    def clearing(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, -self.indexer_speed)

    @magicbot.timed_state(duration=0.5, must_finish=True)
    def firing(self):
        self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)

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
