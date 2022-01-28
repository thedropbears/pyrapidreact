import magicbot
import rev
import time
import ctre
from enum import Enum


class IndexerState(Enum):
    indexing = 1
    stopped = 2
    reading = 3
    clearing = 4
    firing = 5


class Indexer:

    is_red = magicbot.tunable(False)

    indexer_speed = magicbot.tunable(0.8)
    feeder_speed = magicbot.tunable(1)

    clearing_time = magicbot.tunable(1)
    firing_time = magicbot.tunable(0.5)

    colour_sensor: rev.ColorSensorV3
    indexer_motor: ctre.TalonSRX
    feed_motor: ctre.TalonSRX

    reading_time = magicbot.tunable(0.5)

    def setup(self):
        self.state = IndexerState.indexing
        self.firing_since = time.monotonic()
        self.clearing_since = time.monotonic()
        self.reading_since = time.monotonic()
        self.read_colour = 0
        self.read_total = 0

        self.indexer_motor.setInverted(False)
        self.feed_motor.setInverted(False)

    def execute(self) -> None:
        if self.state == IndexerState.clearing:
            if time.monotonic() - self.clearing_since > self.clearing_time:
                self.state = IndexerState.indexing
            self.feed_motor.set(ctre.ControlMode.PercentOutput, -self.feeder_speed)
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, -self.indexer_speed)

        if self.state == IndexerState.indexing:
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
            self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)
            if self.has_ball():
                self.state = IndexerState.reading
                self.reading_since = time.monotonic()

        if self.state == IndexerState.stopped:
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, 0)
            self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)

        if self.state == IndexerState.reading:
            if time.monotonic() - self.reading_since > self.reading_time:
                if self.read_colour / self.read_total > 0.5:  # ball is ours
                    self.state = IndexerState.stopped
                else:
                    self.clear()
                self.read_colour = 0
                self.read_total = 0
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, 0)
            self.feed_motor.set(ctre.ControlMode.PercentOutput, 0)
            self.read_colour += self.is_ball_ours()
            self.read_total += 1

        if self.state == IndexerState.firing:
            self.indexer_motor.set(ctre.ControlMode.PercentOutput, self.indexer_speed)
            self.feed_motor.set(ctre.ControlMode.PercentOutput, self.feeder_speed)
            if time.monotonic() - self.firing_since > self.firing_time:
                self.state = IndexerState.stopped

    def fire(self) -> None:
        self.firing_since = time.monotonic()
        self.state = IndexerState.firing

    def clear(self):
        self.state = IndexerState.clearing
        self.clearing_since = time.monotonic()

    def stop(self):
        self.state = IndexerState.stopped

    def start(self):
        self.state = IndexerState.indexing

    @magicbot.feedback
    def has_ball(self) -> bool:
        return self.colour_sensor.getProximity() > 400

    @magicbot.feedback
    def is_ball_ours(self) -> bool:
        values = self.colour_sensor.getRawColor()
        return (values.red > values.blue) == self.is_red

    @magicbot.feedback
    def colour_values(self):
        values = self.colour_sensor.getRawColor()
        return f"R: {values.red}, G: {values.green}, B: {values.green}"

    @magicbot.feedback
    def colour_prox(self):
        return str(self.colour_sensor.getProximity())

    @magicbot.feedback
    def is_ready(self) -> bool:
        return self.has_ball() and self.is_ball_ours()

    def has_read(self):
        # return time.monotonic() - self.reading_since > self.reading_time
        return self.state == IndexerState.stopped
