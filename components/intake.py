import magicbot
import wpilib
import rev
import time

class Indexer:
    intakeSpeed = magicbot.tunable(0.8)
    indexerSpeed = magicbot.tunable(0.5)

    clearingTime = magicbot.tunable(1) # second
    fireTime = magicbot.tunable(0.5)
    is_red = magicbot.tunable(False)

    prox_limit = magicbot.tunable(300)

    colour_sensor: rev.ColorSensorV3

    def __init__(self) -> None:
        self.clearing = False
        self.clearingSince = time.monotonic() # when you began to clear

        self.intaking = True # if we are running intake and indexer
        self.firing = False # if we are sending a ball to be shot
        self.firingSince = time.monotonic() # when you began to fire

        self.colorValues = self.colour_sensor.getColors()

    def execute(self) -> None:
        if self.clearing:
            if time.monotonic() - self.clearingSince > self.clearingTime:
                self.clearing = False
            # run intake/indexer backwards
            return

        if self.intaking:
            # run intake/indexer forward
            if self.get_ball():
                self.intaking = False
        else:
            if self.has_ball() and not self.is_ball_ours():
                self.clearing = True
                self.clearingSince = time.monotonic()
        
        if self.firing:
            # run everything forward
            if time.monotonic() - self.firingSince > self.fireTime:
                self.firing = False
                self.intaking = True

    def fire(self) -> None:
        self.intaking = True
        self.firing = True
            
    @magicbot.feedback
    def has_ball(self) -> bool:
        return self.colour_sensor.proximity() > self.prox_limit

    @magicbot.feedback
    def is_ball_ours(self) -> bool:
        values = self.colour_sensor.getColors()
        return (values.red > values.blue) == self.is_red

    @maigcbot.feedback
    def is_ready(self) -> bool:
        return self.has_ball() and self.is_ball_ours()