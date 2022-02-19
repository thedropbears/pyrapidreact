import rev
import wpilib
from magicbot import tunable, feedback


class Indexer:
    indexer_front_motor: rev.CANSparkMax
    indexer_back_motor: rev.CANSparkMax
    colour_sensor: rev.ColorSensorV3
    prox_sensor1: wpilib.DigitalInput
    prox_sensor2: wpilib.DigitalInput
    # indexer_piston: wpilib.Solenoid

    is_firing = tunable(False)
    is_red = tunable(False)
    indexer_speed = tunable(0.5)
    # Front, mid, back
    speeds = (0.0, 0.0)

    def setup(self) -> None:
        self.indexer_front_motor.setInverted(True)
        self.indexer_back_motor.setInverted(False)

    def execute(self) -> None:
        # print(self.speeds)
        self.indexer_front_motor.set(self.speeds[0])
        self.indexer_back_motor.set(self.speeds[1])

    @feedback
    def has_back(self) -> bool:
        return not self.prox_sensor1.get() or not self.prox_sensor2.get()

    @feedback
    def has_front(self) -> bool:
        return self.colour_sensor.getProximity() > 400

    @feedback
    def is_front_ours(self) -> bool:
        raw = self.colour_sensor.getRawColor()
        return (raw.red > raw.blue) == self.is_red

    @feedback
    def get_colours(self) -> str:
        raw = self.colour_sensor.getRawColor()
        return f"r{raw.red:.3f}, g{raw.green:.3f}, b{raw.blue:.3f}"

    @feedback
    def get_speeds(self) -> str:
        return f"{self.speeds[0]}, {self.speeds[1]}"

    def set(self, front: int, back: int) -> None:
        self.speeds = tuple(self.indexer_speed * s for s in (front, back))

    def set_piston(self, on: bool) -> None:
        # self.indexer_piston.set(on)
        pass
