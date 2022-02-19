import rev
import wpilib
from magicbot import tunable, feedback


class Indexer:
    indexer_front_motor: rev.CANSparkMax
    indexer_mid_motor: rev.CANSparkMax
    indexer_back_motor: rev.CANSparkMax
    colour_sensor: rev.ColorSensorV3
    breakbeam_sensor: wpilib.DigitalInput
    indexer_piston: wpilib.Solenoid

    is_firing = tunable(False)
    is_red = tunable(False)
    indexer_speed = tunable(0.5)
    # Front, mid, back
    speeds = (0.0, 0.0, 0.0)

    def setup(self) -> None:
        self.indexer_front_motor.setInverted(False)
        self.indexer_mid_motor.setInverted(False)
        self.indexer_back_motor.setInverted(False)

    def execute(self) -> None:
        self.indexer_front_motor.set(self.speeds[0])
        self.indexer_mid_motor.set(self.speeds[1])
        self.indexer_back_motor.set(self.speeds[2])

    @feedback
    def has_back(self) -> bool:
        return self.breakbeam_sensor.get()

    def has_front(self) -> bool:
        return self.colour_sensor.getProximity() > 400

    def is_front_ours(self) -> bool:
        raw = self.colour_sensor.getRawColor()
        return (raw.red > raw.blue) == self.is_red

    def set(self, front: int, mid: int, back: int) -> None:
        self.speeds = tuple(self.indexer_speed * s for s in (front, mid, back))

    def set_piston(self, on: bool) -> None:
        self.indexer_piston.set(on)
