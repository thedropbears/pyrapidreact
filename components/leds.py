import wpilib
from enum import Enum


class LedStates(Enum):
    NO_BALL = (255, 0, 0)
    # turret wrong or flywheel wrong
    TARGETING = (255, 160, 0)
    # too close or too far
    RANGE = (255, 160, 0)
    SPEED = (255, 0, 150)
    READY = (0, 255, 0)


class StatusLights:
    leds: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 72
        self.led_bottom = wpilib.AddressableLED.LEDData(175, 0, 0)

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.leds_data = [wpilib.AddressableLED.LEDData(0, 0, 0)] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set(self, state: LedStates) -> None:
        colour = state.value
        print(f"set leds {colour[0]} {colour[1]} {colour[2]}")
        self.led_data = [
            wpilib.AddressableLED.LEDData(colour[0], colour[1], colour[2])
        ] * self.led_length

    def execute(self) -> None:
        self.leds.setData(self.leds_data)
