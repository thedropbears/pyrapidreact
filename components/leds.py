import wpilib
from enum import Enum


class LedStates(Enum):
    NO_BALL = (255, 0, 0)
    # turret wrong or flywheel wrong
    TARGETING = (255, 160, 0)
    # too close or too far
    RANGE = (0, 0, 255)
    SPEED = (255, 0, 150)
    READY = (0, 255, 0)


class StatusLights:
    leds: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 43  # TODO: check length
        self.single_data = wpilib.AddressableLED.LEDData(255, 0, 0)
        self.leds_data = [self.single_data] * self.led_length

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set(self, state: LedStates) -> None:
        self.single_data.setRGB(state.value[0], state.value[1], state.value[2])

    def execute(self) -> None:
        self.leds.setData(self.leds_data)
