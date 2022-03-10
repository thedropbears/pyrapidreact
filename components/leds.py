import wpilib
import time
from enum import Enum
from typing import Optional, Tuple


MAX_BRIGHTNESS = 200  # Between 0-255 of Value on HSV scale

class LedColours(Enum):
    # Use HSV to get nicer fading
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (30, 255, MAX_BRIGHTNESS)
    PINK = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)


class StatusLights:
    leds: wpilib.AddressableLED

    def __init__(self) -> None:
        self.led_length = 262

        self.is_flashing = False

        self.flash_timer = time.monotonic()
        self.FLASH_DELAY = 0.4

        self.is_pulsing = False

        self.pulse_multiplier = 1.0
        self.pulse_increasing = False
        self.MAX_PULSE = 1.0
        self.MIN_PULSE = 0.1
        self.PULSE_CHANGE = 0.01

        self.colour = (0, 0, 0)

    def fade(self, brightness: float) -> Tuple[int, int, int]:
        return (self.colour[0], self.colour[1], round(self.colour[2] * brightness))

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def pulse(self, colour: Optional[LedColours] = None) -> None:
        if colour is not None:
            self.colour = colour.value
        self.is_pulsing = True
        self.is_flashing = False

    def flash(self, colour: Optional[LedColours] = None) -> None:
        if colour is not None:
            self.colour = colour.value
        self.is_flashing = True
        self.flash_timer = time.monotonic()
        self.is_pulsing = False

    def solid(self, colour: Optional[LedColours] = None) -> None:
        if colour is not None:
            self.colour = colour.value
        self.is_flashing = False
        self.is_pulsing = False

    def pulse_calculation(self) -> Tuple[int, int, int]:
        if self.pulse_multiplier >= self.MAX_PULSE:
            self.pulse_multiplier = self.MAX_PULSE
            self.pulse_increasing = False
        elif self.pulse_multiplier <= self.MIN_PULSE:
            self.pulse_multiplier = self.MIN_PULSE
            self.pulse_increasing = True

        self.pulse_multiplier += self.PULSE_CHANGE * (
            1 if self.pulse_increasing else -1
        )

        return self.fade(self.pulse_multiplier)

    def flash_calculation(self) -> Tuple[int, int, int]:
        if int((time.monotonic() - self.flash_timer) / self.FLASH_DELAY) % 2:
            return self.colour
        else:
            return LedColours.OFF.value

    def execute(self) -> None:
        if self.is_flashing:
            colour = self.flash_calculation()
        elif self.is_pulsing:
            colour = self.pulse_calculation()
        else:
            colour = self.colour
        self.single_led_data.setHSV(colour[0], colour[1], colour[2])
        self.leds.setData(self.leds_data)
