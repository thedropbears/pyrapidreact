import wpilib
import time
from enum import Enum


class LedColours(Enum):
    RED = (255, 0, 0)
    ORANGE = (255, 160, 0)
    PINK = (255, 0, 150)
    BLUE = (0, 0, 255)
    GREEN = (0, 255, 0)
    OFF = (0, 0, 0)


class StatusLights:
    leds: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 280  # TODO: check length
        self.sync_time = 10000

        self.is_flashing = False

        self.flash_timer = time.time()
        self.FLASH_DELAY = 0.4

        self.is_pulsing = False

        self.pulse_multiplier = 1
        self.pulse_increasing = False
        self.MAX_PULSE = 1
        self.MIN_PULSE = 0
        self.PULSE_CHANGE = 0.02

        self.colour = (0,0,0)

    def mult_tuple(self, arg1: tuple[int,int,int], arg2: float):
        return (int(arg1[0] * arg2), int(arg1[1] * arg2), int(arg1[2] * arg2))

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData(255, 0, 150)
        self.leds.setSyncTime(self.sync_time)
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def pulse(self, colour=None):
        if not colour == None:
            self.colour = colour
        self.is_pulsing = True
        #self.pulse_multiplier = 1
        self.is_flashing = False

    def flash(self, colour=None):
        if not colour == None:
            self.colour = colour
        self.is_flashing = True
        self.flash_timer = time.time()
        self.is_pulsing = False

    def solid(self, colour=None):
        if not colour == None:
            self.colour = colour
        self.is_flashing = False
    
    def stop_pulse(self):
        self.is_pulsing = False

    def pulse_calculation(self, colour):
        if self.pulse_multiplier >= self.MAX_PULSE:
            self.pulse_increasing = False
        elif self.pulse_multiplier <= self.MIN_PULSE:
            self.pulse_increasing = True

        self.pulse_multiplier += self.PULSE_CHANGE * (
            1 if self.pulse_increasing else -1
        )

        return self.mult_tuple(colour, self.pulse_multiplier)

    def flash_calculation(self, colour):
        if int(((time.time() - self.flash_timer) / self.FLASH_DELAY)) % 2:
            return colour
        else:
            return LedColours.OFF.value

    def execute(self) -> None:
        if self.is_flashing:
            colour = self.flash_calculation(self.colour)
        elif self.is_pulsing:
            colour = self.pulse_calculation(self.colour)
        else:
            colour = self.colour
        # colour = self.pulse_calculation(self.colour)
        self.single_led_data.setRGB(colour[0], colour[1], colour[2])
        self.leds.setData(self.leds_data)
