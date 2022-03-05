import wpilib
import time
from enum import Enum

class LedStates(Enum):
    RED = (255, 0, 0)
    ORANGE = (255, 160, 0)
    PINK = (255, 0, 150)
    BLUE = (0,0,255)
    GREEN = (0, 255, 0)

class StatusLights:
    leds: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 72  # TODO: check length

        self.is_flashing = False

        self.flash_timer = time.time()
        self.FLASH_DELAY = 0.3

        self.is_pulsing = False

        self.pulse_multiplier = 1
        self.pulse_increasing = False
        self.MAX_PULSE = 1
        self.MIN_PULSE = 0.1
        self.PULSE_CHANGE = 0.03

    def mult_tuple(self, arg1: tuple, arg2: float):
        return (arg1[0] * arg2, arg1[1] * arg2, arg1[2] * arg2)

    def setup(self) -> None:
        self.leds.setLength(self.led_length)
        self.leds_data = [wpilib.AddressableLED.LEDData(0, 0, 0)] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()
    
    def pulse(self):
        self.is_pulsing = True
        self.pulse_multiplier = 1
        self.is_flashing = False

    def flash(self):
        self.is_flashing = True
        self.flash_timer = time.time()
        self.is_pulsing = False

    def solid(self):
        self.is_flashing = False
        self.is_pulsing = False
    
    def pulse_calc(self, colour):
        if self.is_pulsing:
            if self.pulse_multiplier >= self.MAX_PULSE:
                self.pulse_increasing = False
            elif self.pulse_multiplier <= self.MIN_PULSE:
                self.pulse_increasing = True
            
            self.pulse_multiplier += self.PULSE_CHANGE * (1 if self.pulse_increasing else -1)
        
            return self.mult_tuple(colour, self.pulse_multiplier)
        else:
            return colour
    
    def flash_calc(self, colour):
        if self.is_flashing:
            if ((time.time()-self.flash_timer)/self.FLASH_DELAY)%2:
                return self.mult_tuple(colour, 1)
            else:
                return self.mult_tuple(colour, 0)
        else:
            return colour
            
    def set_colour(self, state: tuple) -> None:
        colour = state.value
        colour = self.flash_calc(colour)
        colour = self.pulse_calc(colour)
        print(f"set leds {colour[0]} {colour[1]} {colour[2]}")
        self.led_data = [
            wpilib.AddressableLED.LEDData(colour[0], colour[1], colour[2])
        ] * self.led_length

    def execute(self) -> None:
        self.leds.setData(self.leds_data)
