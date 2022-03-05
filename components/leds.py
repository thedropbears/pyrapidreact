from logging import raiseExceptions
import wpilib
import time
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
        self.led_length = 72  # TODO: check length

        self.is_flashing = False

        self.flash_on = True
        self.FLASH_DELAY = 0.3
        self.flash_timer = time.time()

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
    
    def start_pulse(self):
        self.is_pulsing = True
        self.pulse_multiplier = 1

    def stop_pulse(self):
        self.is_pulsing = False

    def start_flash(self):
        self.is_flashing = True
        self.flash_timer = time.time()
    
    def stop_flash(self):
        self.is_flashing = False
    
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
            if self.flash_timer + self.FLASH_DELAY >= time.time():
                self.flash_timer = time.time()
                self.flash_on = not self.flash_on
            return self.mult_tuple(colour, (1 if self.flash_on else 0))
        else:
            return colour
            
    def set(self, state: LedStates) -> None:
        colour = state.value
        colour = self.flash_calc(colour)
        colour = self.pulse_calc(colour)
        print(f"set leds {colour[0]} {colour[1]} {colour[2]}")
        self.led_data = [
            wpilib.AddressableLED.LEDData(colour[0], colour[1], colour[2])
        ] * self.led_length

    def execute(self) -> None:
        self.leds.setData(self.leds_data)
