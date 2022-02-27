import wpilib
import time

class LEDScreen:
    led: wpilib.AddressableLED

    def __init__(self):
        self.led_length = 72
        self.led_bottom = wpilib.AddressableLED.LEDData(175, 0, 0)
        self.led_middle = wpilib.AddressableLED.LEDData(0, 175, 0)
        self.led_top = wpilib.AddressableLED.LEDData(0, 0, 175)

        self.flash_on = True
        self.flash_delay = 0.5 #seconds
        self.flash_timer = time.time()

    def setup(self) -> None:
        self.led.setLength(self.led_length)
        self.led_rows = (
            [self.led_bottom] * int(self.led_length / 3)
            + [self.led_middle] * int(self.led_length / 3)
            + [self.led_top] * int(self.led_length / 3)
        )
        self.led.setData(self.led_rows)
        self.led.start()

    def set_bottom_row(self, r, g, b) -> None:
        self.led_bottom.setRGB(r, g, b)

    def set_middle_row(self, r, g, b) -> None:
        self.led_middle.setRGB(r, g, b)

    def set_top_row(self, r, g, b) -> None:
        self.led_top.setRGB(r, g, b)

    def execute(self) -> None:
        self.led.setData(self.led_rows)
    
    def update_lights(self, status):
        colour = (0,0,0)

        if status == 0:
            colour = (0,255,0)
        elif status == 1:
            colour = (255,0,0)
        elif status == 2:
            if self.flash_on:
                colour = (252, 94, 3)
            else:
                colour = (0,0,0)

            if time.time() >= self.flash_timer + self.flash_delay:
                self.flash_on = not self.flash_on
                self.flash_timer = time.time()
        elif status == 3:
            colour = (252, 94, 3)
        
        self.set_top_row(colour[0],colour[1],colour[2])
        self.set_top_row(colour[0],colour[1],colour[2])
        self.set_top_row(colour[0],colour[1],colour[2])