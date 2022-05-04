import math
import wpilib
import random
import time
from enum import Enum, auto
from typing import List, Optional, Tuple
from utilities.scalers import scale_value


MAX_BRIGHTNESS = 180  # Between 0-255 of Value on HSV scale


class LedColours(Enum):
    # Use HSV to get nicer fading, hues are 0-180 so half usual hue
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (20, 255, MAX_BRIGHTNESS)
    YELLOW = (30, 255, MAX_BRIGHTNESS)
    PINK = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (90, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    WHITE = (0, 0, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)


class DisplayType(Enum):
    MORSE = auto()
    PACMAN = auto()
    RAINBOW = auto()
    SOLID = auto()
    PULSE = auto()
    FLASH = auto()


# creates a list of LEDData's from a List of (hsv col, repetitions)
def make_pattern(data: List[Tuple[Tuple, int]]) -> List[wpilib.AddressableLED.LEDData]:
    pattern_data = []
    for colour, number in data:
        for _ in range(number):
            x = wpilib.AddressableLED.LEDData()
            x.setHSV(colour.value[0], colour.value[1], colour.value[2])
            pattern_data.append(x)
    return pattern_data


class StatusLights:
    leds: wpilib.AddressableLED

    FLASH_PERIOD = 0.4
    PULSE_PERIOD = 1
    PACMAN_PERIOD = 60
    RAINBOW_PERIOD = 15

    def __init__(self) -> None:
        self.led_length = 262

        self.pattern_start_time = time.monotonic()

        self.colour = (0, 0, 0)
        self.cur_pattern = DisplayType.SOLID

        self._morse_message = ""

    def setup(self) -> None:
        self.choose_morse_message()
        self.leds.setLength(self.led_length)
        self.single_led_data = wpilib.AddressableLED.LEDData()
        self.leds_data = [self.single_led_data] * self.led_length
        self.leds.setData(self.leds_data)
        self.leds.start()

    def set(self, pattern: DisplayType, colour: Optional[LedColours] = None) -> None:
        if colour is not None:
            self.colour = colour.value
        if self.cur_pattern is not pattern:
            self.pattern_start_time = time.monotonic()
        self.cur_pattern = pattern

    def set_disabled(self):
        if self.cur_pattern not in [
            DisplayType.MORSE,
            DisplayType.PACMAN,
            DisplayType.RAINBOW,
        ]:
            self.cur_pattern = DisplayType.MORSE
            self.choose_morse_message("GG EZ")
            self.pattern_start_time = time.monotonic()

    def _pulse_calculation(self) -> Tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.pattern_start_time
        brightness = math.cos(elapsed_time * math.pi) / 2 + 0.5
        return (self.colour[0], self.colour[1], round(self.colour[2] * brightness))

    def _flash_calculation(self) -> Tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.pattern_start_time
        brightness = math.cos(self.FLASH_PERIOD * elapsed_time / math.pi) / 2 + 1
        return (self.colour[0], self.colour[1], self.colour[2] * round(brightness))

    def _rainbow_calculation(self) -> Tuple[int, int, int]:
        elapsed_time = time.monotonic() - self.pattern_start_time
        loop_time = self.RAINBOW_PERIOD / 3
        hue = round(180 * (elapsed_time / loop_time % 1))
        if elapsed_time > self.RAINBOW_PERIOD:
            self.set(DisplayType.PACMAN)
        return (hue, 255, MAX_BRIGHTNESS)

    @property
    def _morse_length(self) -> int:
        # A dash is three times as long as a dot
        # A space between characters is three dots
        # A space between dots and dashes is one dot
        # A space between words is 7 dots
        return (
            self._morse_message.count(".")
            + 3 * self._morse_message.count("-")
            + 3 * self._morse_message.count(" ")
        )

    def _morse_calculation(self) -> Tuple[int, int, int]:
        # Work out how far through the message we are
        DOT_LENGTH = 0.15  # seconds
        total_time = self._morse_length * DOT_LENGTH
        elapsed_time = time.monotonic() - self.pattern_start_time
        if elapsed_time > total_time:
            self.set(DisplayType.RAINBOW)
        running_total = 0.0
        for token in self._morse_message:
            if token == ".":
                running_total += 1.0 * DOT_LENGTH
            if token == "-" or token == " ":
                running_total += 3.0 * DOT_LENGTH
            if running_total > elapsed_time:
                # This is the current character
                if token == " ":
                    return LedColours.OFF.value
                else:
                    return LedColours.BLUE.value
        # Default - should never be hit
        return LedColours.OFF.value

    def _pacman_calculation(self) -> None:
        elapsed_time = time.monotonic() - self.pattern_start_time
        # find the pattern of leds and its position
        if elapsed_time % self.PACMAN_PERIOD < self.PACMAN_PERIOD / 2:
            pattern = make_pattern(
                [
                    (LedColours.ORANGE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.PINK, 2),
                    (LedColours.OFF, 2),
                    (LedColours.CYAN, 2),
                    (LedColours.OFF, 2),
                    (LedColours.RED, 2),
                    (LedColours.OFF, 5),
                    (LedColours.YELLOW, 3),
                ]
            )
            pacman_position = scale_value(
                elapsed_time % self.PACMAN_PERIOD,
                0,
                self.PACMAN_PERIOD / 2,
                -len(pattern),
                self.led_length,
            )
        else:
            pattern = make_pattern(
                [
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 2),
                    (LedColours.BLUE, 2),
                    (LedColours.OFF, 5),
                    (LedColours.YELLOW, 3),
                ]
            )
            pacman_position = scale_value(
                elapsed_time % self.PACMAN_PERIOD,
                self.PACMAN_PERIOD / 2,
                self.PACMAN_PERIOD,
                self.led_length,
                -len(pattern),
            )
        pacman_position = round(pacman_position)
        # create a list of LEDData's with the pattern surrounded by off led's
        led_data = []
        if pacman_position > 0:
            led_data.extend(
                [wpilib.AddressableLED.LEDData(0, 0, 0)] * math.floor(pacman_position)
            )
            led_data.extend(pattern)
        else:
            led_data.extend(pattern[-pacman_position:-1])
        leds_left = self.led_length - len(led_data)
        if leds_left > 0:
            led_data.extend([wpilib.AddressableLED.LEDData(0, 0, 0)] * leds_left)
        self.leds.setData(led_data[: self.led_length])

        if elapsed_time > self.PACMAN_PERIOD:
            self.set(DisplayType.MORSE)
            self.choose_morse_message()

    def choose_morse_message(self, _message: str = None) -> None:
        # Choose a morse message at random, unless specific message requested
        # Only use lowercase and spaces
        MESSAGES = [
            "KILL ALL HUMANS",
            "MORSE CODE IS FOR NERDS",
            "HONEYBADGER DONT CARE",
            "GLHF",
        ]
        if _message is None:
            message = random.choice(MESSAGES)
        else:
            message = _message.upper()
        # Convert to dots and dashes
        MORSE_CODE_DICT = {
            "A": ".-",
            "B": "-...",
            "C": "-.-.",
            "D": "-..",
            "E": ".",
            "F": "..-.",
            "G": "--.",
            "H": "....",
            "I": "..",
            "J": ".---",
            "K": "-.-",
            "L": ".-..",
            "M": "--",
            "N": "-.",
            "O": "---",
            "P": ".--.",
            "Q": "--.-",
            "R": ".-.",
            "S": "...",
            "T": "-",
            "U": "..-",
            "V": "...-",
            "W": ".--",
            "X": "-..-",
            "Y": "-.--",
            "Z": "--..",
            "1": ".----",
            "2": "..---",
            "3": "...--",
            "4": "....-",
            "5": ".....",
            "6": "-....",
            "7": "--...",
            "8": "---..",
            "9": "----.",
            "0": "-----",
        }
        self._morse_message = ""
        for character in message:
            if character != " ":
                self._morse_message += " ".join(MORSE_CODE_DICT[character]) + "  "
            else:
                self._morse_message += "     "
        # Add more space at the end
        self._morse_message += "  "

    def execute(self) -> None:
        if self.cur_pattern is DisplayType.FLASH:
            colour = self._flash_calculation()
        elif self.cur_pattern is DisplayType.PULSE:
            colour = self._pulse_calculation()
        elif self.cur_pattern is DisplayType.SOLID:
            colour = self.colour
        elif self.cur_pattern is DisplayType.RAINBOW:
            colour = self._rainbow_calculation()
        elif self.cur_pattern is DisplayType.MORSE:
            colour = self._morse_calculation()
        elif self.cur_pattern is DisplayType.PACMAN:
            self._pacman_calculation()
            return  # pacman sets the leds its self

        self.single_led_data.setHSV(colour[0], colour[1], colour[2])
        self.leds.setData(self.leds_data)
