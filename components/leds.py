import math
import wpilib
import random
import time
from enum import Enum, auto
from typing import Optional, Tuple


MAX_BRIGHTNESS = 180  # Between 0-255 of Value on HSV scale


class LedColours(Enum):
    # Use HSV to get nicer fading
    RED = (0, 255, MAX_BRIGHTNESS)
    ORANGE = (30, 255, MAX_BRIGHTNESS)
    YELLOW = (70, 255, MAX_BRIGHTNESS)
    PINK = (150, 255, MAX_BRIGHTNESS)
    BLUE = (120, 255, MAX_BRIGHTNESS)
    CYAN = (140, 255, MAX_BRIGHTNESS)
    GREEN = (60, 255, MAX_BRIGHTNESS)
    OFF = (0, 0, 0)

class StatusLights:
    leds: wpilib.AddressableLED
    class DisabledPatternType(Enum):
        MORSE = auto()
        PACMAN = auto()
        RAINBOW = auto()
        OFF = auto()

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

        self.disabled_pattern = self.DisabledPatternType.OFF

        self._pacman_position = 0
        self._pacman_direction = 1
        self._morse_message = ""
        self._morse_start_time = time.monotonic()
        self._rainbow_hue = 0

    def fade(self, brightness: float) -> Tuple[int, int, int]:
        return (self.colour[0], self.colour[1], round(self.colour[2] * brightness))

    def setup(self) -> None:
        self.choose_morse_message()
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
        self.disabled_pattern = self.DisabledPatternType.OFF

    def flash(self, colour: Optional[LedColours] = None) -> None:
        if colour is not None:
            self.colour = colour.value
        self.is_flashing = True
        self.flash_timer = time.monotonic()
        self.is_pulsing = False
        self.disabled_pattern = self.DisabledPatternType.OFF

    def solid(self, colour: Optional[LedColours] = None) -> None:
        if colour is not None:
            self.colour = colour.value
        self.is_flashing = False
        self.is_pulsing = False
        self.disabled_pattern = self.DisabledPatternType.OFF

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

    @property
    def _morse_length(self) -> int:
        # A dash is three times as long as a dot
        # A space between characters is three dots
        # A space between dots and dashes is one dot
        # A space between words is 7 dots
        return (
            self._morse_message.count(".")
            + 3 * self._morse_message.count("-")
            + self._morse_message.count(" ")
        )

    def rainbow(self) -> Tuple[int, int, int]:
        self._rainbow_hue += 1
        if self._rainbow_hue > 255 * 2:
            self._rainbow_hue = 0
            self.pick_pasttime()
        return [round(self._rainbow_hue%255), 255, MAX_BRIGHTNESS]

    def morse(self) -> Tuple[int, int, int]:
        # Work out how far through the message we are
        DOT_LENGTH = 0.1  # seconds
        total_time = self._morse_length * DOT_LENGTH
        elapsed_time = time.monotonic() - self._morse_start_time
        if elapsed_time > total_time:
            self.choose_morse_message()
            self.pick_pasttime()
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
                    return self.colour
        # Default - should never be hit
        return LedColours.OFF.value

    def choose_morse_message(self, message: str = None) -> None:
        # Choose a morse message at random, unless specific message requested
        # Only use lowercase and spaces
        MESSAGES = [
            "KILL ALL HUMANS",
            "SEGFAULT CORE DUMPED",
            "MORSE CODE IS FOR NERDS",
            "HONEYBADGER DONT CARE",
        ]
        if message is None:
            message = random.choice(MESSAGES)
        else:
            message = message.upper()
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
        for character in message:
            if character != " ":
                self._morse_message += " ".join(MORSE_CODE_DICT[character]) + "   "
            else:
                self._morse_message += "    "
        # Add more space at the end
        self._morse_message += "  "
        self.display_morse = True

    def pacman(self) -> None:
        if self._pacman_direction == 1:
            # fmt: off
            runs = [
                [LedColours.ORANGE]* 2,
                [LedColours.OFF]   * 2,
                [LedColours.CYAN]  * 2,
                [LedColours.OFF]   * 2,
                [LedColours.PINK]  * 2,
                [LedColours.OFF]   * 2,
                [LedColours.RED]   * 2,
                [LedColours.OFF]   * 5,
                [LedColours.YELLOW]* 4,
            ]
        else:
            runs = [
                [LedColours.YELLOW]* 4,
                [LedColours.OFF]   * 5,
                [LedColours.BLUE]  * 2,
                [LedColours.OFF]   * 2,
                [LedColours.BLUE]  * 2,
                [LedColours.OFF]   * 2,
                [LedColours.BLUE]  * 2,
                [LedColours.OFF]   * 2,
                [LedColours.BLUE]  * 2,
            ]
            # fmt: on
        pattern = []
        for run in runs:
            pattern.extend(run)
        pattern = [wpilib.AddressableLED.LEDData().setHSV(*x.value) for x in pattern]
        data = []
        data.extend([LedColours.OFF.value]*math.floor(self._pacman_position))
        data.extend(pattern)
        leds_left = self.led_length-len(data)
        if leds_left < 0:
            data.extend([LedColours.OFF.value]*leds_left)
        self.leds.setData(self.leds_data)
        self._pacman_position += 0.5 * self._pacman_direction
        if self._pacman_position > self.led_length:
            self._pacman_direction = -1
        if self._pacman_position < 0:
            self._pacman_position = 0
            self._pacman_direction = 1
            self.pick_pasttime()

    def pick_pasttime(self):
        """Picks a random pattern to display"""
        last = self.disabled_pattern
        self.disabled_pattern = random.choice(list(self.DisabledPatternType._member_map_.values()))
        while self.disabled_pattern is self.DisabledPatternType.OFF and not self.disabled_pattern is last:
            self.disabled_pattern = random.choice(list(self.DisabledPatternType._member_map_.values()))
        print(f"picked {self.disabled_pattern.name} ##########################")

    def enable_patterns(self):
        if self.disabled_pattern is self.DisabledPatternType.OFF:
            self.pick_pasttime()

    def execute(self) -> None:
        if self.disabled_pattern is self.DisabledPatternType.RAINBOW:
            colour = self.rainbow()
        elif self.disabled_pattern is self.DisabledPatternType.MORSE:
            colour = self.morse()
        elif self.disabled_pattern is self.DisabledPatternType.PACMAN:
            self.pacman()
            return
        elif self.is_flashing:
            colour = self.flash_calculation()
        elif self.is_pulsing:
            colour = self.pulse_calculation()
        else:
            colour = self.colour
        self.single_led_data.setHSV(colour[0], colour[1], colour[2])
        self.leds.setData(self.leds_data)
