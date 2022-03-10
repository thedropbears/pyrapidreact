import wpilib
import random
import time
from enum import Enum
from typing import Optional, Tuple


MAX_BRIGHTNESS = 180  # Between 0-255 of Value on HSV scale


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

        self._display_morse = False
        self._morse_message = ""
        self._morse_start_time = time.monotonic()

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

    def morse(self) -> Tuple[int, int, int]:
        # Work out how far through the message we are
        DOT_LENGTH = 0.1  # seconds
        total_time = self._morse_length * DOT_LENGTH
        elapsed_time = time.monotonic() - self._morse_start_time
        if elapsed_time > total_time:
            # Restart the message
            self._morse_start_time = time.monotonic()
            elapsed_time = 0.0
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
            "HELP ME",
            "YOU PASS BUTTER",
            "ATTACK",
            "NO DISASSEMBLE",
            "IM BACK BABY",
            "KILL ALL HUMANS",
            "MALFUNCTION",
            "JOHNNY 5 ALIVE",
            "COMPLIANCE",
            "YOUR MOVE CREEP",
            "COME WITH ME IF YOU WANT TO LIVE",
            "HASTA LA VISTA BABY",
            "JAMES AND LUCIEN ARE COOL",
            "BITE MY SHINY METAL ASS",
            "BENDER IS GREAT",
            "IM SORRY DAVE",
            "MY MIND IS GOING",
            "EXTERMINATE",
            "SEGFAULT CORE DUMPED",
            "THIS PARTY SUCKS MORE THAN STAIRS",
            "MORSE CODE IS FOR NERDS",
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

    def execute(self) -> None:
        if self.display_morse:
            # IF we are doing morse, we ignore the flashing and pulsing
            colour = self.morse()
        else:
            if self.is_flashing:
                colour = self.flash_calculation()
            elif self.is_pulsing:
                colour = self.pulse_calculation()
            else:
                colour = self.colour
        self.single_led_data.setHSV(colour[0], colour[1], colour[2])
        self.leds.setData(self.leds_data)
