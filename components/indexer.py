import rev
import wpilib
from enum import Enum
from magicbot import tunable, feedback


class CargoColour(Enum):
    NONE = wpilib.DriverStation.Alliance.kInvalid
    RED = wpilib.DriverStation.Alliance.kRed
    BLUE = wpilib.DriverStation.Alliance.kBlue

    def is_opposition(self) -> bool:
        return self.value != wpilib.DriverStation.getAlliance()

    def is_valid(self) -> bool:
        return self is not self.NONE


class Indexer:
    class Direction(Enum):
        OFF = 0
        FORWARDS = 1
        BACKWARDS = -1

    # The "tunnel" is the horizontal part of the indexer that the cargo enters first
    # The "chimney" is the vertical section of the indexer that feeds the shooter
    # The "cat flap" is the moving flap that allows an opposition cargo to be captured
    indexer_tunnel_motor: rev.CANSparkMax
    indexer_chimney_motor: rev.CANSparkMax
    colour_sensor: rev.ColorSensorV3
    lower_chimney_prox_sensor: wpilib.DigitalInput
    upper_chimney_prox_sensor: wpilib.DigitalInput
    cat_flap_piston: wpilib.DoubleSolenoid
    tunnel_break_beam: wpilib.DigitalInput

    is_firing = tunable(False)
    tunnel_speed = tunable(0.8)
    chimney_speed = tunable(1.0)

    _tunnel_direction = Direction.OFF
    _chimney_direction = Direction.OFF
    _cat_flap_is_open = False

    red_total = int(0)
    blue_total = int(0)

    has_trapped_cargo = tunable(False)

    def setup(self) -> None:
        self.indexer_tunnel_motor.restoreFactoryDefaults()
        self.indexer_tunnel_motor.setInverted(True)
        self.indexer_tunnel_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)
        self.indexer_chimney_motor.restoreFactoryDefaults()
        self.indexer_chimney_motor.setInverted(False)
        self.indexer_chimney_motor.setIdleMode(rev.CANSparkMax.IdleMode.kBrake)

    def execute(self) -> None:
        if self._tunnel_direction is Indexer.Direction.BACKWARDS:
            self.indexer_tunnel_motor.set(-1.0)
        elif self._tunnel_direction is Indexer.Direction.FORWARDS:
            self.indexer_tunnel_motor.set(self.tunnel_speed)
        else:
            self.indexer_tunnel_motor.set(0.0)
        self.indexer_chimney_motor.set(
            self.chimney_speed * self._chimney_direction.value
        )
        if self._cat_flap_is_open:
            self.cat_flap_piston.set(wpilib.DoubleSolenoid.Value.kForward)
        else:
            self.cat_flap_piston.set(wpilib.DoubleSolenoid.Value.kReverse)

        # Default state is for nothing to be moving and for the cat flap to be down
        self._tunnel_direction = self._chimney_direction = Indexer.Direction.OFF
        self._cat_flap_is_open = False

    @feedback
    def has_cargo_in_chimney(self) -> bool:
        return (
            not self.lower_chimney_prox_sensor.get()
            or not self.upper_chimney_prox_sensor.get()
        )

    @feedback
    def has_cargo_in_tunnel(self) -> bool:
        return not self.tunnel_break_beam.get()

    @feedback
    def last_cargo_was_opposition(self) -> bool:
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            return self.red_total > self.blue_total
        elif wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            return self.blue_total > self.red_total
        return True

    @feedback
    def ready_to_intake(self) -> bool:
        # We cannot have a cargo in the tunnel, and we can't already have two cargo (one in chimney and one trapped)
        if self.has_cargo_in_tunnel() or (
            self.has_cargo_in_chimney() and self.has_trapped_cargo
        ):
            return False

        return True

    @feedback
    def get_colours(self) -> str:
        raw = self.colour_sensor.getRawColor()
        return f"r{raw.red:.3f}, g{raw.green:.3f}, b{raw.blue:.3f}"

    @feedback
    def get_proximity(self) -> float:
        return self.colour_sensor.getProximity()

    @feedback
    def red_value(self) -> int:
        return self.red_total

    @feedback
    def blue_value(self) -> int:
        return self.blue_total

    def open_cat_flap(self) -> None:
        self._cat_flap_is_open = True

    def close_cat_flap(self) -> None:
        self._cat_flap_is_open = False

    def run_tunnel_motor(self, direction: Direction) -> None:
        self._tunnel_direction = direction

    def run_chimney_motor(self, direction: Direction) -> None:
        self._chimney_direction = direction

    def get_cargo_colour(self) -> CargoColour:

        colour = self.colour_sensor.getRawColor()

        # In testing, the value of blue when we have red cargo never went above 600
        if colour.red > 700 or colour.blue > 700:
            self.red_total += colour.red
            self.blue_total += colour.blue

        if self.blue_total > self.red_total:
            return CargoColour.BLUE
        elif self.red_total > self.blue_total:
            return CargoColour.RED
        else
            return CargoColour.NONE

    def reset_cargo_colour(self) -> None:
        self.blue_total = 0
        self.red_total = 0
