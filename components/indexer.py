import rev
import wpilib
from enum import Enum
from magicbot import tunable, feedback


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

    is_firing = tunable(False)
    is_red = tunable(False)
    indexer_speed = tunable(0.5)

    _tunnel_direction = Direction.OFF
    _chimney_direction = Direction.OFF
    _cat_flap_is_open = False

    has_trapped_cargo = tunable(False)

    def setup(self) -> None:
        self.indexer_tunnel_motor.setInverted(True)
        self.indexer_chimney_motor.setInverted(False)

    def execute(self) -> None:
        self.indexer_tunnel_motor.set(self.indexer_speed * self._tunnel_direction.value)
        self.indexer_chimney_motor.set(
            self.indexer_speed * self._chimney_direction.value
        )
        if self._cat_flap_is_open:
            self.cat_flap_piston.set(
                wpilib.DoubleSolenoid.Value.kForward
            )  # TODO check direction
        else:
            self.cat_flap_piston.set(
                wpilib.DoubleSolenoid.Value.kReverse
            )  # TODO check direction

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
        return self.colour_sensor.getProximity() > 400

    @feedback
    def has_opposition_cargo_in_tunnel(self) -> bool:
        raw = self.colour_sensor.getRawColor()
        return self.has_cargo_in_tunnel() and (raw.red > raw.blue) == self.is_red

    @feedback
    def ready_to_intake(self) -> bool:
        # We cannot have a cargo in the tunnel, and we can't already have two cargo (one in chimney and one trapped)
        return not self.has_cargo_in_tunnel() and not (
            self.has_cargo_in_chimney() and self.has_trapped_cargo
        )

    @feedback
    def get_colours(self) -> str:
        raw = self.colour_sensor.getRawColor()
        return f"r{raw.red:.3f}, g{raw.green:.3f}, b{raw.blue:.3f}"

    def open_cat_flap(self) -> None:
        self._cat_flap_is_open = True

    def close_cat_flap(self) -> None:
        self._cat_flap_is_open = False

    def run_tunnel_motor(self, direction: Direction) -> None:
        self._tunnel_direction = direction

    def run_chimney_motor(self, direction: Direction) -> None:
        self._chimney_direction = direction
