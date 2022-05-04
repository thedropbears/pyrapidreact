from components.shooter import Shooter
from components.indexer import Indexer
from components.chassis import Chassis
from controllers.shooter import ShooterController
from components.leds import StatusLights, LedColours, DisplayType
from components.vision import Vision
import wpilib


class LedController:
    indexer: Indexer
    chassis: Chassis
    shooter: Shooter
    shooter_control: ShooterController
    status_lights: StatusLights
    vision: Vision

    def __init__(self):
        self.is_enabled = False

    def execute(self) -> None:
        if not self.vision.is_connected() and wpilib.RobotBase.isReal():
            self.status_lights.set(DisplayType.PULSE, LedColours.WHITE)
        elif not self.is_enabled:
            self.status_lights.set_disabled()
        elif (
            self.chassis.translation_velocity.norm() > self.shooter_control.MAX_SPEED
            or self.chassis.rotation_velocity.radians()
            > self.shooter_control.MAX_ROTATION
        ):
            self.status_lights.set(DisplayType.SOLID, LedColours.PINK)
        elif not self.indexer.is_full():
            self.status_lights.set(DisplayType.SOLID, LedColours.RED)
        elif self.indexer.has_cargo_in_tunnel() or self.indexer.has_cargo_in_chimney():
            self.status_lights.set(DisplayType.SOLID, LedColours.ORANGE)
        else:
            self.status_lights.set(DisplayType.SOLID, LedColours.GREEN)

        if self.shooter_control.auto_shoot and self.indexer.has_cargo_in_chimney():
            self.status_lights.set(DisplayType.PULSE)
