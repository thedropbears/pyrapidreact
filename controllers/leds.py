from components.shooter import Shooter
from components.indexer import Indexer
from components.chassis import Chassis
from controllers.shooter import ShooterController
from components.leds import StatusLights, LedColours
from components.vision import Vision


class LedController:
    indexer: Indexer
    chassis: Chassis
    shooter: Shooter
    shooter_control: ShooterController
    status_lights: StatusLights
    vision: Vision

    def execute(self) -> None:
        if not self.vision.is_ready():
            self.status_lights.pulse(LedColours.PINK)
        if not self.indexer.has_cargo_in_chimney():
            self.status_lights.solid(LedColours.ORANGE)
        elif not self.shooter_control.in_range():
            self.status_lights.solid(LedColours.BLUE)
        elif (
            self.chassis.translation_velocity.norm() > self.shooter_control.MAX_SPEED
            or self.chassis.rotation_velocity.radians()
            > self.shooter_control.MAX_ROTATION
        ):
            self.status_lights.solid(LedColours.RED)
        else:
            self.status_lights.solid(LedColours.GREEN)

        if self.shooter_control.auto_shoot and self.indexer.has_cargo_in_chimney():
            self.status_lights.pulse()
