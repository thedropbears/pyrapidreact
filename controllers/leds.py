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

    def execute(self):
        if not self.vision.is_ready():
            if not self.status_lights.is_flashing:
                self.status_lights.flash(LedColours.PINK.value)
        elif not self.indexer.has_cargo_in_chimney():
            if not self.status_lights.is_flashing:
                self.status_lights.flash(LedColours.ORANGE.value)
        elif (
            self.shooter_control.distance > self.shooter_control.MAX_DIST
            or self.shooter_control.distance < self.shooter_control.MIN_DIST
        ):
            self.status_lights.solid(LedColours.BLUE.value)
        elif (
            self.chassis.translation_velocity.norm() > self.shooter_control.MAX_SPEED
            or self.chassis.rotation_velocity.radians()
            > self.shooter_control.MAX_ROTATION
        ):
            self.status_lights.solid(LedColours.RED.value)
        # elif not self.shooter.is_at_speed() or not self.turret.is_on_target():
        #     self.status_lights.solid(LedColours.PINK)
        else:
            self.status_lights.solid(LedColours.GREEN.value)
