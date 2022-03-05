from components.shooter import Shooter
from components.indexer import Indexer
from components.chassis import Chassis
from components.turret import Turret
from controllers.shooter import ShooterController
from components.leds import StatusLights, LedStates


class LedController:
    indexer: Indexer
    chassis: Chassis
    shooter: Shooter
    turret: Turret
    shooter_control: ShooterController
    status_lights: StatusLights

    def execute(self):
        if not self.indexer.has_cargo_in_chimney():
            self.status_lights.set(LedStates.NO_BALL)
        elif (
            self.shooter_control.distance > self.shooter_control.MAX_DIST
            or self.shooter_control.distance < self.shooter_control.MIN_DIST
        ):
            self.status_lights.set(LedStates.RANGE)
        elif (
            self.chassis.translation_velocity.norm() > self.shooter_control.MAX_SPEED
            or self.chassis.rotation_velocity.radians()
            > self.shooter_control.MAX_ROTATION
        ):
            self.status_lights.set(LedStates.SPEED)
        elif not self.shooter.is_at_speed() or not self.turret.is_on_target():
            self.status_lights.set(LedStates.TARGETING)
        else:
            self.status_lights.set(LedStates.READY)
