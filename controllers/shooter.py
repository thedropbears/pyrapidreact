import math
from components.indexer import Indexer
from components.shooter import Shooter
from components.turret import Turret
from components.intake import Intake
from components.vision import Vision
from magicbot import (
    StateMachine,
    tunable,
    default_state,
    timed_state,
    state,
    feedback,
)
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
import wpilib
import navx
import wpiutil.log
from utilities.functions import constrain_angle, interpolate
from enum import Enum, auto


class ShooterCommand(Enum):
    """Represents the intent of the shooter"""

    NONE = auto()
    FIRE = auto()
    CLEAR = auto()

    def try_fire(self) -> None:
        if self is self.NONE:
            self = self.FIRE

    def try_reset(self) -> None:
        if self is self.FIRE:
            self = self.NONE


class ShooterController(StateMachine):
    shooter: Shooter
    turret: Turret
    indexer: Indexer
    intake: Intake
    chassis: Chassis
    imu: navx.AHRS
    vision: Vision

    # If set to true, flywheel speed is set from tunable
    # Otherwise it is calculated from the interpolation table
    interpolation_override = tunable(False)
    flywheel_speed = tunable(0.0)

    distance = 0.0
    # fmt: off
    ranges_lookup =         ( 3.0,  3.5,  4.0,  4.5,  5.0,  5.5,  6.0,  6.5,  7.0,  7.5,  8.0)
    flywheel_speed_lookup = (27.5, 28.0, 30.0, 32.5, 36.0, 38.5, 41.0, 42.5, 43.5, 45.0, 47.0)
    times_lookup =          (0.85, 0.95, 1.04, 1.13, 1.22, 1.31, 1.40, 1.49, 1.59, 1.68, 1.77)
    # fmt: on

    MAX_DIST = 8
    MIN_DIST = 2.75

    MAX_SPEED = 2.0  # m/s
    MAX_ROTATION = 2.0  # rad/s
    MAX_ACCEL = 0.3  # G
    ALLOWABLE_TURRET_ERROR = 0.5  # m, ring is 1.22m diameter

    # firing limits for automatic shoot mode
    AUTO_MAX_DIST = 7.5
    AUTO_MIN_DIST = 3
    AUTO_MAX_SPEED = 1.0  # m/s
    AUTO_MAX_ROTATION = 1.0  # rad/s
    AUTO_MAX_ACCEL = 0.2  # G
    AUTO_ALLOWABLE_TURRET_ERROR = 0.3  # m, ring is 1.22m diameter

    _command = ShooterCommand.NONE
    field: wpilib.Field2d
    data_log: wpiutil.log.DataLog

    # dont want to lead shots in auto beacuse we are shoot on the move
    # and the it causes weird behavoir with wrapping
    lead_shots = tunable(False)

    auto_shoot = False

    def __init__(self) -> None:
        self.flywheels_running = True
        self.track_target = True

    def setup(self) -> None:
        self.log_pose = wpiutil.log.DoubleArrayLogEntry(self.data_log, "/shooter/pose")
        self.log_vision = wpiutil.log.DoubleArrayLogEntry(
            self.data_log, "/shooter/vision"
        )
        self.field_effective_goal = self.field.getObject("effective_goal")
        self.field_effective_goal.setPose(Pose2d(0, 0, 0))

    @default_state
    def tracking(self) -> None:
        if self._command == ShooterCommand.CLEAR:
            self.next_state("prepare_to_clear")

        cur_pose = self.chassis.estimator.getEstimatedPosition()

        # adjust shot to hit while moving
        flight_time = 0.0  # Only compensate for time of flight if told to...

        for _ in range(3):
            robot_movement = self.chassis.translation_velocity * flight_time
            effective_pose = Pose2d(
                cur_pose.translation() + robot_movement, cur_pose.rotation()
            )
            self.distance = effective_pose.translation().distance(Translation2d())
            flight_time = interpolate(
                self.distance, self.ranges_lookup, self.times_lookup
            )
            if not self.lead_shots or self.distance < self.MIN_DIST:
                # Only run once if we aren't compensating. This will mean ToF is considered to be zero (ie no compensation)
                break

        self.field_effective_goal.setPose(
            Pose2d(-robot_movement.X(), -robot_movement.Y(), Rotation2d(0))
        )

        turret_pose = self.chassis.robot_to_world(
            self.shooter.turret_offset, effective_pose
        )
        field_angle = math.atan2(-turret_pose.Y(), -turret_pose.X())
        angle = constrain_angle(field_angle - cur_pose.rotation().radians())

        if self.track_target:
            self.turret.slew_local(angle)

        if self.flywheels_running:
            if self.interpolation_override:
                self.shooter.motor_speed = self.flywheel_speed
            else:
                self.shooter.motor_speed = interpolate(
                    self.distance, self.ranges_lookup, self.flywheel_speed_lookup
                )
        accel = math.hypot(
            self.imu.getWorldLinearAccelX(), self.imu.getWorldLinearAccelY()
        )
        if self.indexer.has_cargo_in_chimney() and self.shooter.is_at_speed():
            if (
                self._command == ShooterCommand.FIRE
                and self.turret.is_on_target(
                    math.atan(self.ALLOWABLE_TURRET_ERROR / self.distance)
                )
                and self.distance > self.MIN_DIST
                and self.distance < self.MAX_DIST
                and self.chassis.translation_velocity.norm() < self.MAX_SPEED
                and self.chassis.rotation_velocity.radians() < self.MAX_ROTATION
                and accel < self.MAX_ACCEL
            ):
                self.next_state("firing")

            if (
                self.auto_shoot
                and self.turret.is_on_target(
                    math.atan(self.AUTO_ALLOWABLE_TURRET_ERROR / self.distance)
                )
                and self.distance > self.AUTO_MIN_DIST
                and self.distance < self.AUTO_MAX_DIST
                and self.chassis.translation_velocity.norm() < self.AUTO_MAX_SPEED
                and self.chassis.rotation_velocity.radians() < self.AUTO_MAX_ROTATION
                and accel < self.AUTO_MAX_ACCEL
            ):
                self.next_state("firing")

        self._command.try_reset()

    @timed_state(duration=0.5, first=True, next_state="tracking", must_finish=True)
    def firing(self, initial_call) -> None:
        if initial_call:
            pose = self.chassis.get_pose()
            self.log_pose.append(
                [pose.X(), pose.Y(), pose.rotation().radians(), self.turret.get_angle()]
            )
            self.log_vision.append(
                [
                    self.vision.has_target,
                    self.vision.distance,
                    self.vision.target_pitch,
                    self.vision.target_yaw,
                ]
            )
        if self.flywheels_running:
            if self.interpolation_override:
                self.shooter.motor_speed = self.flywheel_speed
            else:
                self.shooter.motor_speed = interpolate(
                    self.distance, self.ranges_lookup, self.flywheel_speed_lookup
                )
        self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)

    @state(must_finish=True)
    def prepare_to_clear(self) -> None:
        self.shooter.motor_speed = 5
        if self.shooter.is_at_speed():
            self.next_state("clearing")

    @timed_state(duration=0.75, next_state="tracking", must_finish=True)
    def clearing(self) -> None:
        """Plop ball out shooter at low speed"""
        self._command = ShooterCommand.NONE
        self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)
        self.indexer.run_tunnel_motor(Indexer.Direction.FORWARDS)

    def clear(self) -> None:
        self._command = ShooterCommand.CLEAR

    @feedback
    def distance_to_goal(self) -> float:
        return self.distance

    def in_range(self) -> bool:
        return self.distance < self.MAX_DIST and self.distance > self.MIN_DIST

    def fire(self) -> None:
        self._command.try_fire()
