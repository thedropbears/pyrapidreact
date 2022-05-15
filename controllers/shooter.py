import math
from components.indexer import Indexer
from components.shooter import Shooter
from components.turret import Turret
from components.intake import Intake
from components.vision import Vision
from magicbot import (
    StateMachine,
    state,
    tunable,
    default_state,
    timed_state,
    feedback,
    will_reset_to,
)
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
import wpilib
import navx
import wpiutil.log
from utilities.functions import constrain_angle, interpolate


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
    flywheel_speed_lookup = (24.5, 27.0, 28.5, 30.0, 36.0, 38.0, 40.0, 41.0, 42.0, 42.5, 43.0)
    times_lookup =          (0.77, 0.86, 0.94, 1.02, 1.10, 1.18, 1.26, 1.34, 1.43, 1.51, 1.59)
    # fmt: on

    MAX_DIST = 8
    MIN_DIST = 2.75
    LEAD_MIN_DIST = 2

    MAX_SPEED = 1.5  # m/s
    MAX_ROTATION = 2.0  # rad/s
    MAX_ACCEL = 0.3  # G
    ALLOWABLE_TURRET_ERROR = 0.3  # m, ring is 1.22m diameter
    ALLOWABLE_FLYWHEEL_ERROR = 0.5  # rps

    _wants_to_fire = will_reset_to(False)
    field: wpilib.Field2d
    data_log: wpiutil.log.DataLog

    # dont want to lead shots in auto beacuse we are shoot on the move
    # and the it causes weird behavoir with wrapping
    lead_shots = tunable(False)

    auto_shoot = False

    def __init__(self) -> None:
        self.flywheels_running = True
        self.track_target = True
        self._reject_through_turret = False

    def setup(self) -> None:
        self.log_pose = wpiutil.log.DoubleArrayLogEntry(self.data_log, "/shooter/pose")
        self.log_vision = wpiutil.log.DoubleArrayLogEntry(
            self.data_log, "/shooter/vision"
        )
        self.field_effective_goal = self.field.getObject("effective_goal")
        self.field_effective_goal.setPose(Pose2d(0, 0, 0))

    def _track(self) -> None:
        cur_pose = self.chassis.estimator.getEstimatedPosition()

        # clamp speed used for leading shots at our max allowed shooting speed
        chassis_speed = self.chassis.translation_velocity.norm()
        if chassis_speed != 0:
            # adjust shot to hit while moving
            flight_time = 0.0  # Only compensate for time of flight if told to...

            chassis_unit_velocity = self.chassis.translation_velocity / chassis_speed
            chassis_velocity = chassis_unit_velocity * min(
                chassis_speed, self.MAX_SPEED
            )
            for _ in range(3):
                robot_movement = chassis_velocity * flight_time
                effective_pose = Pose2d(
                    cur_pose.translation() + robot_movement, cur_pose.rotation()
                )
                self.distance = effective_pose.translation().distance(Translation2d())
                if not self.lead_shots or self.distance < self.LEAD_MIN_DIST:
                    # Only run once if we aren't compensating.
                    # This will mean ToF is considered to be zero (ie no compensation)
                    break
                flight_time = interpolate(
                    self.distance, self.ranges_lookup, self.times_lookup
                )
        else:
            robot_movement = Translation2d()
            effective_pose = cur_pose

        self.field_effective_goal.setPose(Pose2d(-robot_movement, Rotation2d(0)))

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
            elif self._reject_through_turret:
                self.shooter.motor_speed = 8
            else:
                self.shooter.motor_speed = interpolate(
                    self.distance, self.ranges_lookup, self.flywheel_speed_lookup
                )

    @default_state
    def tracking(self) -> None:
        self._track()

        accel = math.hypot(
            self.imu.getWorldLinearAccelX(), self.imu.getWorldLinearAccelY()
        )
        if self.indexer.has_cargo_in_chimney():
            if (
                self.shooter.flywheel_error() < 2
                and self._reject_through_turret
                and self.turret.is_on_target(math.atan(math.radians(45)))
            ):
                self.next_state("firing")

            if (
                (self._wants_to_fire or self.auto_shoot)
                and self.turret.is_on_target(
                    math.atan(self.ALLOWABLE_TURRET_ERROR / self.distance)
                )
                and self.shooter.flywheel_error() < self.ALLOWABLE_FLYWHEEL_ERROR
                and self.distance > self.MIN_DIST
                and self.distance < self.MAX_DIST
                and self.chassis.translation_velocity.norm() < self.MAX_SPEED
                and abs(self.chassis.rotation_velocity.radians()) < self.MAX_ROTATION
                and accel < 0.5
            ):
                self.next_state("firing")

    @timed_state(duration=0.5, first=True, next_state="resetting", must_finish=True)
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
        self._track()
        self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)

    @state(must_finish=True)
    def resetting(self):
        self._reject_through_turret = False
        self.next_state("tracking")

    @feedback
    def turret_rejection(self):
        return self._reject_through_turret

    @feedback
    def distance_to_goal(self) -> float:
        return self.distance

    def in_range(self) -> bool:
        return self.distance < self.MAX_DIST and self.distance > self.MIN_DIST

    @feedback
    def on_target(self) -> bool:
        return self.turret.is_on_target(math.atan(self.ALLOWABLE_TURRET_ERROR / 9.0))

    def fire(self) -> None:
        self._wants_to_fire = True
