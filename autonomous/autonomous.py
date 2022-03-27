import logging
import math

from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import controller, trajectory
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.trajectory import (
    Trajectory,
    TrajectoryConfig,
    TrajectoryGenerator,
    constraint,
)
import wpilib

from components.chassis import Chassis
from controllers.shooter import ShooterController
from controllers.indexer import IndexerController
from components.indexer import Indexer
from components.intake import Intake

from dataclasses import dataclass
from typing import List
from enum import Enum, auto


class WaypointType(Enum):
    PICKUP = auto()
    SHOOT = auto()
    SIMPLE = auto()


@dataclass
class Movement:
    type: WaypointType
    trajectory: Trajectory
    chassis_heading: Rotation2d


class AutoBase(AutonomousStateMachine):
    chassis: Chassis
    indexer: Indexer
    intake: Intake
    indexer_control: IndexerController
    shooter_control: ShooterController

    field: wpilib.Field2d

    logger: logging.Logger

    drive_rotation_constraints = trajectory.TrapezoidProfileRadians.Constraints(8, 21)

    ALLOWED_TRANS_ERROR = 0.05
    ALLOWED_ROT_ERROR = math.radians(10)

    def __init__(self):
        super().__init__()

        rotation_controller = controller.ProfiledPIDControllerRadians(
            4, 0, 0.3, self.drive_rotation_constraints
        )
        rotation_controller.enableContinuousInput(-math.pi, math.pi)
        self.drive_controller = controller.HolonomicDriveController(
            controller.PIDController(3.5, 0, 0.4),
            controller.PIDController(3.5, 0, 0.4),
            rotation_controller,
        )
        self.drive_controller.setTolerance(
            Pose2d(
                self.ALLOWED_TRANS_ERROR,
                self.ALLOWED_TRANS_ERROR,
                self.ALLOWED_ROT_ERROR,
            )
        )
        self.movements: List[Movement] = []

        wpilib.SmartDashboard.putNumber("auto_vel", 0.0)

    def setup(self) -> None:
        self.field_auto_target_pose = self.field.getObject("auto_target_pose")
        self.auto_trajectory = self.field.getObject("auto_trajectory")
        self.field_auto_target_pose.setPose(Pose2d(0, 0, 0))

        # Leave some headroom over the max unloaded speed
        max_speed = self.chassis.max_attainable_wheel_speed * 0.7
        self.trajectory_config = TrajectoryConfig(
            maxVelocity=max_speed, maxAcceleration=2.9
        )  # Acceleration expressed as max_speed / t where t is time taken to reach max speed
        self.trajectory_config.addConstraint(
            constraint.SwerveDrive4KinematicsConstraint(
                self.chassis.kinematics,
                maxSpeed=self.chassis.max_attainable_wheel_speed,
            )
        )
        # add additional constraints here if required

    def on_enable(self) -> None:
        self.chassis.set_pose(self.movements[0].trajectory.initialPose())

        # generates initial velocity profile
        self.current_movement_idx = 0
        self.current_movement = self.movements[self.current_movement_idx]
        self.current_trajectory = self.current_movement.trajectory
        self.trajectory_start_time = 0.0
        self.indexer_control.ignore_colour = True
        self.auto_trajectory.setTrajectory(self.current_trajectory)
        super().on_enable()

    @state(first=True)
    def startup(self, tm: float) -> None:
        # To make an initial shoot state, create a tiny trajectory
        self.trajectory_start_time = tm
        self.next_state("move")

    @state
    def move(self, tm: float) -> None:
        # indexer controller will hanle it self raising and lowering
        if self.indexer.ready_to_intake():
            self.intake.deployed = True
            self.indexer_control.wants_to_intake = True
        # calculate speed and position from current trajectory
        traj_time = tm - self.trajectory_start_time
        target_state = self.current_trajectory.sample(traj_time)
        target_heading = self.current_movement.chassis_heading

        current_pose = self.chassis.get_pose()

        if traj_time > self.current_trajectory.totalTime() and (
            self.drive_controller.atReference() or wpilib.RobotBase.isSimulation()
        ):
            self.logger.info(
                f"Got to end of movement {self.current_movement_idx} at {tm}"
            )
            if self.current_movement.type is WaypointType.SHOOT:
                self.next_state("firing")
            elif self.current_movement.type is WaypointType.PICKUP:
                self.next_state("pickup")
            else:
                self.move_next_waypoint(tm)
        self.chassis_speeds = self.drive_controller.calculate(
            currentPose=current_pose,
            desiredState=target_state,
            angleRef=target_heading,
        )
        self.chassis.drive_local(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

        # send poses to driverstation
        display_pose = Pose2d(target_state.pose.translation(), target_heading)
        self.field_auto_target_pose.setPose(display_pose)

        wpilib.SmartDashboard.putNumber("auto_vel", float(target_state.velocity))

        self._maybe_hail_mary()

    @state
    def pickup(self, state_tm: float, tm: float) -> None:
        """Waits until full"""
        self.intake.deployed = True
        self.indexer_control.wants_to_intake = True
        if (
            self.indexer.has_cargo_in_chimney()
            and self.indexer.has_cargo_in_tunnel()
            or state_tm > 1.5
        ) or (wpilib.RobotBase.isSimulation() and state_tm > 1):
            self.next_state("move")
            self.move_next_waypoint(tm)

        self._maybe_hail_mary()

    def _maybe_hail_mary(self) -> None:
        """Shoot if we're reaching the end of autonomous and we have a ball."""
        match_time = wpilib.DriverStation.getMatchTime()
        if (
            -1 < match_time <= 2.5
            and self.indexer.has_cargo_in_chimney()
        ):
            self.next_state("firing")

    @state
    def firing(self, state_tm: float, tm: float) -> None:
        """Waits until empty"""
        self.shooter_control.fire()
        self.intake.deployed = False
        if state_tm > 2.5 or not (
            self.indexer.has_cargo_in_chimney()
            or self.indexer.has_cargo_in_tunnel()
            or self.indexer_control.current_state == "transferring_to_chimney"
            or self.indexer_control.current_state == "firing"
            or wpilib.RobotBase.isSimulation()
        ):
            self.next_state("move")
            self.move_next_waypoint(tm)

    @state
    def finished(self) -> None:
        """Finished, keeps trying to fire incase we have any balls left"""
        self.shooter_control.fire()
        self.intake.deployed = False

    def move_next_waypoint(self, cur_time: float) -> None:
        """Translates the generated trajectory to move to the next waypoint"""
        if self.current_movement_idx == len(self.movements) - 1:
            # last trajectory in the current profile
            self.next_state("finished")
            return

        self.current_movement_idx += 1
        self.current_movement = self.movements[self.current_movement_idx]
        self.current_trajectory = self.current_movement.trajectory

        self.auto_trajectory.setTrajectory(self.current_trajectory)

        self.trajectory_start_time = cur_time


# balls positions are described in https://docs.google.com/document/d/1K2iGdIX5vyCDEaJtaLdUiC-ihC9xyGYjrKFfLbvpusI/edit

# start positions
# don't change rotations because the initial robot pose is set from them!
right_mid_start = Pose2d(-0.630, -2.334, Rotation2d.fromDegrees(-88.5))
left_mid_start = Pose2d(-2.156, 1.093, Rotation2d.fromDegrees(136.5))


class TestAuto(AutoBase):
    MODE_NAME = "test"

    def setup(self) -> None:
        super().setup()
        self.movements = [
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(0, 0, Rotation2d.fromDegrees(-45)),
                    end=Pose2d(2, 0, Rotation2d.fromDegrees(45)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(45),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(2, 0, Rotation2d.fromDegrees(45)),
                    end=Pose2d(2, 2, Rotation2d.fromDegrees(135)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(135),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(2, 2, Rotation2d.fromDegrees(135)),
                    end=Pose2d(0, 2, Rotation2d.fromDegrees(225)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(225),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(0, 2, Rotation2d.fromDegrees(225)),
                    end=Pose2d(0, 0, Rotation2d.fromDegrees(315)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(315),
            ),
        ]


class ExerciseAuto(AutoBase):
    MODE_NAME = "exercise"
    # Run a test routine in confined spaces that has the same total distance and changes of direction as the 5 ball

    def setup(self) -> None:
        super().setup()
        self.movements = [
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-1.5, 0, Rotation2d.fromDegrees(180)),
                    end=Pose2d(-3, 0, Rotation2d.fromDegrees(180)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(180),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-3, 0, Rotation2d.fromDegrees(0)),
                    end=Pose2d(-1.5, 2, Rotation2d.fromDegrees(90)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(90),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-1.5, 2, Rotation2d.fromDegrees(-90)),
                    end=Pose2d(-1.5, -2, Rotation2d.fromDegrees(-90)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(-90),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-1.5, -2, Rotation2d.fromDegrees(90)),
                    end=Pose2d(-3, 2, Rotation2d.fromDegrees(180)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(135),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-3, 2, Rotation2d.fromDegrees(-90)),
                    end=Pose2d(-1.5, 0, Rotation2d.fromDegrees(0)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(180),
            ),
        ]


class FiveBall(AutoBase):
    """Auto starting middle of right tarmac, picking up balls 3, 2 and both at terminal"""

    MODE_NAME = "Five Ball: Right - Terminal"
    DEFAULT = True

    def setup(self) -> None:
        super().setup()
        self.movements = [
            Movement(
                WaypointType.SHOOT,
                TrajectoryGenerator.generateTrajectory(
                    start=right_mid_start,
                    end=Pose2d(-0.65, -3.55, Rotation2d.fromDegrees(-90)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(-88.5),
            ),
            Movement(
                WaypointType.SHOOT,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-0.65, -3.55, Rotation2d.fromDegrees(100)),
                    end=Pose2d(-3.4, -2.1, Rotation2d.fromDegrees(180)),
                    interiorWaypoints=[
                        Translation2d(-1.8, -2.3),
                    ],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(180),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-3.4, -2.1, Rotation2d.fromDegrees(170)),
                    end=Pose2d(-6.95, -2.8, Rotation2d.fromDegrees(-135)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(-135),
            ),
            Movement(
                WaypointType.SHOOT,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-6.95, -2.8, Rotation2d.fromDegrees(45)),
                    end=Pose2d(-5.0, -2.0, Rotation2d.fromDegrees(0)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(-135),
            ),
        ]


class FourBall(AutoBase):
    """Auto starting middle of left tarmac, picking up ball 1 and both at terminal
    In case we have a partner who can do a three ball"""

    MODE_NAME = "Four Ball: Left - Terminal"

    def setup(self) -> None:
        super().setup()
        self.movements = [
            Movement(
                WaypointType.SHOOT,
                TrajectoryGenerator.generateTrajectory(
                    start=left_mid_start,
                    end=Pose2d(-3.1, 1.8, Rotation2d.fromDegrees(160)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(130),
            ),
            Movement(
                WaypointType.PICKUP,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-3.1, 1.8, Rotation2d.fromDegrees(-130)),
                    end=Pose2d(-7.25, -2.75, Rotation2d.fromDegrees(-135)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(-136),
            ),
            Movement(
                WaypointType.SHOOT,
                TrajectoryGenerator.generateTrajectory(
                    start=Pose2d(-7.25, -2.75, Rotation2d.fromDegrees(75)),
                    end=Pose2d(-5.0, 0.0, Rotation2d.fromDegrees(30)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(-136),
            ),
        ]


class TwoBall(AutoBase):
    """Auto starting middle of left tarmac, picking up ball 1
    In case we have a partner who can do a five ball
    """

    MODE_NAME = "Two Ball: Left"

    def setup(self) -> None:
        super().setup()
        self.movements = [
            Movement(
                WaypointType.SHOOT,
                TrajectoryGenerator.generateTrajectory(
                    start=left_mid_start,
                    end=Pose2d(-3.1, 1.8, Rotation2d.fromDegrees(130)),
                    interiorWaypoints=[],
                    config=self.trajectory_config,
                ),
                Rotation2d.fromDegrees(130),
            ),
        ]
