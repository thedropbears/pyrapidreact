import logging
import math

from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import controller, trajectory
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
import wpilib

from components.chassis import Chassis
from controllers.shooter import ShooterController
from controllers.indexer import IndexerController
from components.indexer import Indexer
from components.intake import Intake
from utilities import trajectory_generator
from typing import List
from enum import Enum, auto

from utilities.functions import constrain_angle


class WaypointType(Enum):
    PICKUP = auto()
    SHOOT = auto()
    SIMPLE = auto()


class Waypoint:
    def __init__(
        self,
        x: float,
        y: float,
        rotation: Rotation2d,
        waypoint_type: WaypointType = WaypointType.SIMPLE,
    ) -> None:
        """x, y: field position in meters
        rotation: desired robot heading
        """
        self.pose = Pose2d(x, y, rotation)
        self.type = waypoint_type


class AutoBase(AutonomousStateMachine):
    chassis: Chassis
    indexer: Indexer
    intake: Intake
    indexer_control: IndexerController
    shooter_control: ShooterController

    field: wpilib.Field2d

    logger: logging.Logger

    max_speed = 2.5
    max_accel = 1.75

    ALLOWED_TRANS_ERROR = 0.2
    ALLOWED_ROT_ERROR = math.radians(20)

    def __init__(self, waypoints: List[Waypoint]):
        super().__init__()
        self.waypoints = waypoints
        self.waypoints_poses = [w.pose for w in self.waypoints]
        # applies to the linear speed, not turning
        self.linear_constraints = TrapezoidProfile.Constraints(
            self.max_speed, self.max_accel
        )

        self.drive_rotation_constraints = (
            trajectory.TrapezoidProfileRadians.Constraints(2, 2)
        )

        rotation_controller = controller.ProfiledPIDControllerRadians(
            2, 0, 0, self.drive_rotation_constraints
        )
        rotation_controller.enableContinuousInput(-math.pi, math.pi)
        self.drive_controller = controller.HolonomicDriveController(
            controller.PIDController(1, 0, 0.2),
            controller.PIDController(1, 0, 0.2),
            rotation_controller,
        )

        # all in meters along straight line path
        self.total_length = trajectory_generator.total_length(self.waypoints_poses)
        # how far around the current position is used to smooth the path
        self.look_around = 0.3
        # the index of the waypoint we are currently going towards or at
        self.cur_waypoint = 0

        self.last_pose = self.waypoints[0].pose
        self.trap_profile_start_time = 0.0

        wpilib.SmartDashboard.putNumber("auto_vel", 0.0)

    def setup(self) -> None:
        field_goal = self.field.getObject("goal")
        field_goal.setPose(trajectory_generator.goal_to_field(Pose2d(0, 0, 0)))

    def on_enable(self):
        self.chassis.set_pose(self.waypoints_poses[0])

        self.last_pose = self.waypoints[0].pose
        # generates initial velocity profile
        self.cur_waypoint = 0
        self.trap_profile = self._generate_trap_profile(TrapezoidProfile.State(0, 0))
        self.indexer_control.ignore_colour = True
        super().on_enable()

    @state(first=True)
    def startup(self):
        if self.waypoints[0].type is WaypointType.SHOOT:
            self.next_state("firing")
        else:
            self.next_state("move")

    @state
    def move(self, tm: float) -> None:
        # indexer controller will hanle it self raising and lowering
        if self.indexer.ready_to_intake():
            self.intake.deployed = True
            self.indexer_control.wants_to_intake = True
        # calculate speed and position from current trapazoidal profile
        trap_time = tm - self.trap_profile_start_time
        linear_state = self.trap_profile.calculate(trap_time)

        # find current goal pose
        goal_pose = trajectory_generator.smooth_path(
            self.waypoints_poses,
            self.look_around,
            linear_state.position,
        )
        goal_rotation = goal_pose.rotation()
        # the difference between last goal pose and this goal pose
        goal_pose_diff = goal_pose.translation() - self.last_pose.translation()
        # set the rotation on the goal pose to the direction its traveling for the feedforward
        goal_pose_fake = Pose2d(
            goal_pose.translation(), Rotation2d(goal_pose_diff.x, goal_pose_diff.y)
        )

        cur_pose = self.chassis.estimator.getEstimatedPosition()

        # check if we're done current waypoint
        translation_error = cur_pose.translation().distance(goal_pose.translation())
        rotation_error = constrain_angle(
            cur_pose.rotation().radians() - goal_pose.rotation().radians()
        )
        is_close = (
            abs(translation_error) < self.ALLOWED_TRANS_ERROR
            and abs(rotation_error) < self.ALLOWED_ROT_ERROR
        )
        if self.trap_profile.isFinished(trap_time) and (
            self.waypoints[self.cur_waypoint].type is WaypointType.SIMPLE or is_close
        ):
            self.logger.info(f"Got to waypoint{self.cur_waypoint} at {tm}")
            waypoint_type = self.waypoints[self.cur_waypoint].type
            if waypoint_type is WaypointType.SHOOT:
                self.next_state("firing")
            elif waypoint_type is WaypointType.PICKUP:
                self.next_state("pickup")
            else:
                self.move_next_waypoint(tm)

        # currentPose rotation and linearVelocityRef is only used for feedforward
        self.chassis_speeds = self.drive_controller.calculate(
            currentPose=cur_pose,
            poseRef=goal_pose_fake,
            linearVelocityRef=linear_state.velocity * 0.15,  # used for feedforward
            angleRef=goal_rotation,
        )
        self.chassis.drive_local(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

        # send poses to driverstation
        display_poses = [goal_pose, cur_pose]
        self.field.getRobotObject().setPoses(
            [trajectory_generator.goal_to_field(p) for p in display_poses]
        )
        wpilib.SmartDashboard.putNumber("auto_vel", float(linear_state.velocity))

        self.last_pose = goal_pose

    @state
    def pickup(self, state_tm: float, tm: float) -> None:
        """Waits until full"""
        if self.indexer.ready_to_intake():
            self.intake.deployed = True
            self.indexer_control.wants_to_intake = True
        if (
            self.indexer.has_cargo_in_chimney()
            and self.indexer.has_cargo_in_tunnel()
            or state_tm > 6
        ):
            self.next_state("move")
            self.move_next_waypoint(tm)

    @state
    def firing(self, state_tm: float, tm: float) -> None:
        """Waits until empty"""
        self.shooter_control.fire()
        self.intake.deployed = False
        if state_tm > 2 or not (
            self.indexer.has_cargo_in_chimney()
            or self.indexer.has_cargo_in_tunnel()
            or self.indexer_control.current_state == "transferring_to_chimney"
            or self.indexer_control.current_state == "firing"
        ):
            self.next_state("move")
            self.move_next_waypoint(tm)

    @state
    def finished(self) -> None:
        """Finished, keeps trying to fire incase we have any balls left"""
        self.shooter_control.fire()
        self.intake.deployed = False

    def move_next_waypoint(self, cur_time: float) -> None:
        """Creates the trapazoidal profile to move to the next waypoint"""
        if self.cur_waypoint >= len(self.waypoints) - 1:
            self.next_state("finished")
            return
        # last state in the current profile
        last_end = self.trap_profile.calculate(self.trap_profile.totalTime())
        self.cur_waypoint += 1
        self.trap_profile = self._generate_trap_profile(last_end)
        self.trap_profile_start_time = cur_time

    def _generate_trap_profile(
        self, current_state: TrapezoidProfile.State
    ) -> TrapezoidProfile:
        """Generates a linear trapazoidal trajectory that goes from current state to goal waypoint"""
        end_point = trajectory_generator.total_length(
            self.waypoints_poses[: self.cur_waypoint + 1]
        )
        waypoint_type = self.waypoints[self.cur_waypoint].type
        if (
            waypoint_type is WaypointType.SHOOT
            or waypoint_type is WaypointType.PICKUP
            or self.cur_waypoint >= len(self.waypoints)
        ):
            end_speed = 0.0
        else:
            end_speed = self.max_speed
        return TrapezoidProfile(
            self.linear_constraints,
            goal=TrapezoidProfile.State(end_point, end_speed),
            initial=current_state,
        )


# balls positions are described in https://docs.google.com/document/d/1K2iGdIX5vyCDEaJtaLdUiC-ihC9xyGYjrKFfLbvpusI/edit
# note these are positions of the balls, not where you should go to pick them up
# so actual auto waypoints should be chosen manually, only using these as referances
blue_balls = [
    (-3.299, 2.116),  # left
    (-3.219, -2.176),  # middle
    (-0.681, -3.826),  # right
    (-7.156, -3.010),  # terminal
    (-7.924, -1.559),  # cargo line
]
red_balls = [
    (-2.249, 3.169),  # left
    (-3.771, -0.935),  # middle
    (-0.850, -3792),  # right
]
# start positions
right_mid_start = Pose2d(-0.711, -2.419, Rotation2d.fromDegrees(-88.5))
right_left_start = Pose2d(-1.846, -1.555, Rotation2d.fromDegrees(-133.5))
left_mid_start = Pose2d(-2.273, 1.090, Rotation2d.fromDegrees(136.5))


class TestAuto(AutoBase):
    MODE_NAME = "Test"

    def __init__(self):
        super().__init__(
            [
                Waypoint(0, 0, Rotation2d.fromDegrees(0)),
                Waypoint(2, 0, Rotation2d.fromDegrees(90)),
                Waypoint(2, 2, Rotation2d.fromDegrees(180)),
                Waypoint(0, 2, Rotation2d.fromDegrees(270)),
                Waypoint(0, 0, Rotation2d.fromDegrees(0)),
            ]
        )


class FiveBall(AutoBase):
    """Auto starting middle of right tarmac, picking up balls 3, 2 and both at 4"""

    MODE_NAME = "Five Ball"
    DEFAULT = True

    def __init__(self):
        super().__init__(
            [
                Waypoint(-0.711, -2.419, Rotation2d.fromDegrees(-88.5)),  # start
                Waypoint(
                    -0.711, -3.3, Rotation2d.fromDegrees(-95), WaypointType.SHOOT
                ),  # 3
                Waypoint(-1.5, -2.7, Rotation2d.fromDegrees(-200)),
                Waypoint(
                    -2.9, -2.378, Rotation2d.fromDegrees(-206), WaypointType.SHOOT
                ),  # 2
                Waypoint(
                    -7.1, -2.65, Rotation2d.fromDegrees(-136), WaypointType.PICKUP
                ),  # 4
                Waypoint(
                    -5.0, 0, Rotation2d.fromDegrees(-130), WaypointType.SHOOT
                ),  # shoot
            ]
        )
