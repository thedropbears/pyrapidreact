from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import controller, trajectory
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
import wpilib

from components.chassis import Chassis
from components.target_estimator import TargetEstimator
from controllers.shooter import ShooterController
from controllers.indexer import IndexerController
from utilities import trajectory_generator
import math
from typing import List, Tuple


class AutoBase(AutonomousStateMachine):
    """Follows a smoothed linear path between waypoints
    if it has a ball it stops before the next waypoint to fire it, otherwise dosent stop"""

    chassis: Chassis
    target_estimator: TargetEstimator
    indexer_control: IndexerController
    shooter_control: ShooterController

    field: wpilib.Field2d

    waypoints: List[Pose2d]

    def __init__(self):
        super().__init__()
        # applies to the linear speed, not turning
        self.linear_constraints = TrapezoidProfile.Constraints(1, 1)

        self.drive_rotation_constrants = trajectory.TrapezoidProfileRadians.Constraints(
            2, 2
        )

        rotation_controller = controller.ProfiledPIDControllerRadians(
            2, 0, 0, self.drive_rotation_constrants
        )
        rotation_controller.enableContinuousInput(-math.pi, math.pi)
        self.drive_controller = controller.HolonomicDriveController(
            controller.PIDController(0.5, 0, 0),
            controller.PIDController(0.5, 0, 0),
            rotation_controller,
        )

        # all in meters along straight line path
        self.total_length = trajectory_generator.total_length(self.waypoints)
        # how far around the current position is used to smooth the path
        self.look_around = 0.3

        self.logger.info(f"[{self.MODE_NAME}] total length {self.total_length}s")
        self.last_pose = self.waypoints[0]
        self.trap_profile_start_time = 0

        wpilib.SmartDashboard.putNumber("auto_vel", 0.0)

    def setup(self):
        field_bballs = self.field.getObject("bballs")
        field_rballs = self.field.getObject("rballs")
        field_goal = self.field.getObject("goal")
        field_bballs.setPoses(
            [trajectory_generator.goal_to_field(Pose2d(*b, 0)) for b in blue_balls]
        )
        field_rballs.setPoses(
            [trajectory_generator.goal_to_field(Pose2d(*b, 0)) for b in red_balls]
        )
        field_goal.setPose(trajectory_generator.goal_to_field(Pose2d(0, 0, 0)))

    def on_enable(self):
        self.target_estimator.set_pose(self.waypoints[0])

        self.last_pose = self.waypoints[0]
        # generates initial velocity profile
        self.trap_profile, self.stop_point = self.generate_trap_profile(
            len(self.waypoints), TrapezoidProfile.State(0, 0)
        )
        return super().on_enable()

    def generate_trap_profile(
        self, goal: int, current_state: TrapezoidProfile.State
    ) -> Tuple[TrapezoidProfile, float]:
        """Generates a linear trapazoidal trajectory that goes from current state to goal"""
        stop_point = trajectory_generator.total_length(self.waypoints[:goal])
        ret = (
            TrapezoidProfile(
                self.linear_constraints,
                goal=TrapezoidProfile.State(stop_point, 0),
                initial=current_state,
            ),
            stop_point,
        )
        self.logger.info(f"[{self.MODE_NAME}] generated {ret[0].totalTime()}s")
        return ret

    @state(first=True)
    def move(self, tm):
        # indexer controller will hanle it self raising and lowering
        self.indexer_control.wants_to_intake = True
        # always be trying to fire
        self.shooter_control.wants_to_fire = True
        # calculate speed and position from current trapazoidal profile
        trap_time = tm
        linear_state = self.trap_profile.calculate(trap_time)
        is_done = self.trap_profile.isFinished(
            trap_time
        )  # TODO: change to our error is below value

        if is_done:
            self.logger.info(f"[{self.MODE_NAME}] Done at {tm}")
            self.next_state("stopped")

        # find distance to start and end so look_around can be adjusted to not look beyond edges
        to_start = linear_state.position - self.trap_profile.calculate(0).position
        to_end = self.stop_point - linear_state.position
        look_around = min(to_start, min(to_end, self.look_around))
        # find current goal pose
        goal_pose = trajectory_generator.smooth_path(
            self.waypoints, look_around, linear_state.position
        )
        goal_rotation = goal_pose.rotation()
        # the difference between last goal pose and this goal pose
        goal_pose_diff = goal_pose.translation() - self.last_pose.translation()
        # set the rotation on the goal pose to the direction its traveling for the feedforward
        goal_pose_fake = Pose2d(
            goal_pose.translation(), Rotation2d(goal_pose_diff.x, goal_pose_diff.y)
        )

        cur_pose = self.chassis.odometry.getPose()
        # currentPose rotation and linearVelocityRef is only used for feedforward
        self.chassis_speeds = self.drive_controller.calculate(
            currentPose=cur_pose,
            poseRef=goal_pose_fake,
            linearVelocityRef=linear_state.velocity * 0.15,  # used for feedforward
            angleRef=goal_rotation,
        )
        # send poses to driverstation
        display_poses = [goal_pose, self.chassis.odometry.getPose()]
        self.field.getRobotObject().setPoses(
            [trajectory_generator.goal_to_field(p) for p in display_poses]
        )
        self.chassis.drive_local(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

        wpilib.SmartDashboard.putNumber("auto_vel", float(linear_state.velocity))
        wpilib.SmartDashboard.putNumber("auto_look_around", look_around)

        self.last_pose = goal_pose

    @state
    def stopped(self):
        """Finished moving but still want to be trying to fire,
        needed for second ball at terminal"""
        self.shooter_control.wants_to_fire = True


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
right_mid_start = Pose2d(-0.611, -2.319, Rotation2d.fromDegrees(-88.5))
right_left_start = Pose2d(-1.846, -1.555, Rotation2d.fromDegrees(-133.5))
left_mid_start = Pose2d(-2.273, 1.090, Rotation2d.fromDegrees(136.5))


class TestAuto(AutoBase):
    MODE_NAME = "Test"

    def __init__(self):
        self.waypoints = [
            Pose2d(0, 0, 0),
            Pose2d(2, 0, math.pi / 2),
            Pose2d(2, 2, math.pi),
            Pose2d(0, 2, 3 * math.pi / 2),
            Pose2d(0, 0, 0),
        ]
        super().__init__()


class FiveBall(AutoBase):
    """Auto starting middle of right tarmac, picking up balls 3, 2 and both at 4"""

    MODE_NAME = "Five Ball"
    DEFAULT = True

    def __init__(self):
        self.waypoints = [
            right_mid_start,
            Pose2d(-0.711, -3.5, -math.pi / 2),  # 3
            Pose2d(-2.789, -2.378, Rotation2d.fromDegrees(-206)),  # 2
            Pose2d(-6.85, -2.681, Rotation2d.fromDegrees(-136)),  # 4
        ]
        super().__init__()


class StealBall3(AutoBase):
    """Starting in middle corner of left tarmac, picking up ball 1, r2 and 4 (terminal)"""

    MODE_NAME = "Steal + terminal"

    def __init__(self):
        self.waypoints = [
            left_mid_start,
            Pose2d(-3, 1.9, Rotation2d.fromDegrees(136)),  # 1
            Pose2d(-3.632, -0.481, Rotation2d.fromDegrees(180 + 73)),  # r2
            Pose2d(-6.813, -2.681, Rotation2d.fromDegrees(-136)),  # 4
        ]
        super().__init__()
