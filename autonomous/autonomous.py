from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import controller, trajectory
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrapezoidProfile
import wpilib

from components.chassis import Chassis
from components.indexer import Indexer
from controllers.shooter import ShooterController
from utilities import trajectory_generator
import math
from typing import List


class AutoBase(AutonomousStateMachine):
    """Follows a smoothed linear path between waypoints
    if it has a ball it stops before the next waypoint to fire it, otherwise dosent stop"""

    chassis: Chassis
    indexer: Indexer
    shooter_control: ShooterController

    field: wpilib.Field2d

    waypoints: List[Pose2d]

    def __init__(self):
        super().__init__()
        # applies to the linear speed, not turning
        self.linear_constraints = TrapezoidProfile.Constraints(1.5, 1)
        # since chassis speeds should be limited in trajectory generation this is high to not hold it back
        self.drive_rotation_constrants = trajectory.TrapezoidProfileRadians.Constraints(
            3, 5
        )
        self.drive_controller = controller.HolonomicDriveController(
            controller.PIDController(5, 0, 0),
            controller.PIDController(5, 0, 0),
            controller.ProfiledPIDControllerRadians(
                0.5, 0, 0, self.drive_rotation_constrants
            ),
        )

        # all in meters along straight line path
        self.total_length = trajectory_generator.total_length(self.waypoints)
        self.pre_stop = 1  # how far before the next waypoint to stop if you have a ball
        # self.stop_point = self.total_length
        self.goal = 1
        self.stop_point = (
            trajectory_generator.total_length(self.waypoints[: self.goal])
            - self.pre_stop
        )
        print(
            f"[{self.MODE_NAME}] total length {trajectory_generator.total_length(self.waypoints)}s"
        )

        # generates initial velocity profileW
        self.trap_profile = TrapezoidProfile(
            self.linear_constraints,
            goal=TrapezoidProfile.State(self.stop_point, 0),
            initial=TrapezoidProfile.State(0, 0),
        )
        self.last_pose = self.waypoints[0]
        self.trap_profile_start_time = 0

        wpilib.SmartDashboard.putNumber("auto_vel", 0.0)

    def setup(self):
        field_bballs = self.field.getObject("bballs")
        field_rballs = self.field.getObject("rballs")
        field_goal = self.field.getObject("goal")
        field_bballs.setPoses(
            [trajectory_generator.ours_to_wpi(Pose2d(*b, 0)) for b in blue_balls]
        )
        field_rballs.setPoses(
            [trajectory_generator.ours_to_wpi(Pose2d(*b, 0)) for b in red_balls]
        )
        field_goal.setPose(trajectory_generator.ours_to_wpi(Pose2d(0, 0, 0)))

        # set target estimator pose to self.waypoints[0]

    @state(first=True)
    def move(self, state_tm):
        # always be trying to fire
        self.shooter_control.fire_input()
        # calculate speed and position from current trapazoidal profile
        trap_time = state_tm - self.trap_profile_start_time
        linear_state = self.trap_profile.calculate(trap_time)
        # find goal waypoint index
        # goal = trajectory_generator.next_waypoint() if self.indexer.has_ball() else trajectory_generator.next_waypoint()+1
        goal = self.goal + self.trap_profile.isFinished(trap_time)  # for testing
        if not goal == self.goal:  # if we want to move stop point
            if goal > len(self.waypoints):
                self.stop_point = trajectory_generator.total_length(self.waypoints)
            else:
                # stop point before next ball if we havent fired yet
                self.stop_point = (
                    trajectory_generator.total_length(self.waypoints[:goal])
                    - self.pre_stop
                )
            print(f"regenerating idx {goal}, pos {self.stop_point}")
            # regenerate velocity profile with initial velocity as current velocity
            self.trap_profile = TrapezoidProfile(
                self.linear_constraints,
                goal=TrapezoidProfile.State(self.stop_point, 0),
                initial=TrapezoidProfile.State(
                    linear_state.position, linear_state.velocity
                ),
            )
            self.trap_profile_start_time = state_tm
            # recalculate speed and position so we can use it in this loop
            linear_state = self.trap_profile.calculate(0)
            self.goal = goal

        # find distance to start and end so look_around can be adjusted to not look beyond edges
        to_start = linear_state.position
        to_end = self.stop_point - linear_state.position
        look_around = min(to_start, min(to_end, 0.5))
        # find current goal pose
        cur_pose = trajectory_generator.smooth_path(
            self.waypoints, look_around, linear_state.position
        )
        self.chassis_speeds = self.drive_controller.calculate(
            currentPose=self.chassis.odometry.getPose(),
            poseRef=cur_pose,
            linearVelocityRef=linear_state.velocity,  # (cur_pose.translation().distance(self.last_pose)),
            angleRef=cur_pose.rotation(),
        )
        if (
            self.trap_profile.isFinished(trap_time)
            and linear_state.position >= self.total_length
        ):
            print("############# DONE ############")
            self.next_state("stopped")
        # send poses to driverstation
        display_poses = [cur_pose, self.chassis.odometry.getPose()]
        self.field.getRobotObject().setPoses(
            [trajectory_generator.ours_to_wpi(p) for p in display_poses]
        )
        self.chassis.drive_local(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

        wpilib.SmartDashboard.putNumber("auto_vel", float(linear_state.velocity))

        self.last_pose = cur_pose

    @state
    def stopped(self):
        """Finished moving but still want to be trying to fire"""
        self.shooter_control.fire_input()
        if not self.indexer.has_ball:
            self.done()


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
start_positions = [
    Pose2d(-0.711, -2.419, math.radians(-88.5)),
    Pose2d(0, 0, 0),
    Pose2d(0, 0, 0),
]


class TestAuto(AutoBase):
    MODE_NAME = "Test"

    def __init__(self):
        self.waypoints = [
            Pose2d(0, 0, 0),
            Pose2d(2, 0, 0),
            Pose2d(2, 2, 0),
            Pose2d(4, 2, math.pi),
        ]
        super().__init__()


class FiveBall(AutoBase):
    MODE_NAME = "Five Ball"
    DEFAULT = True

    def __init__(self):
        self.waypoints = [
            start_positions[0],
            Pose2d(-0.711, -3.351, -math.pi / 2),
            Pose2d(-2.789, -2.378, math.radians(-206)),
            Pose2d(-6.813, -2.681, math.radians(-136)),
        ]
        super().__init__()
