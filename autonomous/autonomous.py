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


class AutoBase(AutonomousStateMachine):
    """Follows a smoothed linear path between waypoints
    if it has a ball it stops before the next waypoint to fire it, otherwise dosent stop"""

    chassis: Chassis
    indexer: Indexer
    shooter_control: ShooterController

    def __init__(self):
        super().__init__()
        # applies to the linear speed, not turning
        self.linear_constraints = TrapezoidProfile.Constraints(1.5, 1)
        # since chassis speeds should be limited in trajectory generation this is high to not hold it back
        self.drive_rotation_constrants = trajectory.TrapezoidProfileRadians.Constraints(
            3, 5
        )
        self.drive_controller = controller.HolonomicDriveController(
            controller.PIDController(0, 0, 0),
            controller.PIDController(0, 0, 0),
            controller.ProfiledPIDControllerRadians(
                0, 0, 0, self.drive_rotation_constrants
            ),
        )

        # both in meters along straight line path
        self.pre_stop = 1  # how far before the next waypoint to stop if you have a ball
        self.stop_point = trajectory_generator.total_length(self.waypoints)
        print("total length", trajectory_generator.total_length(self.waypoints))

        # generates initial velocity profileW
        self.trap_profile = TrapezoidProfile(
            self.linear_constraints,
            goal=TrapezoidProfile.State(self.stop_point, 0),
            initial=TrapezoidProfile.State(0, 0),
        )
        self.trap_profile_start_time = 0

        wpilib.SmartDashboard.putNumber("auto_vel", 0.0)

    def setup(self):
        # set target estimator pose to self.waypoints[0]
        pass

    @state(first=True)
    def move(self, state_tm):
        # always be trying to fire
        self.shooter_control.fire_input()

        linear_state = self.trap_profile.calculate(
            state_tm - self.trap_profile_start_time
        )
        if False:  # if we want to move stop point
            # regenerate velocity profile with initial velocity as current velocity
            self.stop_point = (
                trajectory_generator.next_waypoint(
                    self.waypoints, linear_state.position
                )
                - self.pre_stop
            )
            self.trap_profile = TrapezoidProfile(
                self.linear_constraints,
                goal=TrapezoidProfile.State(self.stop_point, 0),
                initial=TrapezoidProfile.State(
                    linear_state.position, linear_state.velocity
                ),
            )
            self.trap_profile_start_time = wpilib.Timer.getFPGATimestamp()
            linear_state = self.trap_profile.calculate(
                state_tm - self.trap_profile_start_time
            )
        to_start = linear_state.position
        to_end = self.stop_point - linear_state.position
        look_around = min(
            to_start, min(to_end, 0.5)
        )  # make look around 0 at start and end
        cur_pose = trajectory_generator.smooth_path(
            self.waypoints, look_around, linear_state.position
        )
        self.chassis_speeds = self.drive_controller.calculate(
            currentPose=self.chassis.odometry.getPose(),
            poseRef=cur_pose,
            linearVelocityRef=linear_state.velocity,
            angleRef=cur_pose.rotation(),
        )
        # self.chassis.field.setRobotPose(cur_pose)  # for debugging
        self.chassis.drive_field(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

        wpilib.SmartDashboard.putNumber("auto_vel", float(linear_state.velocity))


# balls positions are described in https://docs.google.com/document/d/1K2iGdIX5vyCDEaJtaLdUiC-ihC9xyGYjrKFfLbvpusI/edit
# note these are positions of the balls, not where you should go to pick them up
# so actual auto waypoints should be chosen manually, only using these as referances
# 1 - (-3299.01, 2116.76)
# 2 - (-3219.33, -2176.8)
# 3 - (-681.01, -3826.06)
# 4 - (-7371.33, -3218.24) # terminal
# 5 - (-7924.80, -1559.69) # cargo line
# r1- (-3771.83, -935.86)
start_positions = {
    "start_1": Pose2d(0, 0, 0),
    "start_2": Pose2d(0, 0, 0),
    "start_3": Pose2d(0, 0, 0),
}


class TestAuto(AutoBase):
    MODE_NAME = "Test"
    DEFAULT = True

    def __init__(self):
        self.waypoints = [
            Pose2d(0, 0, 0),
            Pose2d(2, 0, math.tau),
        ]
        super().__init__()


# class FiveBall(AutoBase):
#     MODE_NAME = "Five Ball"
#     def __init__(self):
#         self.waypoints = []
#         super().__init__()
