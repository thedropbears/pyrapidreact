from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import geometry, controller, trajectory
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d

from components.chassis import Chassis
from utilities import trajectory_generator
import math


class AutoBase(AutonomousStateMachine):
    """Follows a smoothed linear path between waypoints
    if it has a ball it stops before the next waypoint to fire it, otherwise dosent stop"""

    MODE_NAME = "Drive Backward"
    DEFAULT = True

    chassis: Chassis

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

        self.waypoints = [
            geometry.Pose2d(0, 0, 0),
            geometry.Pose2d(2, 0, math.tau),
            geometry.Pose2d(4, 4, 0),
            geometry.Pose2d(6, 0, math.tau),
        ]
        # both in meters along straight line path
        self.stop_point = trajectory_generator.totalLength(self.waypoints)
        print("total length", trajectory_generator.totalLength(self.waypoints))

        # generates initial velocity profileW
        self.trap_profile = TrapezoidProfile(
            self.linear_constraints,
            goal=TrapezoidProfile.State(self.stop_point, 0),
            initial=TrapezoidProfile.State(0, 0),
        )
        self.trap_profile_start_time = 0

    @state(first=True)
    def move(self, state_tm):
        linear_state = self.trap_profile.calculate(
            state_tm - self.trap_profile_start_time
        )
        if False:  # if we want to move stop point
            # regenerate velocity profile with initial velocity as current velocity
            self.trap_profile = TrapezoidProfile(
                self.linear_constraints,
                goal=TrapezoidProfile.State(self.stop_point, 0),
                initial=TrapezoidProfile.State(
                    linear_state.position, linear_state.velocity
                ),
            )
            self.trap_profile_start_time = 0
            linear_state = self.trap_profile.calculate(
                state_tm - self.trap_profile_start_time
            )
        to_start = linear_state.position
        to_end = self.stop_point - linear_state.position
        look_around = min(
            to_start, min(to_end, 1)
        )  # make look around 0 at start and end
        cur_pose = trajectory_generator.smoothPath(
            self.waypoints, look_around, linear_state.position
        )
        self.chassis_speeds = self.drive_controller.calculate(
            self.chassis.odometry.getPose(),
            poseRef=cur_pose,
            linearVelocityRef=linear_state.velocity,
            angleRef=Rotation2d(cur_pose.rotation().radians()),
        )
        self.chassis.field.setRobotPose(cur_pose)
        self.chassis.drive_field(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )
