from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import geometry, controller, trajectory
from wpimath.trajectory import TrapezoidProfile
from wpimath.geometry import Rotation2d

from components.chassis import Chassis
from utilities import trajectory_generator


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
            geometry.Pose2d(3, 0, 0),
            geometry.Pose2d(3, 3, 0),
            geometry.Pose2d(6, 3, 0),
        ]
        # both in meters along straight line path
        self.stop_point = trajectory_generator.totalLength(self.waypoints)
        print("total length", trajectory_generator.totalLength(self.waypoints))

        # generates initial
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
            self.trap_profile = TrapezoidProfile(
                self.linear_constraints,
                goal=TrapezoidProfile.State(self.stop_point, 0),
                initial=TrapezoidProfile.State(
                    linear_state.position, linear_state.velocity
                ),
            )
            self.trap_profile_start_time = 0
            d, vel = self.trap_profile.calculate(
                state_tm - self.trap_profile_start_time
            )
        cur_pose = trajectory_generator.smoothPath(
            self.waypoints, 2, linear_state.position
        )
        # print(f"d:{linear_state.position}, cur_pose:{cur_pose}")
        self.chassis_speeds = self.drive_controller.calculate(
            self.chassis.odometry.getPose(),
            poseRef=cur_pose,
            linearVelocityRef=linear_state.velocity,
            angleRef=Rotation2d(cur_pose.rotation().radians() + 1.5),
        )
        # print(f"{round(linear_state.position, 1)}-chassis speeds: {self.chassis_speeds},\t goal pose: {cur_pose}")
        self.chassis.field.setRobotPose(cur_pose)
        self.chassis.drive_field(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

    @state
    def wait(self):
        pass
