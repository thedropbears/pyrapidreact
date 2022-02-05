import traceback
import magicbot
from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import trajectory, geometry, controller, kinematics
import wpilib
import ctre
import wpimath

from components.chassis import Chassis


class AutoBase(AutonomousStateMachine):

    MODE_NAME = "Drive Backward"
    DEFAULT = True

    chassis: Chassis

    def __init__(self):
        super().__init__()
        self.max_accel = 0.5
        self.max_vel = 0.5
        self.config = trajectory.TrajectoryConfig(self.max_vel, self.max_accel)
        self.target_trajectory = None
        constraints = trajectory.TrapezoidProfileRadians.Constraints(self.max_vel, self.max_accel)
        self.drive_controller = controller.HolonomicDriveController(controller.PIDController(0,0,0), controller.PIDController(0,0,0), controller.ProfiledPIDControllerRadians(0,0,0, constraints))
        #self.chassis_speeds = trajectory.ChassisSpeeds()

    @state(first=True)
    def starting(self):
        self.start_angle = geometry.Rotation2d(0,0)
        self.end_angle = geometry.Rotation2d(0,0)
        self.start_position = geometry.Pose2d(geometry.Translation2d(0,0), self.start_angle)
        self.end_position = geometry.Pose2d(geometry.Translation2d(-1,0), self.end_angle)
        self.target_trajectory = trajectory.TrajectoryGenerator.generateTrajectory(self.start_position, [], self.end_position, self.config)
        self.next_state("move")

    @state
    def move(self, state_tm):
        self.chassis_speeds = self.drive_controller.calculate(self.chassis.odometry.getPose(), self.target_trajectory.sample(state_tm), self.end_angle)
        self.chassis._drive(chassis_speeds)
        self.return_chassis()

    @magicbot.feedback
    def return_chassis(self):
        return chassis_speeds
    
    @state
    def shoot(self):
        pass