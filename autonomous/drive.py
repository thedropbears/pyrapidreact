import traceback
from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import trajectory, geometry, controller
import wpilib
import ctre
import wpimath


class AutoBase(AutonomousStateMachine):

    MODE_NAME = "Drive Backward"
    DEFAULT = True

    def __init__(self):
        super().__init__()
        self.max_accel = 0.5
        self.max_vel = 2
        self.config = trajectory.TrajectoryConfig(self.max_vel, self.max_accel)
        self.target_trajectory = None
        constraints = trajectory.TrapezoidProfileRadians.Constraints(self.max_vel, self.max_accel)
        self.drive_controller = controller.HolonomicDriveController(controller.PIDController(0,0,0), controller.PIDController(0,0,0), controller.ProfiledPIDControllerRadians(0,0,0, constraints))

    @state(first=True)
    def starting(self):
        self.start_angle = geometry.Rotation2d(0,0)
        self.end_angle = geometry.Rotation2d(0,0)
        self.start_position = geometry.Pose2d(geometry.Translation2d(0,0), self.start_angle)
        self.end_position = geometry.Pose2d(geometry.Translation2d(-1,0), self.end_angle)
        self.target_trajectory = trajectory.TrajectoryGenerator.generateTrajectory(self.start_position, [], self.end_position, self.config)
        self.next_state("move")
        pass

    @state
    def move(self):
        chassis_speeds = self.drive_controller.calculate(self.start_position, self.target_trajectory.sample(0), self.end_angle)
        pass
    
    @state
    def shoot(self):
        pass