from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import trajectory, geometry, controller
import wpilib
import ctre


class AutoBase(AutonomousStateMachine):
    def __init__(self):
        super().__init__()
        max_accel = 0
        max_vel = 0
        trajectory.TrajectoryConfig(max_vel, max_accel)
        self.target_trajectory = None

    @state(first=True)
    def starting(self):
        start_position = geometry.Pose2d(geometry.Translation2d(0,0), geometry.Rotation2d(0,0))
        end_position = geometry.Pose2d(geometry.Translation2d(-1,0), geometry.Rotation2d(0,0))
        self.target_trajectory = trajectory.TrajectoryGenerator.generateTrajectory(start=start_position, end=end_position)
        self.next_state("move")
        pass

    @state
    def move(self):
        
        pass
    
    @state
    def shoot(self):
        pass