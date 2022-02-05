from magicbot.state_machine import AutonomousStateMachine, state
from wpimath import trajectory, geometry, controller, kinematics

from components.chassis import Chassis


class AutoBase(AutonomousStateMachine):

    MODE_NAME = "Drive Backward"
    DEFAULT = True

    chassis: Chassis

    def __init__(self):
        super().__init__()
        self.max_accel = 0.2
        self.max_vel = 0.5
        constraints = trajectory.TrapezoidProfileRadians.Constraints(
            self.max_vel, self.max_accel
        )
        self.drive_controller = controller.HolonomicDriveController(
            controller.PIDController(0, 0, 0),
            controller.PIDController(0, 0, 0),
            controller.ProfiledPIDControllerRadians(0, 0, 0, constraints),
        )
        self.config = trajectory.TrajectoryConfig(self.max_vel, self.max_accel)
        self.gen = trajectory.TrajectoryGenerator()
        self.chassis_speeds = kinematics.ChassisSpeeds(0, 0, 0)

    def setup(self):
        self.config.setKinematics(self.chassis.kinematics)

        self.start_position = geometry.Pose2d(
            geometry.Translation2d(0, 0), geometry.Rotation2d(0)
        )
        self.end_position = geometry.Pose2d(
            geometry.Translation2d(1, 1), geometry.Rotation2d()
        )
        self.target_trajectory = self.gen.generateTrajectory(
            self.start_position,
            [geometry.Translation2d(1, -0.1)],
            self.end_position,
            self.config,
        )
        print("total time: ", self.target_trajectory.totalTime())
        self.next_state("move")

    @state(first=True)
    def move(self, state_tm):
        self.chassis_speeds = self.drive_controller.calculate(
            self.chassis.odometry.getPose(),
            self.target_trajectory.sample(state_tm),
            self.end_position.rotation(),
        )

        self.chassis.drive_local(
            self.chassis_speeds.vx, self.chassis_speeds.vy, self.chassis_speeds.omega
        )

        # if state_tm > self.target_trajectory.totalTime:
        #     self.

    # @magicbot.feedback
    # def return_chassis(self):
    #     return f"x:{self.chassis_speeds.vx}, y:{self.chassis_speeds.vy}, a: {self.chassis_speeds.omega}"

    def shoot(self):
        pass
