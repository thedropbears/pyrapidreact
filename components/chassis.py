from collections import deque
from logging import Logger
import math
from typing import Optional, Deque

import ctre
import magicbot
import navx
import wpilib

from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    ChassisSpeeds,
    SwerveModuleState,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d
from wpimath.estimator import SwerveDrive4PoseEstimator

from utilities.functions import constrain_angle
from utilities.ctre import TalonEncoder
from wpimath.controller import SimpleMotorFeedforwardMeters
from utilities.trajectory_generator import goal_to_field


class SwerveModule:
    DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
    STEER_GEAR_RATIO = (14 / 50) * (10 / 60)
    WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi

    METRES_TO_DRIVE_UNITS = 2048 / WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO
    DRIVE_SENSOR_TO_METRES = 1 / METRES_TO_DRIVE_UNITS

    STEER_SENSOR_TO_RAD = math.tau / 2048 * STEER_GEAR_RATIO
    STEER_RAD_TO_SENSOR = 1 / STEER_SENSOR_TO_RAD

    SLEW_CRUISE_VELOCITY = 400  # rpm
    CRUISE_ACCELERATION = 200  # rpm/s

    def __init__(
        self,
        x: float,
        y: float,
        drive: ctre.TalonFX,
        steer: ctre.TalonFX,
        encoder: TalonEncoder,
        encoder_offset: float = 0,
        steer_reversed=True,
        drive_reversed=False,
    ):
        self.translation = Translation2d(x, y)
        self.steer = steer
        self.steer.configFactoryDefault()
        self.steer.setNeutralMode(ctre.NeutralMode.Brake)
        self.steer.setInverted(steer_reversed)

        self.steer.config_kP(0, 0.15035, 10)
        self.steer.config_kI(0, 0, 10)
        self.steer.config_kD(0, 5.6805, 10)
        self.steer.configAllowableClosedloopError(
            0, self.STEER_RAD_TO_SENSOR * math.radians(3)
        )
        self.steer.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )

        self.encoder = encoder
        self.encoder_offset = constrain_angle(encoder_offset)

        self.drive = drive
        self.drive.configFactoryDefault()
        self.drive.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive.setInverted(drive_reversed)
        self.drive.configVoltageCompSaturation(12, timeoutMs=10)
        self.drive.enableVoltageCompensation(True)
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.65599, kV=2.8309, kA=0.15238)

        self.drive.config_kP(0, 0.00064721, 10)
        self.drive.config_kI(0, 0, 10)
        self.drive.config_kD(0, 0, 10)

        # Reduce CAN status frame rates
        steer.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_1_General, 250, 10)
        drive.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_1_General, 250, 10)
        encoder.talon.setStatusFramePeriod(
            ctre.StatusFrameEnhanced.Status_1_General, 250, 10
        )

    def get_angle(self) -> float:
        """Gets steer angle from absolute encoder"""
        return self.encoder.getPosition() + self.encoder_offset

    def get_motor_angle(self) -> float:
        """Gets steer angle from motor's integrated relative encoder"""
        return self.steer.getSelectedSensorPosition() * self.STEER_SENSOR_TO_RAD

    def get_rotation(self) -> Rotation2d:
        """Get the angle of the module."""
        return Rotation2d(self.get_motor_angle())

    def get_speed(self):
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_SENSOR_TO_METRES * 10

    def set(self, desired_state: SwerveModuleState):

        if abs(desired_state.speed) < 1e-3:
            self.drive.set(ctre.ControlMode.Velocity, 0)
            self.steer.set(ctre.ControlMode.Velocity, 0)
            return

        current_angle = self.get_angle()
        target_displacement = constrain_angle(
            desired_state.angle.radians() - current_angle
        )
        target_angle = target_displacement + current_angle
        self.steer.set(
            ctre.ControlMode.Position, target_angle * self.STEER_RAD_TO_SENSOR
        )

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = desired_state.speed * math.cos(target_displacement) ** 2
        speed_volt = self.drive_ff.calculate(target_speed)
        self.drive.set(
            ctre.ControlMode.Velocity,
            target_speed * self.METRES_TO_DRIVE_UNITS / 10,
            ctre.DemandType.ArbitraryFeedForward,
            speed_volt / 12,
        )

    def sync_steer_encoders(self):
        self.steer.setSelectedSensorPosition(
            self.get_angle() * self.STEER_RAD_TO_SENSOR
        )

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class Chassis:
    # assumes square chassis
    width = 0.6167  # meters between modules from CAD

    vx = magicbot.will_reset_to(0.0)
    vy = magicbot.will_reset_to(0.0)
    vz = magicbot.will_reset_to(0.0)

    chassis_1_drive: ctre.TalonFX
    chassis_1_steer: ctre.TalonFX
    chassis_2_drive: ctre.TalonFX
    chassis_2_steer: ctre.TalonFX
    chassis_3_drive: ctre.TalonFX
    chassis_3_steer: ctre.TalonFX
    chassis_4_drive: ctre.TalonFX
    chassis_4_steer: ctre.TalonFX

    chassis_1_encoder: TalonEncoder
    chassis_2_encoder: TalonEncoder
    chassis_3_encoder: TalonEncoder
    chassis_4_encoder: TalonEncoder

    imu: navx.AHRS

    control_loop_wait_time: float

    debug_steer_pos = magicbot.tunable(0)

    desired_states = None
    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))
    field: wpilib.Field2d
    logger: Logger

    def __init__(self):
        self.pose_history: Deque[Pose2d] = deque([], maxlen=100)
        self.translation_velocity = Translation2d()
        self.rotation_velocity = Rotation2d()

    def setup(self):
        # mag encoder only
        self.chassis_1_encoder.talon.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10
        )
        self.chassis_2_encoder.talon.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10
        )
        self.chassis_3_encoder.talon.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10
        )
        self.chassis_4_encoder.talon.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10
        )

        self.modules = [
            SwerveModule(
                self.width / 2,
                self.width / 2,
                self.chassis_1_drive,
                self.chassis_1_steer,
                self.chassis_1_encoder,
                encoder_offset=-4.612,
            ),
            SwerveModule(
                -self.width / 2,
                self.width / 2,
                self.chassis_2_drive,
                self.chassis_2_steer,
                self.chassis_2_encoder,
                encoder_offset=-1.725,
            ),
            SwerveModule(
                -self.width / 2,
                -self.width / 2,
                self.chassis_3_drive,
                self.chassis_3_steer,
                self.chassis_3_encoder,
                encoder_offset=-5.051,
            ),
            SwerveModule(
                self.width / 2,
                -self.width / 2,
                self.chassis_4_drive,
                self.chassis_4_steer,
                self.chassis_4_encoder,
                encoder_offset=-0.854,
            ),
        ]

        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].translation,
            self.modules[1].translation,
            self.modules[2].translation,
            self.modules[3].translation,
        )
        self.sync_all()
        self.imu.zeroYaw()
        self.estimator = SwerveDrive4PoseEstimator(
            self.imu.getRotation2d(),
            Pose2d(0, 0, 0),
            self.kinematics,
            stateStdDevs=(0.05, 0.05, math.radians(5)),
            localMeasurementStdDevs=(0.01,),
            visionMeasurementStdDevs=(0.5, 0.5, 0.2),
        )
        self.set_pose(Pose2d(-0.711, -2.419, Rotation2d.fromDegrees(-88.5)))

        self.field_obj = self.field.getObject("estimator_pose")

        self.control_rate = 1 / self.control_loop_wait_time

    def drive_field(self, x, y, z):
        """Field oriented drive commands"""
        rotation = self.imu.getRotation2d()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, rotation)

    def drive_local(self, x, y, z):
        """Field oriented drive commands"""
        self.chassis_speeds = ChassisSpeeds(x, y, z)

    def execute(self):
        self.desired_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        for state, module in zip(self.desired_states, self.modules):
            state = SwerveModuleState.optimize(state, module.get_rotation())
            module.set(state)
        self.kinematics.desaturateWheelSpeeds
        self.estimator.update(
            self.imu.getRotation2d(),
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )

        self.translation_velocity = (
            self.estimator.getEstimatedPosition().translation()
            - self.pose_history[0].translation()
        ) * self.control_rate
        self.rotation_velocity = (
            self.estimator.getEstimatedPosition().rotation()
            - self.pose_history[0].rotation()
        ) * self.control_rate

        self.pose_history.appendleft(self.estimator.getEstimatedPosition())
        self.field_obj.setPose(goal_to_field(self.pose_history[0]))

    @magicbot.feedback
    def get_imu_rotation(self):
        return self.imu.getRotation2d().radians()

    def sync_all(self):
        for m in self.modules:
            m.sync_steer_encoders()

    def set_pose(self, pose: Pose2d) -> None:
        self.pose_history = deque([pose], maxlen=100)
        self.estimator.resetPosition(pose, self.imu.getRotation2d())

    def get_pose_at(self, t: float) -> Pose2d:
        """Gets where the robot was at t"""
        loops_ago = int((wpilib.Timer.getFPGATimestamp() - t) * self.control_rate)
        if loops_ago < 0:
            self.logger.warning("vision clocks warpped")
            return self.estimator.getEstimatedPosition()
        if loops_ago >= len(self.pose_history):
            return (
                self.pose_history[-1]
                if len(self.pose_history) > 0
                else self.estimator.getEstimatedPosition()
            )
        return self.pose_history[loops_ago]

    def robot_to_world(
        self, offset: Translation2d, robot: Optional[Pose2d] = None
    ) -> Pose2d:
        """Transforms a translation from robot space to world space (e.g. turret position)"""
        if robot is None:
            robot = self.estimator.getEstimatedPosition()
        return Pose2d(
            robot.translation() + offset.rotateBy(robot.rotation()),
            robot.rotation(),
        )
