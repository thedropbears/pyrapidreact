from logging import Logger
import math
import time
from typing import Optional

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
from wpimath.interpolation import TimeInterpolatablePose2dBuffer

from utilities.functions import constrain_angle
from utilities.ctre import FALCON_CPR
from wpimath.controller import SimpleMotorFeedforwardMeters


class SwerveModule:
    DRIVE_GEAR_RATIO = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0)
    STEER_GEAR_RATIO = (14 / 50) * (10 / 60)
    WHEEL_CIRCUMFERENCE = 4 * 2.54 / 100 * math.pi

    DRIVE_MOTOR_REV_TO_METRES = WHEEL_CIRCUMFERENCE * DRIVE_GEAR_RATIO
    METRES_TO_DRIVE_COUNTS = FALCON_CPR / DRIVE_MOTOR_REV_TO_METRES
    DRIVE_COUNTS_TO_METRES = DRIVE_MOTOR_REV_TO_METRES / FALCON_CPR

    STEER_MOTOR_REV_TO_RAD = math.tau * STEER_GEAR_RATIO
    STEER_COUNTS_TO_RAD = STEER_MOTOR_REV_TO_RAD / FALCON_CPR
    STEER_RAD_TO_COUNTS = FALCON_CPR / STEER_MOTOR_REV_TO_RAD

    def __init__(
        self,
        x: float,
        y: float,
        drive: ctre.TalonFX,
        steer: ctre.TalonFX,
        encoder: ctre.CANCoder,
        steer_reversed=True,
        drive_reversed=False,
    ):
        self.translation = Translation2d(x, y)
        self.steer = steer
        self.drive = drive

        # Reduce CAN status frame rates before configuring
        steer.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_1_General, 250, 10)
        drive.setStatusFramePeriod(ctre.StatusFrameEnhanced.Status_1_General, 250, 10)

        self.steer.setNeutralMode(ctre.NeutralMode.Brake)
        self.steer.setInverted(steer_reversed)

        self.steer.config_kP(0, 0.15035, 10)
        self.steer.config_kI(0, 0, 10)
        self.steer.config_kD(0, 5.6805, 10)
        self.steer.configAllowableClosedloopError(
            0, self.STEER_RAD_TO_COUNTS * math.radians(3)
        )
        self.steer.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )

        self.encoder = encoder
        encoder.configFeedbackCoefficient(
            math.tau / 4096,
            "rad",
            ctre.SensorTimeBase.PerSecond,
            timeoutMs=10,
        )

        self.drive.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive.setInverted(drive_reversed)
        self.drive.configVoltageCompSaturation(12, timeoutMs=10)
        self.drive.enableVoltageCompensation(True)
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.18877, kV=2.7713, kA=0.18824)
        self.drive.configVelocityMeasurementPeriod(
            ctre.SensorVelocityMeasPeriod.Period_5Ms
        )
        self.drive.configVelocityMeasurementWindow(8)

        self.drive.config_kP(0, 0.011489, 10)
        self.drive.config_kI(0, 0, 10)
        self.drive.config_kD(0, 0, 10)

    def get_absolute_angle(self) -> float:
        """Gets steer angle from absolute encoder"""
        return self.encoder.getAbsolutePosition()

    def get_unconstrained_angle(self) -> float:
        """Gets steer angle from motor's integrated relative encoder"""
        return self.steer.getSelectedSensorPosition() * self.STEER_COUNTS_TO_RAD

    def get_rotation(self) -> Rotation2d:
        """Get the angle of the module."""
        return Rotation2d(self.get_unconstrained_angle())

    def get_speed(self) -> float:
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_COUNTS_TO_METRES * 10

    def set(self, desired_state: SwerveModuleState):

        if abs(desired_state.speed) < 1e-3:
            self.drive.set(ctre.ControlMode.Velocity, 0)
            self.steer.set(ctre.ControlMode.Velocity, 0)
            return

        current_angle = self.get_unconstrained_angle()
        target_displacement = constrain_angle(
            desired_state.angle.radians() - current_angle
        )
        target_angle = target_displacement + current_angle
        self.steer.set(
            ctre.ControlMode.Position, target_angle * self.STEER_RAD_TO_COUNTS
        )

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = desired_state.speed * math.cos(target_displacement) ** 2
        speed_volt = self.drive_ff.calculate(target_speed)
        self.drive.set(
            ctre.ControlMode.Velocity,
            target_speed * self.METRES_TO_DRIVE_COUNTS / 10,
            ctre.DemandType.ArbitraryFeedForward,
            speed_volt / 12,
        )

    def sync_steer_encoders(self) -> None:
        self.steer.setSelectedSensorPosition(
            self.get_absolute_angle() * self.STEER_RAD_TO_COUNTS
        )

    def get(self) -> SwerveModuleState:
        return SwerveModuleState(self.get_speed(), self.get_rotation())


class Chassis:
    # assumes square chassis
    width = 0.6167  # meters between modules from CAD
    # maxiumum speed for any wheel
    max_attainable_wheel_speed = 13 * 0.3048  # ft/s to m/s

    vx = magicbot.will_reset_to(0.0)
    vy = magicbot.will_reset_to(0.0)
    vz = magicbot.will_reset_to(0.0)

    chassis_1_drive: ctre.WPI_TalonFX
    chassis_1_steer: ctre.WPI_TalonFX
    chassis_2_drive: ctre.WPI_TalonFX
    chassis_2_steer: ctre.WPI_TalonFX
    chassis_3_drive: ctre.WPI_TalonFX
    chassis_3_steer: ctre.WPI_TalonFX
    chassis_4_drive: ctre.WPI_TalonFX
    chassis_4_steer: ctre.WPI_TalonFX

    chassis_1_encoder: ctre.CANCoder
    chassis_2_encoder: ctre.CANCoder
    chassis_3_encoder: ctre.CANCoder
    chassis_4_encoder: ctre.CANCoder

    imu: navx.AHRS

    control_loop_wait_time: float

    debug_steer_pos = magicbot.tunable(0)

    desired_states = None
    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))
    field: wpilib.Field2d
    logger: Logger

    vel_avg_alpha = 0.2

    # failsafe assumes it is touching the launchpad closest to the goal
    FAILSAFE_POSE = Pose2d(-4.502, 1.492, 0)

    def __init__(self) -> None:
        self.pose_history = TimeInterpolatablePose2dBuffer(2)
        self.last_pose = Pose2d()
        self.translation_velocity = Translation2d()
        self.rotation_velocity = Rotation2d()
        self.last_time = time.monotonic()

    def setup(self) -> None:
        self.modules = [
            SwerveModule(
                self.width / 2,
                self.width / 2,
                self.chassis_1_drive,
                self.chassis_1_steer,
                self.chassis_1_encoder,
            ),
            SwerveModule(
                -self.width / 2,
                self.width / 2,
                self.chassis_2_drive,
                self.chassis_2_steer,
                self.chassis_2_encoder,
            ),
            SwerveModule(
                -self.width / 2,
                -self.width / 2,
                self.chassis_3_drive,
                self.chassis_3_steer,
                self.chassis_3_encoder,
                drive_reversed=True,
            ),
            SwerveModule(
                self.width / 2,
                -self.width / 2,
                self.chassis_4_drive,
                self.chassis_4_steer,
                self.chassis_4_encoder,
                drive_reversed=True,
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
            stateStdDevs=(0.1, 0.1, math.radians(5)),
            localMeasurementStdDevs=(0.01,),
            visionMeasurementStdDevs=(0.5, 0.5, 0.2),
        )
        self.field_obj = self.field.getObject("fused_pose")
        self.set_pose(Pose2d(-2, 0, Rotation2d.fromDegrees(180)))

    def drive_field(self, vx: float, vy: float, omega: float) -> None:
        """Field oriented drive commands"""
        current_heading = self.get_rotation()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, omega, current_heading
        )

    def drive_local(self, vx: float, vy: float, omega: float) -> None:
        """Robot oriented drive commands"""
        self.chassis_speeds = ChassisSpeeds(vx, vy, omega)

    def execute(self) -> None:
        self.desired_states = self.kinematics.toSwerveModuleStates(self.chassis_speeds)
        self.desired_states = self.kinematics.desaturateWheelSpeeds(
            self.desired_states, attainableMaxSpeed=self.max_attainable_wheel_speed
        )
        for state, module in zip(self.desired_states, self.modules):
            state = SwerveModuleState.optimize(state, module.get_rotation())
            module.set(state)

        self.update_odometry()

        # add to prevent division by 0
        dt = min(time.monotonic() - self.last_time, 0.1) + 1e-10
        # rotation2d and translation2d have mul but not div
        control_rate = 1 / dt
        chassis_speeds = self.kinematics.toChassisSpeeds(
            (
                self.modules[0].get(),
                self.modules[1].get(),
                self.modules[2].get(),
                self.modules[3].get(),
            )
        )
        cur_trans_vel = Translation2d(chassis_speeds.vx, chassis_speeds.vy).rotateBy(
            self.get_rotation()
        )
        self.translation_velocity = (
            cur_trans_vel * self.vel_avg_alpha
            + self.translation_velocity * (1 - self.vel_avg_alpha)
        )
        cur_rot_vel = (
            self.estimator.getEstimatedPosition().rotation() - self.last_pose.rotation()
        ) * control_rate
        self.rotation_velocity = (
            cur_rot_vel * self.vel_avg_alpha
            + self.rotation_velocity * (1 - self.vel_avg_alpha)
        )

        self.update_pose_history()
        self.last_time = time.monotonic()

    def on_enable(self) -> None:
        self.reset_velocity()

    def sync_all(self) -> None:
        for m in self.modules:
            m.sync_steer_encoders()

    def set_pose(self, pose: Pose2d) -> None:
        self.pose_history.clear()
        self.estimator.resetPosition(pose, self.imu.getRotation2d())
        self.update_pose_history()
        self.field.setRobotPose(pose)
        self.field_obj.setPose(pose)
        self.reset_velocity()

    def reset_velocity(self):
        self.translation_velocity = Translation2d(0, 0)
        self.rotation_velocity = Rotation2d(0)
        self.last_time = time.monotonic()

    def set_pose_failsafe(self):
        """Sets the pose to the right side of hanger"""
        self.set_pose(self.FAILSAFE_POSE)

    def zero_yaw(self) -> None:
        """Sets pose to current pose but with a heading of zero"""
        cur_pose = self.estimator.getEstimatedPosition()
        # reset position says to zero encoders distances but i think this is
        # a misake copied from diff drive pose estimator
        # beacuse we never pass the encoder distances to the estimator
        self.estimator.resetPosition(
            Pose2d(cur_pose.translation(), Rotation2d(0)), self.imu.getRotation2d()
        )

    def get_pose(self) -> Pose2d:
        """Get the current location of the robot relative to the goal."""
        return self.estimator.getEstimatedPosition()

    def get_rotation(self) -> Rotation2d:
        """Get the current heading of the robot."""
        return self.get_pose().rotation()

    def get_pose_at(self, t: float) -> Pose2d:
        """Gets where the robot was at t"""
        return self.pose_history.sample(t)

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

    def update_odometry(self) -> None:
        self.estimator.update(
            self.imu.getRotation2d(),
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )
        self.field_obj.setPose(self.get_pose())

    def update_pose_history(self) -> None:
        pose = self.get_pose()
        self.pose_history.addSample(wpilib.Timer.getFPGATimestamp(), pose)
        self.last_pose = pose
