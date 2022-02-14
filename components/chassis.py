import math

import ctre
import wpilib
import magicbot
import navx

from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Odometry,
)
from wpimath.geometry import Translation2d, Rotation2d, Pose2d

from utilities.functions import constrain_angle
from utilities.ctre import TalonEncoder
from wpimath.controller import SimpleMotorFeedforwardMeters


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
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.65599, kV=2.8309, kA=0.15238)

        self.drive.config_kP(0, 0.00064721, 10)
        self.drive.config_kI(0, 0, 10)
        self.drive.config_kD(0, 0, 10)

        self.target_angle = 0

        self.last_speed = 0

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
            if abs(self.last_speed) > 1e-13:
                self.drive.setIntegralAccumulator(0, 0, 0)
                self.drive.set(ctre.ControlMode.Velocity, 0)
                self.steer.set(ctre.ControlMode.Velocity, 0)
                self.last_speed = 0
            return

        self.last_speed = desired_state.speed

        current_angle = self.get_angle()
        target_displacement = constrain_angle(
            desired_state.angle.radians() - current_angle
        )
        self.target_angle = target_displacement + current_angle
        self.steer.set(
            ctre.ControlMode.Position, self.target_angle * self.STEER_RAD_TO_SENSOR
        )

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = desired_state.speed * math.cos(target_displacement) ** 2
        speed_volt = self.drive_ff.calculate(target_speed)
        voltage = wpilib.RobotController.getInputVoltage()
        self.drive.set(
            ctre.ControlMode.Velocity,
            target_speed * self.METRES_TO_DRIVE_UNITS / 10,
            ctre.DemandType.ArbitraryFeedForward,
            speed_volt / voltage,
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

    debug_steer_pos = magicbot.tunable(0)

    desired_states = None

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))

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
        self.odometry = SwerveDrive4Odometry(self.kinematics, self.imu.getRotation2d())
        self.set_odometry(Pose2d(Translation2d(0, 0), Rotation2d(0)))

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

        wpilib.SmartDashboard.putNumber(
            "drive_1_target",
            self.modules[0].get_speed(),
        )
        wpilib.SmartDashboard.putNumber(
            "drive_1_actual",
            self.desired_states[0].speed,
        )

        self.odometry.update(
            self.imu.getRotation2d(),
            self.modules[0].get(),
            self.modules[1].get(),
            self.modules[2].get(),
            self.modules[3].get(),
        )

    @magicbot.feedback
    def get_imu_rotation(self):
        return self.imu.getRotation2d().radians()

    def sync_all(self):
        for m in self.modules:
            m.sync_steer_encoders()

    def set_odometry(self, pose: Pose2d) -> None:
        self.odometry.resetPosition(pose, self.imu.getRotation2d())
