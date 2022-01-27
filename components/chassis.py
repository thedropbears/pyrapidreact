import math

import ctre
import wpilib
import magicbot

from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d

from utilities.functions import constrain_angle
from wpimath.controller import SimpleMotorFeedforwardMeters


class SwerveModule:
    DRIVE_GEAR_RATIO = 16 / 60
    STEER_GEAR_RATIO = 1 / 60

    DRIVE_SENSOR_TO_METRES = DRIVE_GEAR_RATIO / 2048
    METRES_TO_DRIVE_UNITS = 2048 / DRIVE_GEAR_RATIO

    SLEW_CRUISE_VELOCITY = 400  # rpm (according to falconswervetest)
    CRUISE_ACCELERATION = 200  # rpm/s

    def __init__(
        self,
        x: float,
        y: float,
        encoder: ctre.CANCoder,
        steer: ctre.TalonFX,
        drive: ctre.TalonFX,
        steer_reversed=False,
        drive_reversed=False,
    ):
        self.translation = Translation2d(x, y)

        self.encoder = encoder
        # set encoder inverted?
        self.steer = steer
        self.steer.configFactoryDefault()
        self.steer.setNeutralMode(ctre.NeutralMode.Brake)
        self.steer.setInverted(steer_reversed)

        self.steer.configNominalOutputForward(0, 10)
        self.steer.configNominalOutputReverse(0, 10)
        self.steer.configPeakOutputForward(1.0, 10)
        self.steer.configPeakOutputReverse(-1.0, 10)
        self.steer.config_kF(0, self.pidF, 10)  # ?
        self.steer.config_kP(0, self.pidP, 10)
        self.steer.config_kI(0, self.pidI, 10)
        self.steer.config_kD(0, self.pidD, 10)
        self.steer.configMotionCruiseVelocity(self.SLEW_CRUISE_VELOCITY, 10)
        self.steer.configMotionAcceleration(self.CRUISE_ACCELERATION, 10)
        self.steer.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.IntegratedSensor, 0, 10
        )

        self.drive = drive
        self.drive.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive.setInverted(drive_reversed)
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.757, kV=1.3, kA=0.0672)

        drive.config_kP(slotIdx=0, value=2.21e-31, timeoutMs=20)

    def get_angle(self) -> float:
        return self.hall_effect.getPosition()

    def get_motor_angle(self) -> float:
        return self.motor.getSelectedPosition()

    def get_rotation(self) -> Rotation2d:
        return Rotation2d(self.get_angle())

    def get_speed(self):
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_SENSOR_TO_METRES * 10

    def set(self, desired_state: SwerveModuleState):
        current_angle = self.get_angle()
        target_displacement = constrain_angle(
            desired_state.angle.radians() - current_angle
        )
        target_angle = target_displacement + current_angle
        self.steer.set(ctre.ControlMode.MotionMagic, target_angle)

        # rescale the speed target based on how close we are to being correctly aligned
        target_speed = desired_state.speed * math.cos(target_displacement) ** 4
        speed_volt = self.drive_ff.calculate(target_speed)
        voltage = wpilib.RobotController.getInputVoltage()
        self.drive.set(
            ctre.ControlMode.Velocity,
            target_speed * self.METRES_TO_DRIVE_UNITS / 10,
            ctre.DemandType.ArbitraryFeedForward,
            speed_volt / voltage,
        )

    def stop(self) -> None:
        self.drive.stopMotor()
        self.steer.stopMotor()

    def rezero_hall_effect(self):
        print(self.encoder.getPosition())
        self.steer.setSelectedSensorPosition(self.encoder.getAbsolutePosition())


class Chassis:
    # assumes square chassis
    width = 0.75  # meters between modules
    spin_rate = 1.5

    vx = magicbot.will_reset_to(0.0)
    vy = magicbot.will_reset_to(0.0)
    vz = magicbot.will_reset_to(0.0)

    NE_drive: ctre.TalonFX
    NE_steer: ctre.TalonFX
    SE_drive: ctre.TalonFX
    SE_steer: ctre.TalonFX
    SW_drive: ctre.TalonFX
    SW_steer: ctre.TalonFX
    NW_drive: ctre.TalonFX
    NW_steer: ctre.TalonFX

    chassis_NE_encoder: ctre.CANCoder
    chassis_SE_encoder: ctre.CANCoder
    chassis_SW_encoder: ctre.CANCoder
    chassis_NW_encoder: ctre.CANCoder

    gyro: wpilib.ADXRS450_Gyro

    def setup(self):
        self.modules = [
            SwerveModule(
                self.width / 2,
                self.width / 2,
                self.NE_drive,
                self.NE_steer,
            ),
            SwerveModule(
                self.width / 2,
                -self.width / 2,
                self.SE_drive,
                self.SE_steer,
            ),
            SwerveModule(
                -self.width / 2,
                -self.width / 2,
                self.SW_drive,
                self.SW_steer,
            ),
            SwerveModule(
                -self.width / 2,
                self.width / 2,
                self.NW_drive,
                self.NW_steer,
            ),
        ]

        self.kinematics = SwerveDrive4Kinematics(
            **map(lambda x: x.translation, self.modules)
        )

    def drive_field(self, x, y, z):
        """Field oriented drive commands"""
        rotation = self.gyro.getRotation2d()
        chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, z, rotation)
        self._drive(chassis_speeds)

    def drive_local(self, x, y, z):
        """Field oriented drive commands"""
        chassis_speeds = ChassisSpeeds(x, y, z)
        self._drive(chassis_speeds)

    def _drive(self, chassis_speeds):
        for state, module in zip(
            self.kinematics.toSwerveModuleStates(chassis_speeds), self.modules
        ):
            new_state = SwerveModuleState.optimize(state, module.get_rotation())
            module.set(new_state)

    def stop(self):
        for module in self.modules:
            module.stop()

    def execute(self):
        wpilib.SmartDashboard.putNumberArray(
            "swerve_steer_pos", [module.get_angle() for module in self.modules]
        )
        wpilib.SmartDashboard.putNumberArray(
            "swerve_encoder_pos", [module.get_enc_angle() for module in self.modules]
        )
