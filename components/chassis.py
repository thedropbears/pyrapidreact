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

        self.drive = drive
        self.drive.configFactoryDefault()
        self.drive.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive.setInverted(drive_reversed)
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.52933, kV=2.7409, kA=0.056869)

        self.drive.config_kP(0, 0.00012288, 10)
        self.drive.config_kI(0, 0, 10)
        self.drive.config_kD(0, 0, 10)

        self.target_angle = 0

    def get_angle(self) -> float:
        # return self.hall_effect.getPosition()
        return self.get_motor_angle() * self.STEER_SENSOR_TO_RAD

    def get_motor_angle(self) -> float:
        return self.steer.getSelectedSensorPosition()

    def get_rotation(self) -> Rotation2d:
        return Rotation2d(self.get_angle())

    def get_speed(self):
        return self.drive.getSelectedSensorVelocity() * self.DRIVE_SENSOR_TO_METRES * 10

    def set(self, desired_state: SwerveModuleState):
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

    def zero(self):
        self.steer.setSelectedSensorPosition(0)


class Chassis:
    # assumes square chassis
    width = 0.6167  # meters between modules from CAD
    spin_rate = 1.5

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

    imu: navx.AHRS

    debug_steer_pos = magicbot.tunable(0)

    desired_states = None

    chassis_speeds = magicbot.will_reset_to(ChassisSpeeds(0, 0, 0))

    field: wpilib.Field2d

    def setup(self):
        self.modules = [
            SwerveModule(
                self.width / 2,
                self.width / 2,
                self.chassis_1_drive,
                self.chassis_1_steer,
            ),
            SwerveModule(
                -self.width / 2,
                self.width / 2,
                self.chassis_2_drive,
                self.chassis_2_steer,
            ),
            SwerveModule(
                -self.width / 2,
                -self.width / 2,
                self.chassis_3_drive,
                self.chassis_3_steer,
            ),
            SwerveModule(
                self.width / 2,
                -self.width / 2,
                self.chassis_4_drive,
                self.chassis_4_steer,
            ),
        ]

        self.kinematics = SwerveDrive4Kinematics(
            self.modules[0].translation,
            self.modules[1].translation,
            self.modules[2].translation,
            self.modules[3].translation,
        )
        self.zero_all()
        self.odometry = SwerveDrive4Odometry(self.kinematics, self.imu.getRotation2d())
        self.set_odometry(Pose2d(Translation2d(0, 0), Rotation2d(0)))

    def drive_field(self, x, y, z):
        """Field oriented drive commands"""
        rotation = self.imu.getRotation2d()
        self.chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            x, y, z * self.spin_rate, rotation
        )

    def drive_local(self, x, y, z):
        """Field oriented drive commands"""
        self.chassis_speeds = ChassisSpeeds(x, y, z * self.spin_rate)

    def _drive(self, chassis_speeds):
        self.desired_states = self.kinematics.toSwerveModuleStates(chassis_speeds)
        for state, module in zip(self.desired_states, self.modules):
            # new_state = SwerveModuleState.optimize(state, module.get_rotation())
            module.set(state)

    def execute(self):
        self._drive(self.chassis_speeds)
        wpilib.SmartDashboard.putNumberArray(
            "swerve_steer_pos_counts",
            [module.get_motor_angle() for module in self.modules],
        )
        self.odometry.update(
            self.imu.getRotation2d(),
            self.desired_states[0],
            self.desired_states[1],
            self.desired_states[2],
            self.desired_states[3],
        )

    @magicbot.feedback
    def get_imu_rotation(self):
        return self.imu.getRotation2d().radians()

    def zero_all(self):
        for m in self.modules:
            m.zero()

    def set_odometry(self, pose: Pose2d) -> None:
        self.odometry.resetPosition(pose, self.imu.getRotation2d())
