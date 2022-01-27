import math

import ctre
import wpilib
from rev import CANSparkMax, MotorType, ControlType
from wpimath.kinematics import SwerveDrive2Kinematics, ChassisSpeeds, SwerveModuleState
from wpimath.geometry import Translation2d, Rotation2d

from utilities.scalers import rescale_js, scale_value
from utilities.functions import constrain_angle
from wpimath.controller import SimpleMotorFeedforwardMeters


class SwerveModule:
    DRIVE_GEAR_RATIO = 16 / 60
    STEER_GEAR_RATIO = 1 / 60

    DRIVE_SENSOR_TO_METRES = DRIVE_GEAR_RATIO / 2048
    METRES_TO_DRIVE_UNITS = 2048 / DRIVE_GEAR_RATIO

    def __init__(
        self,
        x: float,
        y: float,
        steer: CANSparkMax,
        drive: ctre.WPI_TalonFX,
        steer_reversed=False,
        drive_reversed=False,
    ):
        self.translation = Translation2d(x, y)

        self.steer = steer
        self.steer.setInverted(steer_reversed)
        self.steer.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.steer_reversed = steer_reversed
        self.encoder = self.steer.getAnalog()
        self.hall_effect = self.steer.getEncoder()
        # make the sensor's return value between 0 and 1
        self.encoder.setPositionConversionFactor(math.tau / 3.3)
        self.encoder.setInverted(steer_reversed)
        self.hall_effect.setPositionConversionFactor(self.STEER_GEAR_RATIO * math.tau)
        self.rezero_hall_effect()
        self.steer_pid = steer.getPIDController()
        self.steer_pid.setFeedbackDevice(self.hall_effect)
        self.steer_pid.setSmartMotionAllowedClosedLoopError(math.pi / 180)
        self.steer_pid.setP(1.85e-6)
        self.steer_pid.setI(0)
        self.steer_pid.setD(0)
        self.steer_pid.setFF(0.583 / 12 / math.tau * 60 * self.STEER_GEAR_RATIO)
        self.steer_pid.setSmartMotionMaxVelocity(400)  # RPM
        self.steer_pid.setSmartMotionMaxAccel(200)  # RPM/s

        self.drive = drive
        self.drive.setNeutralMode(ctre.NeutralMode.Brake)
        self.drive.setInverted(drive_reversed)
        self.drive_ff = SimpleMotorFeedforwardMeters(kS=0.757, kV=1.3, kA=0.0672)

        drive.config_kP(slotIdx=0, value=2.21e-31, timeoutMs=20)

    def get_angle(self) -> float:
        return self.hall_effect.getPosition()

    def get_enc_angle(self) -> float:
        return self.encoder.getPosition()

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
        self.steer_pid.setReference(target_angle, ControlType.kSmartMotion)
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
        self.hall_effect.setPosition(self.encoder.getPosition())


class Chassos:
    def setup(self):
        """Robot initialization function"""
        self.gyro = wpilib.ADXRS450_Gyro()
        self.gyro.reset()
        self.gyro.calibrate()

        self.modules = [
            SwerveModule(
                0.8 / 2 - 0.125,
                0.75 / 2 - 0.1,
                CANSparkMax(9, MotorType.kBrushless),
                ctre.WPI_TalonFX(3),
                steer_reversed=False,
                drive_reversed=True,
            ),
            SwerveModule(
                -0.8 / 2 + 0.125,
                -0.75 / 2 + 0.1,
                CANSparkMax(7, MotorType.kBrushless),
                ctre.WPI_TalonFX(5),
                steer_reversed=False,
                drive_reversed=True,
            ),
        ]

        self.kinematics = SwerveDrive2Kinematics(
            self.modules[0].translation, self.modules[1].translation
        )

        self.joystick = wpilib.Joystick(0)

        self.spin_rate = 1.5

    def teleopInit(self):
        """Executed at the start of teleop mode"""

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)

        # this is where the joystick inputs get converted to numbers that are sent
        # to the chassis component. we rescale them using the rescale_js function,
        # in order to make their response exponential, and to set a dead zone -
        # which just means if it is under a certain value a 0 will be sent
        # TODO: Tune these constants for whatever robot they are on
        joystick_vx = (
            -rescale_js(self.joystick.getY(), deadzone=0.1, exponential=1.5)
            * 4
            * throttle
        )
        joystick_vy = (
            -rescale_js(self.joystick.getX(), deadzone=0.1, exponential=1.5)
            * 4
            * throttle
        )
        joystick_vz = (
            -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=20.0)
            * self.spin_rate
        )

        if joystick_vx or joystick_vy or joystick_vz:
            # Drive in field oriented mode unless button 6 is held
            if not self.joystick.getRawButton(6):
                rotation = self.gyro.getRotation2d()
                chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    joystick_vx, joystick_vy, joystick_vz, rotation
                )
            else:
                chassis_speeds = ChassisSpeeds(joystick_vx, joystick_vy, joystick_vz)

            for state, module in zip(
                self.kinematics.toSwerveModuleStates(chassis_speeds), self.modules
            ):
                new_state = SwerveModuleState.optimize(state, module.get_rotation())
                module.set(new_state)
        else:
            for module in self.modules:
                module.stop()

        # Reset the heading when button 7 is pressed
        if self.joystick.getRawButtonPressed(7):
            self.gyro.reset()

    def robotPeriodic(self):
        wpilib.SmartDashboard.putNumberArray(
            "swerve_steer_pos", [module.get_angle() for module in self.modules]
        )
        wpilib.SmartDashboard.putNumberArray(
            "swerve_encoder_pos", [module.get_enc_angle() for module in self.modules]
        )
