#!/usr/bin/env python3

import wpilib
import magicbot
import ctre
import navx
import rev
import math

from components.chassis import Chassis
from components.hanger import Hanger
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.turret import Turret
from controllers.indexer import IndexerController
from components.target_estimator import TargetEstimator

from controllers.shooter import ShooterController
from utilities.scalers import rescale_js, scale_value
from utilities.ctre import TalonEncoder

from utilities import git

GIT_INFO = git.describe()


class MyRobot(magicbot.MagicRobot):
    shooter_control: ShooterController
    indexer_control: IndexerController

    chassis: Chassis
    hanger: Hanger
    intake: Intake
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    target_estimator: TargetEstimator

    def createObjects(self):
        self.logger.info("pyrapidreact %s", GIT_INFO)

        self.imu = navx.AHRS.create_spi()

        self.chassis_1_drive = ctre.TalonFX(1)
        self.chassis_1_steer = ctre.TalonFX(2)
        self.chassis_2_drive = ctre.TalonFX(3)
        self.chassis_2_steer = ctre.TalonFX(4)
        self.chassis_3_drive = ctre.TalonFX(5)
        self.chassis_3_steer = ctre.TalonFX(6)
        self.chassis_4_drive = ctre.TalonFX(7)
        self.chassis_4_steer = ctre.TalonFX(8)

        self.joystick = wpilib.Joystick(0)

        self.shooter_left_motor = ctre.TalonFX(11)
        self.shooter_right_motor = ctre.TalonFX(10)
        self.turret_motor = ctre.TalonSRX(10)

        self.turret_motor = ctre.TalonSRX(15)
        self.turret_absolute_encoder = wpilib.DutyCycleEncoder(0)

        self.intake_motor = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        # self.intake_piston = wpilib.Solenoid(0, )
        self.indexer_front_motor = rev.CANSparkMax(
            2, rev.CANSparkMax.MotorType.kBrushless
        )
        self.indexer_back_motor = rev.CANSparkMax(
            3, rev.CANSparkMax.MotorType.kBrushless
        )
        self.indexer_prox_sensor2 = wpilib.DigitalInput(8)
        self.indexer_prox_sensor1 = wpilib.DigitalInput(6)
        self.colour_sensor = rev.ColorSensorV3(wpilib.I2C.Port.kMXP)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.chassis_1_encoder = TalonEncoder(ctre.TalonSRX(13), unitsPerRev=math.tau)
        self.chassis_2_encoder = TalonEncoder(ctre.TalonSRX(20), unitsPerRev=math.tau)
        self.chassis_3_encoder = TalonEncoder(ctre.TalonSRX(18), unitsPerRev=math.tau)
        self.chassis_4_encoder = TalonEncoder(ctre.TalonSRX(14), unitsPerRev=math.tau)

    def autonomousInit(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def testInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        wpilib.SmartDashboard.putNumberArray(
            "swerve_encoder_adjusted",
            [module.get_angle() for module in self.chassis.modules],
        )
        # absolute encoder readings without offset
        wpilib.SmartDashboard.putNumberArray(
            "swerve_relative_encoder",
            [module.get_motor_angle() for module in self.chassis.modules],
        )

        self.turret.try_sync()

    def teleopPeriodic(self) -> None:
        # handle chassis inputs
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)
        spin_rate = 1.5
        joystick_x = (
            -rescale_js(self.joystick.getY(), deadzone=0.1, exponential=1.5)
            * 4
            * throttle
        )
        joystick_y = (
            -rescale_js(self.joystick.getX(), deadzone=0.1, exponential=1.5)
            * 4
            * throttle
        )
        joystick_z = (
            -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=20.0)
            * spin_rate
        )

        # if joystick_x or joystick_y or joystick_z:
        # Drive in field oriented mode unless button 6 is held
        if not self.joystick.getRawButton(6):
            self.chassis.drive_field(joystick_x, joystick_y, joystick_z)
        else:
            self.chassis.drive_local(joystick_x, joystick_y, joystick_z)

        # Reset the heading when button 7 is pressed
        if self.joystick.getRawButtonPressed(7):
            self.imu.reset()

        if self.joystick.getTriggerPressed():
            self.indexer_control.wants_to_fire = True

        if self.joystick.getRawButtonPressed(2):
            self.indexer_control.wants_to_intake = (
                not self.indexer_control.wants_to_intake
            )

        # manually clear ball
        if self.joystick.getRawButtonPressed(11):
            self.shooter_control.clear_input()

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
