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
from components.vision import Vision

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
    vision: Vision

    def createObjects(self) -> None:
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
        self.codriver = wpilib.XboxController(0)

        self.shooter_left_motor = ctre.TalonFX(11)
        self.shooter_right_motor = ctre.TalonFX(10)

        self.turret_motor = ctre.TalonSRX(15)
        self.turret_absolute_encoder = wpilib.DutyCycleEncoder(0)
        self.turret_cable_piston = wpilib.Solenoid(wpilib.PneumaticsModuleType.CTREPCM, 4)

        self.intake_motor = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.intake_piston = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM, 0, 2
        )  # TODO check port numbers
        self.indexer_tunnel_motor = rev.CANSparkMax(
            2, rev.CANSparkMax.MotorType.kBrushless
        )
        self.indexer_chimney_motor = rev.CANSparkMax(
            3, rev.CANSparkMax.MotorType.kBrushless
        )
        self.upper_chimney_prox_sensor = wpilib.DigitalInput(8)
        self.lower_chimney_prox_sensor = wpilib.DigitalInput(6)
        self.tunnel_break_beam = wpilib.DigitalInput(1)
        self.cat_flap_piston = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM, 1, 3
        )  # TODO correct port numbers
        self.colour_sensor = rev.ColorSensorV3(wpilib.I2C.Port.kMXP)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.chassis_1_encoder = TalonEncoder(ctre.TalonSRX(13), unitsPerRev=math.tau)
        self.chassis_2_encoder = TalonEncoder(ctre.TalonSRX(20), unitsPerRev=math.tau)
        self.chassis_3_encoder = TalonEncoder(ctre.TalonSRX(18), unitsPerRev=math.tau)
        self.chassis_4_encoder = TalonEncoder(ctre.TalonSRX(14), unitsPerRev=math.tau)

        self.auto_shoot = False

    def autonomousInit(self) -> None:
        self.shooter_control.lead_shots = False
        self.intake.auto_retract = False
        self.auto_shoot = False

    def teleopInit(self) -> None:
        self.intake.auto_retract = True
        self.shooter_control.lead_shots = True
        self.indexer_control.ignore_colour = False
        self.auto_shoot = False

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
        self.turret.update_angle_history()
        self.chassis.update_pose_history()
        self.turret.try_sync()
        self.vision.execute()

    def teleopPeriodic(self) -> None:
        # handle chassis inputs
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)
        spin_rate = 2.0
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
            -rescale_js(self.joystick.getZ(), deadzone=0.3, exponential=25.0)
            * spin_rate
        )

        # if joystick_x or joystick_y or joystick_z:
        # Drive in field oriented mode unless button 6 is held
        if not self.joystick.getRawButton(6):
            self.chassis.drive_field(joystick_x, joystick_y, joystick_z)
        else:
            self.chassis.drive_local(joystick_x, joystick_y, joystick_z)

        if self.joystick.getRawButtonPressed(11):
            self.auto_shoot = True

        if self.joystick.getRawButtonPressed(12):
            self.auto_shoot = False

        # reset heading to intake facing directly downfield
        if self.joystick.getRawButtonPressed(9):
            self.chassis.zero_yaw()

        if self.joystick.getTrigger() or self.auto_shoot:
            self.shooter_control.fire()

        if self.joystick.getRawButtonPressed(2):
            if self.intake.deployed:
                self.intake.deployed = False
                if self.indexer_control.current_state == "intaking":
                    self.indexer_control.stop()
            elif self.indexer.ready_to_intake():
                self.indexer_control.wants_to_intake = True
                self.intake.deployed = True

    def testPeriodic(self) -> None:
        # hold y and use joystick throttle to set flywheel speed
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0, 1)
        self.shooter.motor_speed = throttle * 60

        # hold x and use left stick to slew turret
        if self.joystick.getPOV() != -1:
            slew_x = math.sin(math.radians(self.joystick.getPOV(0))) * 25
            self.turret.slew_relative(slew_x)

        # joystick trigger to force fire
        if self.joystick.getTrigger():
            self.indexer.run_chimney_motor(Indexer.Direction.FORWARDS)

        # indexer same as teleop
        if self.joystick.getRawButtonPressed(2):
            if self.intake.deployed:
                self.intake.deployed = False
                if self.indexer_control.current_state == "intaking":
                    self.indexer_control.stop()
            elif self.indexer.ready_to_intake():
                self.indexer_control.wants_to_intake = True
                self.intake.deployed = True

        self.indexer_control.execute()

        self.chassis.execute()
        self.hanger.execute()
        self.intake.execute()
        self.indexer.execute()
        self.shooter.execute()
        self.turret.execute()
        self.vision.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
