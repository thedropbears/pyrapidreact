#!/usr/bin/env python3

import wpilib
import magicbot
import ctre
import navx
import rev
import math

from components.leds import StatusLights
from components.chassis import Chassis
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.turret import Turret
from controllers.indexer import IndexerController
from controllers.leds import LedController
from components.vision import Vision

from controllers.shooter import ShooterController
from utilities.scalers import rescale_js, scale_value

from utilities import git

GIT_INFO = git.describe()


class MyRobot(magicbot.MagicRobot):
    led_control: LedController
    shooter_control: ShooterController
    indexer_control: IndexerController

    status_lights: StatusLights
    chassis: Chassis
    intake: Intake
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    vision: Vision

    lock_motion_while_shooting = magicbot.tunable(False)
    test_chassis_speed = magicbot.tunable(2)

    def createObjects(self) -> None:
        self.logger.info("pyrapidreact %s", GIT_INFO)
        self.data_log = wpilib.DataLogManager.getLog()

        self.imu = navx.AHRS.create_spi()

        self.leds = wpilib.AddressableLED(2)

        self.chassis_1_drive = ctre.WPI_TalonFX(1)
        self.chassis_1_steer = ctre.WPI_TalonFX(2)
        self.chassis_2_drive = ctre.WPI_TalonFX(3)
        self.chassis_2_steer = ctre.WPI_TalonFX(4)
        self.chassis_3_drive = ctre.WPI_TalonFX(5)
        self.chassis_3_steer = ctre.WPI_TalonFX(6)
        self.chassis_4_drive = ctre.WPI_TalonFX(7)
        self.chassis_4_steer = ctre.WPI_TalonFX(8)

        self.joystick = wpilib.Joystick(0)
        self.recorded_joystick_state = (0.0, 0.0, 0.0)
        self.recorded_is_local_driving = False
        self.codriver = wpilib.XboxController(1)

        self.shooter_left_motor = ctre.WPI_TalonFX(11)
        self.shooter_right_motor = ctre.WPI_TalonFX(10)

        self.turret_motor = ctre.WPI_TalonSRX(15)
        self.turret_absolute_encoder = wpilib.DutyCycleEncoder(10)  # navX pin 0
        self.turret_cable_piston = wpilib.Solenoid(
            wpilib.PneumaticsModuleType.CTREPCM, 4
        )

        self.intake_motor = rev.CANSparkMax(9, rev.CANSparkMax.MotorType.kBrushless)
        self.intake_piston = wpilib.DoubleSolenoid(
            wpilib.PneumaticsModuleType.CTREPCM, 0, 2
        )
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
        )
        self.colour_sensor = rev.ColorSensorV3(wpilib.I2C.Port.kMXP)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.chassis_1_encoder = ctre.CANCoder(1)
        self.chassis_2_encoder = ctre.CANCoder(2)
        self.chassis_3_encoder = ctre.CANCoder(3)
        self.chassis_4_encoder = ctre.CANCoder(4)

        if self.isReal():
            try:
                from cscore import CameraServer  # type: ignore
            except ImportError:
                self.logger.exception("Could not import CameraServer")
            else:
                CameraServer.getInstance().startAutomaticCapture()

    def autonomousInit(self) -> None:
        self.shooter_control.lead_shots = False
        self.intake.auto_retract = False
        self.shooter_control.auto_shoot = False
        self.vision.fuse_vision_observations = False

    def teleopInit(self) -> None:
        self.status_lights.display_morse = False
        self.intake.auto_retract = True
        self.shooter_control.lead_shots = True
        self.indexer_control.ignore_colour = False
        self.shooter_control.auto_shoot = False
        self.vision.max_std_dev = 0.4
        self.vision.fuse_vision_observations = True

    def disabledInit(self) -> None:
        self.status_lights.choose_morse_message()

    def disabledPeriodic(self) -> None:
        self.turret.update_angle_history()
        # self.chassis.update_odometry()
        # self.chassis.update_pose_history()
        self.turret.try_sync()
        self.vision.execute()
        self.led_control.execute()
        self.status_lights.execute()

    def teleopPeriodic(self) -> None:
        # handle chassis inputs
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)
        spin_rate = 4.0
        # Don't update these values while firing
        if (
            not self.lock_motion_while_shooting
            or self.shooter_control.current_state != "firing"
        ):
            joystick_x = (
                -rescale_js(self.joystick.getY(), deadzone=0.05, exponential=1.5)
                * 4
                * throttle
            )
            joystick_y = (
                -rescale_js(self.joystick.getX(), deadzone=0.05, exponential=1.5)
                * 4
                * throttle
            )
            joystick_z = (
                -rescale_js(self.joystick.getZ(), deadzone=0.3, exponential=25.0)
                * spin_rate
            )
            self.recorded_joystick_state = (joystick_x, joystick_y, joystick_z)
            self.recorded_is_local_driving = self.joystick.getRawButton(6)

        # Drive in field oriented mode unless button 6 is pressed
        if self.recorded_is_local_driving:
            self.chassis.drive_local(*self.recorded_joystick_state)
        else:
            self.chassis.drive_field(*self.recorded_joystick_state)

        if self.joystick.getRawButtonPressed(9):
            self.shooter_control.lead_shots = True
        if self.joystick.getRawButtonPressed(10):
            self.shooter_control.lead_shots = False

        if self.joystick.getRawButtonPressed(7):
            self.indexer_control.ignore_colour = True
        if self.joystick.getRawButtonPressed(8):
            self.indexer_control.ignore_colour = False

        # reset heading to intake facing directly downfield
        if self.codriver.getYButtonPressed():
            self.chassis.zero_yaw()

        if self.joystick.getTrigger():
            self.shooter_control.fire()

        if self.joystick.getRawButtonPressed(2):
            if self.intake.deployed:
                self.intake.deployed = False
                if self.indexer_control.current_state == "intaking":
                    self.indexer_control.stop()
            elif self.indexer.ready_to_intake():
                self.indexer_control.wants_to_intake = True
                self.intake.deployed = True

        # stop intake motor running if we have something in tunnel
        self.intake.motor_enabled = (
            self.indexer.ready_to_intake()
            or self.indexer_control.current_state == "clearing"
        )

        # hold down 11 to intake untill full, no auto-retract
        self.intake.auto_retract = not self.joystick.getRawButton(11)
        if self.joystick.getRawButton(11):
            self.intake.deployed = True
            self.indexer_control.wants_to_intake = True

        # will retract when has two balls regardless
        if (
            self.indexer.is_full()
            and self.indexer_control.current_state == "stopped"
            and not self.shooter_control._reject_through_turret
        ):
            self.intake.deployed = False

        if self.codriver.getBButtonPressed():
            self.indexer_control.engage("forced_clearing", force=True)

        self.indexer_control.catflap_active = self.codriver.getXButton()

        # Failsafe
        if self.codriver.getAButton():
            self.chassis.set_pose_failsafe()

    def testPeriodic(self) -> None:
        # hold y and use joystick throttle to set flywheel speed
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0, 1)
        self.shooter.motor_speed = throttle * 60

        # hold x and use left stick to slew turret
        if self.joystick.getPOV() != -1:
            self.turret.target += math.sin(math.radians(self.joystick.getPOV(0))) * 0.03

        # use buttons next to pov hat to command a step response from turret
        if self.joystick.getRawButtonPressed(3):
            self.turret.target += math.radians(45)
        if self.joystick.getRawButtonPressed(4):
            self.turret.target += math.radians(-45)
        if self.joystick.getRawButtonPressed(5):
            self.turret.target += math.radians(90)
        if self.joystick.getRawButtonPressed(6):
            self.turret.target += math.radians(-90)

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

        if self.codriver.getBButtonPressed():
            self.indexer_control.engage("forced_clearing", force=True)

        self.indexer_control.catflap_active = self.codriver.getXButton()

        # lower intake without running it
        if self.codriver.getLeftBumper():
            self.intake.motor_enabled = False
            self.intake.deployed = not self.intake.deployed
            self.intake.auto_retract = False

        if self.codriver.getRightBumper():
            self.chassis.drive_local(self.test_chassis_speed, 0, 0)
        else:
            self.chassis.drive_local(0, 0, 0)

        self.indexer_control.execute()

        self.chassis.execute()
        self.intake.execute()
        self.indexer.execute()
        self.shooter.execute()
        self.turret.execute()
        self.vision.execute()
        self.led_control.execute()
        self.status_lights.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
