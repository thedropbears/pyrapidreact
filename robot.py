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
from wpimath.filter import SlewRateLimiter

from controllers.shooter import ShooterController
from utilities.scalers import apply_deadzone, rescale_js, scale_value

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

    lock_motion_while_shooting = magicbot.tunable(True)
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

        self.recorded_drive_state = (0.0, 0.0, 0.0)
        self.recorded_is_local_driving = False
        self.gamepad = wpilib.XboxController(1)
        self.joystick_x_filter = SlewRateLimiter(
            Chassis.max_attainable_wheel_speed / 0.3
        )
        self.joystick_y_filter = SlewRateLimiter(
            Chassis.max_attainable_wheel_speed / 0.3
        )
        self.joystick_z_filter = SlewRateLimiter(7 / 0.2)
        # joystick used for test mode
        self.joystick = wpilib.Joystick(0)

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
        self.intake.auto_retract = True
        self.shooter_control.lead_shots = True
        self.indexer_control.ignore_colour = False
        self.shooter_control.auto_shoot = False
        self.vision.fuse_vision_observations = True

    def disabledPeriodic(self) -> None:
        self.turret.update_angle_history()
        # self.chassis.update_odometry()
        # self.chassis.update_pose_history()
        self.turret.try_sync()
        self.vision.execute()
        self.led_control.execute()
        self.status_lights.execute()

    def teleopPeriodic(self) -> None:
        # left trigger increases turn speed
        spin_rate = scale_value(self.gamepad.getLeftTriggerAxis(), 0, 1, 2.5, 7)
        # Don't update these values while firing
        if (
            not self.lock_motion_while_shooting
            or self.shooter_control.current_state != "firing"
        ):
            joystick_x = (
                -apply_deadzone(self.gamepad.getLeftY(), 0.15)
                * Chassis.max_attainable_wheel_speed
            )
            joystick_y = (
                -apply_deadzone(self.gamepad.getLeftX(), 0.15)
                * Chassis.max_attainable_wheel_speed
            )
            joystick_z = (
                -rescale_js(self.gamepad.getRightX(), deadzone=0.15, exponential=4.0)
                * spin_rate
            )
            self.recorded_drive_state = (
                self.joystick_x_filter.calculate(joystick_x),
                self.joystick_y_filter.calculate(joystick_y),
                self.joystick_z_filter.calculate(joystick_z),
            )
            self.recorded_is_local_driving = self.gamepad.getAButton()

        # Drive in field oriented mode unless A button is pressed
        if self.recorded_is_local_driving:
            self.chassis.drive_local(*self.recorded_drive_state)
        else:
            self.chassis.drive_field(*self.recorded_drive_state)

        if self.gamepad.getRightTriggerAxis() > 0.3:
            self.shooter_control.fire()

        # right bumper puts intake down, hold to hold intake down
        self.intake.auto_retract = not self.gamepad.getLeftBumper()
        if self.gamepad.getLeftBumper():
            self.indexer_control.wants_to_intake = True

        # left bumper raises intake
        if self.gamepad.getRightBumper():
            self.indexer_control.wants_to_intake = False

        if self.gamepad.getYButtonPressed():
            self.indexer_control.engage("forced_clearing", force=True)

        self.indexer_control.catflap_active = self.gamepad.getXButton()

        # Failsafe
        if self.gamepad.getBButton():
            self.chassis.set_pose_failsafe()

        # d pad up to zero heading
        if self.gamepad.getPOV() == 0:
            self.chassis.zero_yaw()

    def testPeriodic(self) -> None:
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
            self.indexer_control.wants_to_intake = (
                not self.indexer_control.wants_to_intake
            )

        # hold down 11 to intake untill full, no auto-retract
        self.intake.auto_retract = not self.joystick.getRawButton(11)
        if self.joystick.getRawButton(11):
            self.indexer_control.wants_to_intake = True

        if self.gamepad.getBButtonPressed():
            self.indexer_control.engage("forced_clearing", force=True)

        self.indexer_control.catflap_active = self.gamepad.getXButton()

        # lower intake without running it
        if self.gamepad.getLeftBumper():
            self.intake.motor_enabled = False
            self.intake.deployed = not self.intake.deployed
            self.intake.auto_retract = False

        if self.gamepad.getRightBumper():
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
