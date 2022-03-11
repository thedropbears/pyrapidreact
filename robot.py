#!/usr/bin/env python3

import wpilib
import magicbot
import ctre
import navx
import rev
import math

from components.leds import StatusLights
from components.chassis import Chassis
from components.hanger import Hanger
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
    hanger: Hanger
    intake: Intake
    indexer: Indexer
    shooter: Shooter
    turret: Turret
    vision: Vision

    def createObjects(self):
        self.logger.info("pyrapidreact %s", GIT_INFO)

        self.imu = navx.AHRS.create_spi()

        self.leds = wpilib.AddressableLED(2)

        self.chassis_1_drive = ctre.TalonFX(1)
        self.chassis_1_steer = ctre.TalonFX(2)
        self.chassis_2_drive = ctre.TalonFX(3)
        self.chassis_2_steer = ctre.TalonFX(4)
        self.chassis_3_drive = ctre.TalonFX(5)
        self.chassis_3_steer = ctre.TalonFX(6)
        self.chassis_4_drive = ctre.TalonFX(7)
        self.chassis_4_steer = ctre.TalonFX(8)

        self.joystick = wpilib.Joystick(0)
        self.codriver = wpilib.XboxController(1)

        self.shooter_left_motor = ctre.TalonFX(11)
        self.shooter_right_motor = ctre.TalonFX(10)

        self.turret_motor = ctre.TalonSRX(15)
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

        self.climb_motor = ctre.TalonFX(22)

        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData(self.field)

        self.chassis_1_encoder = ctre.CANCoder(1)
        self.chassis_2_encoder = ctre.CANCoder(2)
        self.chassis_3_encoder = ctre.CANCoder(3)
        self.chassis_4_encoder = ctre.CANCoder(4)

    def autonomousInit(self) -> None:
        self.shooter_control.lead_shots = False
        self.intake.auto_retract = False
        self.shooter_control.auto_shoot = False
        self.vision.max_std_dev = 0.5

    def teleopInit(self) -> None:
        self.intake.auto_retract = True
        self.shooter_control.lead_shots = True
        self.indexer_control.ignore_colour = False
        self.shooter_control.auto_shoot = False
        self.vision.max_std_dev = 0.2

    def disabledPeriodic(self) -> None:
        self.turret.update_angle_history()
        self.chassis.update_odometry()
        self.chassis.update_pose_history()
        self.turret.try_sync()
        self.vision.execute()
        self.led_control.execute()
        self.status_lights.execute()

    def teleopPeriodic(self) -> None:
        # handle chassis inputs
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)
        spin_rate = 3.0
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
            self.shooter_control.auto_shoot = True

        if self.joystick.getRawButtonPressed(12):
            self.shooter_control.auto_shoot = False

        # reset heading to intake facing directly downfield
        if self.joystick.getRawButtonPressed(9):
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

        # Failsafe
        if self.codriver.getAButton():
            self.chassis.set_pose_failsafe()

        # Climb
        if self.codriver.getLeftBumper() and self.codriver.getRightBumper():
            self.shooter_control.track_target = False
            self.shooter_control.flywheels_running = False
            self.turret.slew_local(math.pi)
            self.intake.deploy_without_running()

        if self.intake.deployed and not self.intake.motor_enabled and abs(self.turret.get_error()) < math.pi / 18:
            right_trigger = self.codriver.getRightTriggerAxis()
            if right_trigger > 0.2:
                self.hanger.enabled = True
                self.hanger.winch(right_trigger)

            left_trigger = self.codriver.getLeftTriggerAxis()
            if left_trigger > 0.2:
                self.hanger.enabled = True
                self.hanger.payout(left_trigger)

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

        # lower intake without running it
        if self.codriver.getLeftBumper():
            self.intake.motor_enabled = False
            self.intake.deployed = not self.intake.deployed
            self.intake.auto_retract = False

        right_trigger = self.codriver.getRightTriggerAxis()
        if right_trigger > 0.2:
            self.hanger.enabled = True
            self.hanger.winch(right_trigger)

        left_trigger = self.codriver.getLeftTriggerAxis()
        if left_trigger > 0.2:
            self.hanger.enabled = True
            self.hanger.payout(left_trigger)

        self.indexer_control.execute()

        self.chassis.execute()
        self.hanger.execute()
        self.intake.execute()
        self.indexer.execute()
        self.shooter.execute()
        self.turret.execute()
        self.vision.execute()
        self.led_control.execute()
        self.status_lights.execute()


if __name__ == "__main__":
    wpilib.run(MyRobot)
