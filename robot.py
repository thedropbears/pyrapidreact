#!/usr/bin/env python3

import wpilib
import magicbot
import ctre
import navx

from components.chassis import Chassis
from components.hanger import Hanger
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.turret import Turret
from components.vision import Vision

from controllers.shooter import ShooterController
from utilities.scalers import rescale_js, scale_value


class MyRobot(magicbot.MagicRobot):
    shooter_control: ShooterController

    chassis: Chassis
    hanger: Hanger
    indexer: Indexer
    intake: Intake
    shooter: Shooter
    turret: Turret
    vision: Vision

    def createObjects(self):
        self.imu = navx.AHRS.create_i2c()

        self.chassis_1_drive = ctre.TalonFX(1)
        self.chassis_1_steer = ctre.TalonFX(2)
        self.chassis_2_drive = ctre.TalonFX(3)
        self.chassis_2_steer = ctre.TalonFX(4)
        self.chassis_3_drive = ctre.TalonFX(5)
        self.chassis_3_steer = ctre.TalonFX(6)
        self.chassis_4_drive = ctre.TalonFX(7)
        self.chassis_4_steer = ctre.TalonFX(8)

        self.joystick = wpilib.Joystick(0)

    def autonomousInit(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def testInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        # handle chassis inputs
        throttle = scale_value(self.joystick.getThrottle(), 1, -1, 0.1, 1)
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
        joystick_z = -rescale_js(self.joystick.getZ(), deadzone=0.2, exponential=20.0)

        # if joystick_x or joystick_y or joystick_z:
        # Drive in field oriented mode unless button 6 is held
        if not self.joystick.getRawButton(6):
            self.chassis.drive_field(joystick_x, joystick_y, joystick_z)
        else:
            self.chassis.drive_local(joystick_x, joystick_y, joystick_z)
        # else:
        #     self.chassis.stop()

        # Reset the heading when button 7 is pressed
        if self.joystick.getRawButtonPressed(7):
            self.gyro.reset()

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
