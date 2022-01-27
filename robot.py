#!/usr/bin/env python3

import wpilib
import magicbot
import ctre
<<<<<<< HEAD
=======
import rev
>>>>>>> added run motor commands

from components.chassis import Chassis
from components.hanger import Hanger
from components.intake import Intake
from components.shooter import Shooter
from components.turret import Turret
from components.vision import Vision

from controllers.shooter import ShooterController

from utilities import git

GIT_INFO = git.describe()


class MyRobot(magicbot.MagicRobot):
    shooter_control: ShooterController

    chassis: Chassis
    hanger: Hanger
    intake: Intake
    shooter: Shooter
    turret: Turret
    vision: Vision

    def createObjects(self) -> None:
        self.logger.info("pyrapidreact %s", GIT_INFO)

        self.shooter_left_motor = ctre.TalonFX(11)
        self.shooter_right_motor = ctre.TalonFX(10)

        self.joystick = wpilib.Joystick(0)

        self.intake_intake_motor = ctre.TalonSRX(13)
        self.intake_indexer_motor = ctre.TalonSRX(14)
        self.intake_feed_motor = ctre.TalonSRX(15)
        self.intake_colour_sensor = rev.ColorSensorV3(wpilib.I2C.Port(0))

    def autonomousInit(self) -> None:
        pass

    def teleopInit(self) -> None:
        pass

    def testInit(self) -> None:
        pass

    def disabledPeriodic(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

    def testPeriodic(self) -> None:
        pass


if __name__ == "__main__":
    wpilib.run(MyRobot)
