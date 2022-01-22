import wpilib
import ctre
import magicbot

from components.chassis import Chassis
from components.hanger import Hanger
from components.indexer import Indexer
from components.intake import Intake
from components.shooter import Shooter
from components.turret import Turret
from components.vision import Vision

from controllers.shooter import ShooterController


class MyRobot(magicbot.MagicRobot):
    shooter_control: ShooterController

    chassis: Chassis
    hanger: Hanger
    indexer: Indexer
    intake: Intake
    shooter: Shooter
    turret: Turret
    vision: Vision

    def createObjects(self) -> None:
        pass

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
