import wpilib
import ctre
import magicbot


class MyRobot(magicbot.MagicRobot):
    flywheelSpeed = magicbot.tunable(0.1)

    def createObjects(self) -> None:
        self.intakeMotor = ctre.TalonFX(2)

    def teleopPeriodic(self):
        self.intakeMotor.set(ctre.TalonFXControlMode.PercentOutput, self.flywheelSpeed)


if __name__ == "__main__":
    wpilib.run(MyRobot)
