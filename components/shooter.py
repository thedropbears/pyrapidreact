from magicbot import tunable
import ctre


class Shooter:
    left_motor: ctre.TalonFX
    right_motor: ctre.TalonFX

    motor_speed = tunable(0.0)

    def setup(self):
        self.left_motor.setInverted(True)
        self.right_motor.setInverted(False)
        

        for motor in self.left_motor, self.right_motor:
            motor.setNeutralMode(ctre.NeutralMode.Coast)

    def execute(self):
        self.right_motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
        self.left_motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
