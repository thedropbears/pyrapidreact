from magicbot import tunable
import ctre
from wpilib import Joystick


class Shooter:
    joystick: Joystick

    left_motor: ctre.TalonFX
    right_motor: ctre.TalonFX

    left_feeder_motor: ctre.TalonSRX
    right_feeder_motor: ctre.TalonSRX


    motor_speed = tunable(0.0)

    def setup(self):
        self.left_motor.setInverted(True)
        self.right_motor.setInverted(False)

        self.left_feeder_motor.setInverted(True)
        self.right_feeder_motor.setInverted(False)
        

        for motor in self.left_motor, self.right_motor, self.left_feeder_motor, self.right_feeder_motor:
            motor.setNeutralMode(ctre.NeutralMode.Coast)

    def execute(self):
        self.right_motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
        self.left_motor.set(ctre.ControlMode.PercentOutput, self.motor_speed)
        
        self.right_feeder_motor.set(ctre.ControlMode.PercentOutput, self.joystick.getTrigger() * self.joystick.getThrottle())
        self.right_feeder_motor.set(ctre.ControlMode.PercentOutput, self.joystick.getTrigger() * self.joystick.getThrottle())
