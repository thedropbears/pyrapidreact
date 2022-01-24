from magicbot import tunable
import ctre
from wpilib import Joystick
import magicbot

import math


class Shooter:
    joystick: Joystick

    left_motor: ctre.TalonFX
    right_motor: ctre.TalonFX

    left_feeder_motor: ctre.TalonSRX
    right_feeder_motor: ctre.TalonSRX

    motor_speed = tunable(0.0)

    pidF = 0.05
    pidP = 0.01
    pidI = 0.000
    pidIZone = 200
    pidD = 0
    SLEW_CRUISE_VELOCITY = 4000
    SCAN_CRUISE_VELOCITY = 1500
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.15)

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 2048
    GEAR_REDUCTION = 1 / 1
    COUNTS_PER_SHOOTER_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_SHOOTER_RADIAN = int(COUNTS_PER_SHOOTER_REV / math.tau)

    feeder_speed = 0.7

    rpm_to_native = magicbot.tunable(60 * 10 / 2048)

    def setup(self):
        self.left_motor.setInverted(True)
        self.right_motor.setInverted(False)

        self.left_feeder_motor.setInverted(False)
        self.right_feeder_motor.setInverted(False)

        for motor in (
            self.left_motor,
            self.right_motor,
            self.left_feeder_motor,
            self.right_feeder_motor,
        ):
            motor.setNeutralMode(ctre.NeutralMode.Coast)

            motor.configNominalOutputForward(0, 10)
            motor.configNominalOutputReverse(0, 10)
            motor.configPeakOutputForward(1.0, 10)
            motor.configPeakOutputReverse(-1.0, 10)
            motor.config_kF(0, self.pidF, 10)
            motor.config_kP(0, self.pidP, 10)
            motor.config_kI(0, self.pidI, 10)
            motor.config_kD(0, self.pidD, 10)
            motor.configSelectedFeedbackSensor(
                ctre.FeedbackDevice.IntegratedSensor, 0, 10
            )

    def execute(self):
        self.left_feeder_motor.set(
            ctre.ControlMode.PercentOutput,
            self.joystick.getTrigger() * self.feeder_speed,
        )
        self.right_feeder_motor.set(
            ctre.ControlMode.PercentOutput,
            self.joystick.getTrigger() * self.feeder_speed,
        )
        self.motor_speed = self.joystick.getThrottle()
        self.right_motor.set(
            ctre.ControlMode.Velocity, self.motor_speed * self.rpm_to_native
        )
        self.left_motor.set(
            ctre.ControlMode.Velocity, self.motor_speed * self.rpm_to_native
        )

    @magicbot.feedback
    def actual_velocity(self):
        return 60 * 10 * self.left_motor.getSelectedSensorVelocity() / 2048

    @magicbot.feedback
    def flywheel_error(self):
        return self.left_motor.getClosedLoopError()
