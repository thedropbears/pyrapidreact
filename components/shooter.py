import ctre
import wpilib
import magicbot

import math


class Shooter:
    joystick: wpilib.Joystick

    left_motor: ctre.TalonFX
    right_motor: ctre.TalonFX

    motor_speed = 0.0
    allowable_error = magicbot.tunable(20.0)

    MAX_RP100ms = 10
    pidF = 1023 / (2048 * MAX_RP100ms)
    pidP = 0.15
    pidI = 0.0
    pidIZone = 200
    pidD = 0.01
    SLEW_CRUISE_VELOCITY = 4000
    SCAN_CRUISE_VELOCITY = 1500
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.15)

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 2048
    GEAR_REDUCTION = 1 / 1
    COUNTS_PER_SHOOTER_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_SHOOTER_RADIAN = int(COUNTS_PER_SHOOTER_REV / math.tau)

    COUNTS_PER_REV = 2048
    RPS_TO_CTRE_UNITS = COUNTS_PER_REV / 10  # counts per 100ms

    MAX_MOTOR_SPEED = 6000 / 60

    COMPENSATED_VOLTAGE = 11.0

    def setup(self):
        self.left_motor.setInverted(False)
        self.right_motor.setInverted(True)

        for motor in (
            self.left_motor,
            self.right_motor,
        ):
            motor.configFactoryDefault()
            motor.setNeutralMode(ctre.NeutralMode.Coast)

            motor.configVoltageCompSaturation(self.COMPENSATED_VOLTAGE, timeoutMs=10)
            motor.enableVoltageCompensation(True)

            motor.config_kF(0, self.pidF, 10)
            motor.config_kP(0, self.pidP, 10)
            motor.config_kI(0, self.pidI, 10)
            motor.config_kD(0, self.pidD, 10)
            motor.configSelectedFeedbackSensor(
                ctre.FeedbackDevice.IntegratedSensor, 0, 10
            )
            motor.setStatusFramePeriod(
                ctre.StatusFrameEnhanced.Status_1_General, 250, 10
            )

    def execute(self):
        self.right_motor.set(
            ctre.ControlMode.Velocity,
            self.motor_speed * self.RPS_TO_CTRE_UNITS,
        )
        self.left_motor.set(
            ctre.ControlMode.Velocity,
            self.motor_speed * self.RPS_TO_CTRE_UNITS,
        )

    @magicbot.feedback
    def actual_velocity(self):
        return self.left_motor.getSelectedSensorVelocity() / self.RPS_TO_CTRE_UNITS

    @magicbot.feedback
    def flywheel_error(self):
        return self.motor_speed - self.actual_velocity()

    def is_at_speed(self) -> bool:
        return abs(self.flywheel_error()) < self.allowable_error
