import ctre
from magicbot import will_reset_to, tunable
from wpimath.geometry import Translation2d

from utilities.ctre import FALCON_CPR


class Shooter:
    left_motor: ctre.TalonFX
    right_motor: ctre.TalonFX

    motor_speed = will_reset_to(0.0)
    allowable_error = tunable(2.0)

    MAX_RP100ms = 10
    pidF = 1023 / (2048 * MAX_RP100ms)
    pidP = 0.15
    pidI = 0.0
    pidIZone = 200
    pidD = 0.01

    # Conversion factor from rev/s to Talon units (counts/100ms).
    RPS_TO_CTRE_UNITS = FALCON_CPR / 10
    CTRE_UNITS_TO_RPS = 10 / FALCON_CPR

    MAX_MOTOR_SPEED = 6000 / 60

    COMPENSATED_VOLTAGE = 11.0
    turret_offset = Translation2d(-0.148, 0)  # From CAD

    def setup(self) -> None:
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

            motor.configVelocityMeasurementPeriod(
                ctre.SensorVelocityMeasPeriod.Period_1Ms, timeoutMs=10
            )
            motor.configVelocityMeasurementWindow(8, timeoutMs=10)

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

    def on_disable(self) -> None:
        self._stop_motors()

    def execute(self) -> None:
        # XXX: mypy type inference bug?
        speed_rps: float = self.motor_speed
        if speed_rps:
            speed = speed_rps * self.RPS_TO_CTRE_UNITS
            self.right_motor.set(ctre.ControlMode.Velocity, speed)
            self.left_motor.set(ctre.ControlMode.Velocity, speed)
        else:
            self._stop_motors()

    def _stop_motors(self) -> None:
        self.left_motor.set(ctre.ControlMode.Disabled, 0)
        self.right_motor.set(ctre.ControlMode.Disabled, 0)

    def actual_velocity(self) -> float:
        return self.left_motor.getSelectedSensorVelocity() * self.CTRE_UNITS_TO_RPS

    def flywheel_error(self) -> float:
        return float(self.motor_speed) - self.actual_velocity()

    def is_at_speed(self) -> bool:
        return abs(self.flywheel_error()) < self.allowable_error
