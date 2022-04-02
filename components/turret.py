from logging import Logger

import ctre
import magicbot
import math
from wpilib import DutyCycleEncoder, Timer, Solenoid
from wpimath.interpolation import TimeInterpolatableFloatBuffer

from utilities.functions import constrain_angle


class Turret:
    motor: ctre.WPI_TalonSRX
    absolute_encoder: DutyCycleEncoder
    cable_piston: Solenoid

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 240 / 60
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = int(COUNTS_PER_TURRET_REV / math.tau)
    RADIANS_PER_COUNT = math.tau / COUNTS_PER_TURRET_REV

    # Turret experiments show speed of 1884 counts/100ms for a throttle of about 0.35
    pidF = (0.35 * 1023) / 1884
    pidP = 3.0
    pidI = 0.02
    pidIZone = 200
    pidD = 90.0

    SLEW_CRUISE_VELOCITY = 10 * COUNTS_PER_TURRET_RADIAN / 10
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.1)

    target = magicbot.tunable(math.pi / 2)

    # max rotation either side of zero
    MIN_ROTATION = math.radians(-15)
    MAX_ROTATION = math.radians(375)

    allowable_velocity_error = magicbot.tunable(0.1)  # turret rev/s

    PISTON_EXTEND_THRESHOLD = math.radians(50)
    PISTON_CONTRACT_THRESHOLD = math.radians(60)

    is_piston_extended = False

    logger: Logger

    def __init__(self) -> None:
        self.angle_history = TimeInterpolatableFloatBuffer(2)
        self.sync_count = 0
        self.abs_offset = 1.886

    def setup(self) -> None:
        self.motor.configFactoryDefault()

        # Positive motion is counterclockwise from above.
        self.motor.setInverted(False)

        self.motor.configPeakCurrentLimit(0, timeoutMs=10)
        self.motor.configContinuousCurrentLimit(15, timeoutMs=10)

        self.motor.config_kF(0, self.pidF, 10)
        self.motor.config_kP(0, self.pidP, 10)
        self.motor.config_IntegralZone(0, self.pidIZone, 10)
        self.motor.config_kI(0, self.pidI, 10)
        self.motor.config_kD(0, self.pidD, 10)

        self.motor.configMotionCruiseVelocity(self.SLEW_CRUISE_VELOCITY, 10)
        self.motor.configMotionAcceleration(self.CRUISE_ACCELERATION, 10)
        # self.motor.configAllowableClosedloopError(0, self.ACCEPTABLE_ERROR_COUNTS, 10)

        self.motor.configSelectedFeedbackSensor(
            ctre.FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10
        )
        self.absolute_encoder.setDistancePerRotation(-math.tau)

    def on_disable(self) -> None:
        self.motor.set(ctre.TalonSRXControlMode.Disabled, 0)

    def on_enable(self):
        self.try_sync()

    def try_sync(self) -> None:
        if self.absolute_encoder.isConnected():
            self.sync_count += 1
            if self.sync_count == 150:
                angle = self.absolute_encoder_reading()
                err = self.motor.setSelectedSensorPosition(
                    angle * self.COUNTS_PER_TURRET_RADIAN
                )
                if err != ctre.ErrorCode.OK:
                    self.sync_count -= 1
                self.target = angle

    @classmethod
    def wrap_allowable_angle(cls, theta: float) -> float:
        """
        Constrain a given angle to be within allowable angles for the turret.
        This allows for some overlap around the turret's wraparound.
        """
        while theta > cls.MAX_ROTATION:
            theta -= math.tau
        while theta < cls.MIN_ROTATION:
            theta += math.tau
        return theta

    def execute(self) -> None:
        self.target = self.wrap_allowable_angle(self.target)
        self.update_angle_history()

        self.motor.set(
            ctre.ControlMode.MotionMagic,
            self.target * self.COUNTS_PER_TURRET_RADIAN,
        )

        if self.is_piston_extended:
            if abs(constrain_angle(self.get_angle())) > self.PISTON_CONTRACT_THRESHOLD:
                self.is_piston_extended = False
        else:
            if abs(constrain_angle(self.get_angle())) < self.PISTON_EXTEND_THRESHOLD:
                self.is_piston_extended = True
        self.cable_piston.set(not self.is_piston_extended)

    def slew_relative(self, angle: float) -> None:
        """Slews relative to current turret position"""
        self.target = self.get_angle() + angle

    def slew_local(self, angle: float) -> None:
        """Slew to a robot relative angle"""
        # Make sure we account for overlap, so test if an overlapped angle is actually closer
        delta = constrain_angle(angle - self.get_angle())
        self.slew_relative(delta)

    @magicbot.feedback
    def get_angle(self) -> float:
        return self.motor.getSelectedSensorPosition() * self.RADIANS_PER_COUNT

    def get_error(self) -> float:
        return self.get_angle() - self.target

    def is_on_target(self, allowable_error: float) -> bool:
        return (
            abs(self.get_error()) < allowable_error
            and abs(self.motor.getSelectedSensorVelocity())
            < self.allowable_velocity_error
            * self.COUNTS_PER_TURRET_REV
            / 10  # Convert to counts/100ms
        )

    @magicbot.feedback
    def absolute_encoder_reading(self) -> float:
        angle = self.absolute_encoder.getDistance() + self.abs_offset

        angle %= math.tau
        return angle

    def get_angle_at(self, t: float) -> float:
        return self.angle_history.sample(t)

    def update_angle_history(self) -> None:
        self.angle_history.addSample(Timer.getFPGATimestamp(), self.get_angle())
