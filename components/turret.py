from collections import deque
from logging import Logger
from typing import Deque

import ctre
import magicbot
import math
from wpilib import DutyCycleEncoder, Timer

from utilities.functions import constrain_angle


class Turret:
    motor: ctre.TalonSRX
    absolute_encoder: DutyCycleEncoder

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 240 / 60
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = int(COUNTS_PER_TURRET_REV / math.tau)

    # pidF = 0.71901 / 12 * 1023 / 10 * math.tau / COUNTS_PER_MOTOR_REV
    pidF = 1
    pidP = 2
    pidI = 0.0
    pidIZone = 200
    pidD = 3  # 1.109

    SLEW_CRUISE_VELOCITY = 6 * COUNTS_PER_TURRET_RADIAN / 10
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.1)

    target = magicbot.tunable(0.0)
    control_loop_wait_time: float

    # max rotation either side of zero
    MIN_ROTATION = math.radians(-30)
    MAX_ROTATION = math.radians(390)

    allowable_position_error = magicbot.tunable(math.radians(10))  # radians
    allowable_velocity_error = magicbot.tunable(0.25)  # turret rev/s

    logger: Logger

    def __init__(self) -> None:
        self.angle_history: Deque[float] = deque([], maxlen=100)
        self.has_synced = False
        self.abs_offset = 3.07

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

    def on_enable(self):
        self.try_sync()

    def try_sync(self) -> None:
        if not self.has_synced and self.absolute_encoder.isConnected():
            self.motor.setSelectedSensorPosition(
                self.absolute_encoder_reading() * self.COUNTS_PER_TURRET_RADIAN
            )
            self.has_synced = True

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
        return self.motor.getSelectedSensorPosition() / self.COUNTS_PER_TURRET_RADIAN

    def get_error(self) -> float:
        return self.get_angle() - self.target

    def is_on_target(self) -> bool:
        return (
            abs(self.get_error()) < self.allowable_position_error
            and abs(self.motor.getSelectedSensorVelocity())
            < self.allowable_velocity_error
            * self.COUNTS_PER_TURRET_REV
            / 10  # Convert to counts/100ms
        )

    @magicbot.feedback
    def absolute_encoder_reading(self) -> float:
        angle = self.absolute_encoder.getDistance() + self.abs_offset

        while angle > math.tau:
            angle -= math.tau
        while angle < 0:
            angle += math.tau
        return angle

    def get_angle_at(self, t: float) -> float:
        loops_ago = int((Timer.getFPGATimestamp() - t) / self.control_loop_wait_time)
        if loops_ago < 0:
            self.logger.warning("vision clocks wrapped")
            return self.get_angle()
        if loops_ago >= len(self.angle_history):
            return (
                self.angle_history[-1]
                if len(self.angle_history) > 0
                else self.get_angle()
            )
        return self.angle_history[loops_ago]

    def update_angle_history(self) -> None:
        self.angle_history.appendleft(self.get_angle())
