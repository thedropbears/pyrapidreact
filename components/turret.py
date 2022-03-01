from collections import deque
from logging import Logger
import ctre
import magicbot
import math
from wpilib import DutyCycleEncoder, Timer


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
    MAX_ROTATION = math.radians(200)

    allowable_error = magicbot.tunable(math.radians(20))  # radians

    logger: Logger

    def __init__(self):
        self.angle_history = deque([], maxlen=100)
        self.has_synced = False
        self.abs_offset = 3.07

    def setup(self):
        self.motor.configFactoryDefault()

        # Positive motion is counterclockwise from above.
        self.motor.setInverted(False)
        # set the peak and nominal outputs
        self.motor.configNominalOutputForward(0, 10)
        self.motor.configNominalOutputReverse(0, 10)
        self.motor.configPeakOutputForward(1.0, 10)
        self.motor.configPeakOutputReverse(-1.0, 10)
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

    def on_disable(self):
        self.has_synced = False

    def on_enable(self):
        self.has_synced = False
        self.try_sync()

    def try_sync(self):
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
        while theta < -cls.MAX_ROTATION:
            theta += math.tau
        return theta

    def execute(self) -> None:
        self.target = self.wrap_allowable_angle(self.target)
        self.angle_history.appendleft(self.get_angle())

        self.motor.set(
            ctre.ControlMode.MotionMagic,
            self.target * self.COUNTS_PER_TURRET_RADIAN,
        )

    def slew_relative(self, angle: float) -> None:
        """Slews relative to current turret position"""
        self.slew_local(self.get_angle() + angle)

    def slew_local(self, angle: float) -> None:
        """Slew to a robot relative angle"""
        self.target = angle

    @magicbot.feedback
    def get_angle(self):
        return self.motor.getSelectedSensorPosition() / self.COUNTS_PER_TURRET_RADIAN

    def get_error(self) -> float:
        return self.get_angle() - self.target

    def is_on_target(self) -> bool:
        return abs(self.get_error()) < self.allowable_error

    @magicbot.feedback
    def absolute_encoder_reading(self) -> float:
        angle = self.absolute_encoder.getDistance() + self.abs_offset
        return self.wrap_allowable_angle(angle)

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
