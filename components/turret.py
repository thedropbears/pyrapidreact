from collections import deque
import ctre
import magicbot
import math
import navx


class Turret:
    motor: ctre.TalonSRX
    imu: navx.AHRS

    pidF = 0.2
    pidP = 1.0
    pidI = 0.005
    pidIZone = 200
    pidD = 4.0

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 175 / 24
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = int(COUNTS_PER_TURRET_REV / math.tau)

    SLEW_CRUISE_VELOCITY = 0.1 * COUNTS_PER_TURRET_RADIAN / 10
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.15)

    target = magicbot.tunable(0.0)
    control_loop_wait_time: float

    # max rotation either side of zero
    MAX_ROTATION = math.radians(200)
    soft_limit = math.radians(30)

    def __init__(self):
        self.angle_history = deque([], maxlen=100)

    def setup(self):
        self.motor.configFactoryDefault()

        # Positive motion is counterclockwise from above.
        self.motor.setInverted(True)
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

        self.motor.setSelectedSensorPosition(
            0
        )  # replace 0 with absolute encoder position

    def execute(self) -> None:
        # constrain in a way that allows a bit of overlap
        while self.target > self.MAX_ROTATION:
            self.target -= math.tau
        while self.target < -self.MAX_ROTATION:
            self.target += math.tau
        # soft limits for testing
        self.target = min(max(self.target, -self.soft_limit), self.soft_limit)
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

    def get_angle_at(self, t: float) -> float:
        # loops_ago = int((wpilib.Timer.getFPGATimestamp() - t) / self.control_loop_wait_time)
        # if loops_ago >= len(self.angle_history):
        #     return (
        #         self.angle_history[-1]
        #         if len(self.angle_history) > 0
        #         else self.get_angle()
        #     )
        # return self.angle_history[loops_ago]
        return self.get_angle()
