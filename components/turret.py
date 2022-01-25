import ctre
import magicbot
import math


class Turret:
    motor: ctre.TalonSRX

    pidF = 0.2
    pidP = 1.0
    pidI = 0.005
    pidIZone = 200
    pidD = 4.0
    SLEW_CRUISE_VELOCITY = 4000
    SCAN_CRUISE_VELOCITY = 1500
    CRUISE_ACCELERATION = int(SLEW_CRUISE_VELOCITY / 0.15)

    # Constants for Talon on the turret
    COUNTS_PER_MOTOR_REV = 4096
    GEAR_REDUCTION = 175 / 18
    COUNTS_PER_TURRET_REV = COUNTS_PER_MOTOR_REV * GEAR_REDUCTION
    COUNTS_PER_TURRET_RADIAN = int(COUNTS_PER_TURRET_REV / math.tau)

    target = magicbot.tunable(0)

    def setup(self):
        self._setup_motor()

    def execute(self) -> None:
        self.motor.getSelectedSensorPosition(0)

        self.motor.set(
            ctre.ControlMode.MotionMagic, self.target * self.COUNTS_PER_TURRET_RADIAN
        )

    def slew_relative(self, angle: float) -> None:
        self.target += angle

    def _setup_motor(self) -> None:
        self.motor.configFactoryDefault()

        # Positive motion is counterclockwise from above.
        self.motor.setInverted(ctre.InvertType.InvertMotorOutput)
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
