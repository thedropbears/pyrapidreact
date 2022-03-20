import math
from magicbot import feedback, tunable
from components.turret import Turret
from components.chassis import Chassis
import wpilib
from utilities.scalers import scale_value
from utilities.trajectory_generator import goal_to_field
from photonvision import PhotonCamera, PhotonUtils, LEDMode
from wpimath.geometry import Pose2d, Translation2d, Rotation2d


class Vision:
    """Communicates with limelight to get vision data and calculate pose"""

    turret: Turret
    chassis: Chassis

    CAMERA_OFFSET = -0.1  # m from camera to centre of turret, measured from CAD
    TURRET_OFFSET = -0.15  # m from robot centre to turret centre, measured from CAD

    # camera angle from horizontal
    CAMERA_PITCH = math.radians(28)
    CAMERA_HEIGHT = 0.972
    TARGET_HEIGHT = 2.62
    # goal radius
    GOAL_RADIUS = 0.61

    field: wpilib.Field2d

    fuse_vision_observations = tunable(True)
    gate_innovation = tunable(True)

    def __init__(self) -> None:
        self.camera = PhotonCamera("gloworm")
        self.camera.setLEDMode(LEDMode.kOn)
        self.max_std_dev = 0.4
        self.has_target = False
        self.distance = -1

    def setup(self) -> None:
        self.field_obj = self.field.getObject("vision_pose")

    def execute(self) -> None:
        results = self.camera.getLatestResult()
        self.has_target = results.hasTargets()
        if not self.has_target:
            return
        timestamp = wpilib.Timer.getFPGATimestamp() - results.getLatency()
        target_pitch = math.radians(results.getBestTarget().getPitch())
        target_yaw = -math.radians(
            results.getBestTarget().getYaw()
        )  # PhotonVision has yaw reversed from our RH coordinate system

        # work out our field position when photo was taken
        turret_rotation = self.turret.get_angle_at(timestamp)
        robot_rotation = self.chassis.get_pose_at(timestamp).rotation()

        # angle from the robot to target
        target_angle = turret_rotation + target_yaw
        # distance from camera to middle of goal
        self.distance = (
            PhotonUtils.calculateDistanceToTarget(
                self.CAMERA_HEIGHT, self.TARGET_HEIGHT, self.CAMERA_PITCH, target_pitch
            )
            + self.GOAL_RADIUS
        )
        # Numbers seem correct to around 5m, then start to overestimate
        # Suspect this is because the target starts getting much smaller and apparently flatter
        # Actual - calculated
        # 3 - 2.93
        # 4 - 3.95
        # 5 - 5.04
        # 6 - 6.16 - 2.7% - 97.4%
        # 7 - 7.32 - 4.6% - 95.6%
        # 8 - 8.59 - 7.4% - 93.1%
        # 9 - 9.72 - 8.0% - 92.6%
        scaling = 1.0
        if self.distance > 9.75:
            scaling = 0.926
        elif self.distance > 5.0:
            scaling = scale_value(self.distance, 5.0, 9.75, 1.0, 0.926)
        self.distance *= scaling

        vision_pose = pose_from_vision(
            self.distance, target_angle, robot_rotation.radians()
        )
        self.field_obj.setPose(goal_to_field(vision_pose))

        if self.fuse_vision_observations:
            innovation = vision_pose.translation().distance(
                self.chassis.estimator.getEstimatedPosition().translation()
            )
            # Gate on innovation
            if self.gate_innovation and innovation > 5.0:
                return

            if self.distance < 5.0:
                std_dev = 0.3 * self.max_std_dev
            elif self.distance > 8.0:
                std_dev = 1.0 * self.max_std_dev
            else:
                std_dev = self.max_std_dev * scale_value(
                    self.distance, 5.0, 8.0, 0.3, 1.0
                )
            self.chassis.estimator.addVisionMeasurement(
                vision_pose,
                timestamp,
                (std_dev, std_dev, 0.001),
            )

    @feedback
    def is_ready(self) -> bool:
        return self.has_target

    @feedback
    def get_distance(self) -> float:
        return self.distance


def pose_from_vision(
    range: float, turret_angle: float, chassis_heading: float
) -> Pose2d:
    # Assume robot is at origin
    location = Translation2d(
        Vision.TURRET_OFFSET + math.cos(turret_angle) * range,
        math.sin(turret_angle) * range,
    )
    # Rotate by the chassis heading
    location = location.rotateBy(Rotation2d(chassis_heading))
    # Now transform the pose so that the target is at the origin
    return Pose2d(-location.X(), -location.Y(), chassis_heading)
