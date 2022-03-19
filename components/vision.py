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
    CAMERA_PITCH = math.radians(27)
    CAMERA_HEIGHT = 0.972
    TARGET_HEIGHT = 2.62
    # goal radius
    GOAL_RAD = 0.61

    field: wpilib.Field2d

    fuse_vision_observations = tunable(True)
    gate_innovation = tunable(True)

    def __init__(self) -> None:
        self.camera = PhotonCamera("gloworm")
        self.camera.setLEDMode(LEDMode.kOn)
        self.max_std_dev = 0.5
        self.has_target = False
        self.distance = -1

    def setup(self):
        self.field_obj = self.field.getObject("vision_pose")

    def execute(self):
        # gets vision data from camera
        results = self.camera.getLatestResult()
        self.has_target = results.hasTargets()
        if not self.has_target:
            return
        timestamp = wpilib.Timer.getFPGATimestamp() - results.getLatency()
        target_pitch = math.radians(results.getBestTarget().getPitch())
        target_yaw = math.radians(results.getBestTarget().getYaw())

        # work out our field position when photo was taken
        turret_rotation = self.turret.get_angle_at(timestamp)
        robot_rotation = self.chassis.get_pose_at(timestamp).rotation()

        # angle from the robot to target
        target_angle = turret_rotation + target_yaw
        # distnace from camera to middle of goal
        self.distance = (
            PhotonUtils.calculateDistanceToTarget(
                self.CAMERA_HEIGHT, self.TARGET_HEIGHT, self.CAMERA_PITCH, target_pitch
            )
            + self.GOAL_RAD
        )
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

            std_dev = self.max_std_dev * min(
                1, max(0, scale_value(self.distance, 5, 8, 1, 0.3))
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
