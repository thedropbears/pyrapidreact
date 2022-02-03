from components.vision import Vision
from typing import Tuple
from wpimath.geometry import Pose2d, Translation2d
import navx
import math


class TargetEstimator:
    """Fuses data from vision, odometry, the imu and other sources
    to estimate where the target is from the robot and where you should aim to hit it when moving"""

    # https://robotpy.readthedocs.io/projects/wpimath/en/stable/wpimath.estimator.html
    vision: Vision
    imu: navx.AHRS

    def __init__(self) -> None:
        self.robot_pose: Pose2d = Pose2d(0, 0, 0)

    def execute(self) -> None:
        visionData = self.vision.get_data()
        if visionData is not None:
            self.angle_to_target = visionData.angle
        else:
            self.angle_to_target = 0

    def to_target(self) -> Tuple[float, float]:
        """Returns angle distance and to shoot at to hit the target"""
        field_angle = math.atan2(
            self.robot_pose.Y, self.robot_pose.X
        )  # may need to flip / convert
        local_angle = field_angle - self.imu.getRotation2d()
        return local_angle, self.robot_pose.translation().distance(
            Translation2d(x=0, y=0)
        )
