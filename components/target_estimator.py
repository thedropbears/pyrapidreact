from components.vision import Vision
from typing import Optional, Tuple


class TargetEstimator:
    """Fuses data from vision, odometry, the imu and other sources
    to estimate where the target is from the robot and where you should aim to hit it when moving"""

    # https://robotpy.readthedocs.io/projects/wpimath/en/stable/wpimath.estimator.html
    vision: Vision

    def __init__(self) -> None:
        self.angle_to_target: Optional[float] = None
        self.position: Optional[Tuple[float, float]] = None

    def execute(self) -> None:
        visionData = self.vision.get_data()
        if visionData is not None:
            self.angle_to_target = visionData.angle
        else:
            self.angle_to_target = None

    def to_target(self) -> Optional[float]:
        return self.angle_to_target
