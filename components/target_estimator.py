from collections import deque
from components.chassis import Chassis
from components.turret import Turret
from components.vision import Vision
from typing import Tuple
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from utilities.functions import constrain_angle
import navx
import wpilib
import math

from utilities.scalers import scale_value


class TargetEstimator:
    """Fuses data from vision, odometry, the imu and other sources
    to estimate where the target is from the robot and where you should aim to hit it when moving"""

    # https://robotpy.readthedocs.io/projects/wpimath/en/stable/wpimath.estimator.html
    vision: Vision
    imu: navx.AHRS
    chassis: Chassis
    turret: Turret

    control_loop_wait_time: float

    def __init__(self) -> None:
        self.robot_pose: Pose2d = Pose2d(0, 0, 0)
        self.pose_history: deque = deque([], maxlen=100)

    def execute(self) -> None:
        vis_data = self.vision.get_data()
        if vis_data is not None:
            # vis_angle, vis_dist, vis_fit, vis_time = vision_data
            # work out where the vision data was taken from based on histories
            vis_taken_at_angle = self.get_pose_at(
                vis_data.timestamp
            ).rotation().radians() + self.turret.get_angle_at(vis_data.timestamp)
            # angle from target to robot in world space
            vis_angle_from_target = constrain_angle(vis_taken_at_angle - +math.pi)
            vis_estimate = Translation2d(
                distance=vis_data.distance, angle=Rotation2d(vis_angle_from_target)
            )
            diff = self.robot_pose.translation().distance(vis_estimate)
            vis_confidence = vis_data.fittness * scale_value(diff, 0, 1, 1, 0.05)
        else:
            vis_estimate = Translation2d(0, 0)
            vis_confidence = 0

        odometry_estimate = self.chassis.odometry.getPose().translation()
        total_accel = abs(self.imu.getRawAccelX()) + abs(self.imu.getRawAccelY())
        odometry_confidence = max(0.5, 2 - total_accel)

        imu_estimate = self.robot_pose.translation() + Translation2d(
            self.imu.getDisplacementX(), self.imu.getDisplacementY()
        )
        imu_confidence = 0.1

        total_confidence = sum([vis_confidence, odometry_confidence, imu_confidence])
        best_estimate = (
            vis_estimate * vis_confidence
            + odometry_estimate * odometry_confidence
            + imu_estimate * imu_confidence
        ) / total_confidence
        self.robot_pose = Pose2d(
            best_estimate,
            Rotation2d(self.imu.getFusedHeading()),
        )

        self.chassis.set_odometry(self.robot_pose)
        self.imu.resetDisplacement()

    def to_target(self) -> Tuple[float, float]:
        """Returns angle and distance to shoot at to hit the target"""
        # TODO: adjust for leading shots
        field_angle = math.atan2(
            self.robot_pose.Y(), self.robot_pose.X()
        )  # may need to flip / convert
        local_angle = field_angle - self.robot_pose.rotation().radians()
        return local_angle, self.robot_pose.translation().distance(Translation2d(0, 0))

    def get_pose_at(self, t: float) -> Pose2d:
        loops_ago = (wpilib.Timer.getFPGATimestamp() - t) / self.control_loop_wait_time
        if loops_ago >= len(self.pose_history):
            return (
                self.pose_history[-1] if len(self.pose_history) > 0 else self.robot_pose
            )
        return self.pose_history[loops_ago]
