from collections import deque
from components.chassis import Chassis
from components.turret import Turret
from components.vision import Vision
from typing import Tuple
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from utilities.functions import constrain_angle
from utilities.trajectory_generator import goal_to_field
import navx
import wpilib
import math

from utilities.scalers import scale_value


class TargetEstimator:
    """Fuses data from vision, odometry, the imu and other sources
    to estimate where the robot is on the field and where you should aim to hit the target when moving
    """

    vision: Vision
    imu: navx.AHRS
    chassis: Chassis
    turret: Turret

    control_loop_wait_time: float
    field: wpilib.Field2d

    def __init__(self) -> None:
        self.robot_pose: Pose2d = Pose2d(-2, -1, -math.pi / 2)
        self.pose_history: deque = deque([], maxlen=100)
        # last total displacement
        self.last_imu = Translation2d()
        self.last_odometry = Translation2d()

    def setup(self):
        self.field_robot = self.field.getRobotObject()
        self.field_robot.setPoses([self.robot_pose, self.robot_pose])

    def execute(self) -> None:
        # create time compensated pose estimate from vision
        vis_data = self.vision.get_data()
        if vis_data is not None:
            # work out where the vision data was taken from based on histories
            vis_taken_at = self.get_vis_pose_at(vis_data.timestamp)
            # angle from target to robot in world space
            vis_angle_from_target = constrain_angle(
                vis_taken_at.rotation().radians() - vis_data.angle + math.pi
            )
            # work out where vision though it was when the image was taken
            vis_old_estimate = Translation2d(
                distance=vis_data.distance, angle=Rotation2d(vis_angle_from_target)
            )
            # the difference between where we though we were and where vision though we
            # were at the point in time when the image was taken
            error = vis_old_estimate - vis_taken_at.translation()
            # assume error has remained constant since vision data
            vis_estimate = error + self.robot_pose.translation()
            # trust vision less the more outdated it is
            vis_age = wpilib.Timer.getFPGATimestamp() - vis_data.timestamp
            age_fit = max(0, scale_value(vis_age, 0, 0.2, 1, 0))
            # trust vision less the more it thinks we've moved (to reduce impact of false positives)
            diff = vis_estimate.distance(self.robot_pose.translation())
            diff_fit = max(0, scale_value(diff, 0.25, 1.5, 1, 0.1))
            # combined vision confidence is 0-1
            vis_confidence = 0 * vis_data.fittness * diff_fit * age_fit
        else:
            vis_estimate = Translation2d(0, 0)
            vis_confidence = 0

        # gets how far odometry thinks we have moved since last control loop
        odometry_displacement = self.chassis.odometry.getPose().translation()
        odometry_estimate = (
            self.robot_pose.translation() + odometry_displacement - self.last_odometry
        )
        total_accel = math.sqrt(
            self.imu.getRawAccelX() ** 2 + self.imu.getRawAccelY() ** 2
        )
        odometry_confidence = max(0.5, 2 - total_accel)

        imu_displacement = Translation2d(
            self.imu.getDisplacementX(), self.imu.getDisplacementY()
        )
        imu_estimate = self.robot_pose.translation() + imu_displacement - self.last_imu
        imu_confidence = 0

        total_confidence = vis_confidence + odometry_confidence + imu_confidence
        best_estimate = (
            vis_estimate * vis_confidence
            + odometry_estimate * odometry_confidence
            + imu_estimate * imu_confidence
        ) / total_confidence
        self.robot_pose = Pose2d(
            best_estimate,
            self.imu.getRotation2d(),
        )

        self.field_robot.setPoses(
            [
                goal_to_field(x)
                for x in [
                    Pose2d(
                        imu_displacement,
                        self.imu.getRotation2d(),
                    ),
                    Pose2d(
                        odometry_displacement,
                        self.imu.getRotation2d(),
                    ),
                    Pose2d(
                        vis_estimate,
                        self.imu.getRotation2d(),
                    ),
                ]
            ]
        )

        self.last_imu = imu_displacement
        self.last_odometry = odometry_displacement

    def to_target(self) -> Tuple[float, float]:
        """Returns angle and distance to shoot at to hit the target"""
        # TODO: adjust for leading shots
        field_angle = math.atan2(
            self.robot_pose.Y(), self.robot_pose.X()
        )  # may need to flip / convert
        local_angle = field_angle - self.robot_pose.rotation().radians()
        return local_angle, self.robot_pose.translation().distance(Translation2d(0, 0))

    def to_x(self) -> Tuple[float, float]:
        """Returns the positive x direction on the field to test turret"""
        return -self.robot_pose.rotation().radians(), 2

    def get_pose_at(self, t: float) -> Pose2d:
        """Gets where the robot was at t"""
        # loops_ago = int((wpilib.Timer.getFPGATimestamp() - t) / self.control_loop_wait_time)
        # if loops_ago >= len(self.pose_history):
        #     return (
        #         self.pose_history[-1] if len(self.pose_history) > 0 else self.robot_pose
        #     )
        # if loops_ago < 0:
        #     return self.robot_pose
        # print(loops_ago)
        # return self.pose_history[loops_ago]
        return self.robot_pose

    def get_vis_pose_at(self, t: float) -> Pose2d:
        """Gets where the camera was at t"""
        vis_taken_at_pose = self.get_pose_at(t)
        vis_taken_at_angle = (
            vis_taken_at_pose.rotation().radians() + self.turret.get_angle_at(t)
        )
        # TODO: currently uses center of robot, could offset by turret position on robot and
        # calculate exactly where the camera is based on turret angle
        return Pose2d(vis_taken_at_pose.translation(), Rotation2d(vis_taken_at_angle))

    def set_pose(self, pose: Pose2d):
        self.robot_pose = pose
        self.chassis.set_odometry(pose)
        self.imu.resetDisplacement()
