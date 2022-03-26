import math
from magicbot import feedback, tunable
from components.turret import Turret
from components.chassis import Chassis
import wpilib
from utilities.scalers import scale_value
from photonvision import (
    PhotonCamera,
    PhotonUtils,
    LEDMode,
    SimVisionSystem,
    SimVisionTarget,
)
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d

import random


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

    def _camera_to_robot(self) -> Transform2d:
        turret_azimuth = self.turret.get_angle()
        return Transform2d(
            -(self.TURRET_OFFSET + math.cos(turret_azimuth) * self.CAMERA_OFFSET),
            -(math.sin(turret_azimuth) * self.CAMERA_OFFSET),
            -turret_azimuth,
        )

    def __init__(self) -> None:
        self.camera = PhotonCamera("gloworm")

        if wpilib.RobotBase.isSimulation():
            self.sim_vision_system = SimVisionSystem(
                "gloworm",
                75.76079874010732,
                math.degrees(self.CAMERA_PITCH),
                Transform2d(0, 0, 0),
                self.CAMERA_HEIGHT,
                10.0,
                1280,
                720,
                0.0,
            )
            self.sim_vision_system.addSimVisionTarget(
                SimVisionTarget(
                    targetPos=Pose2d(0, 0, 0),
                    targetHeightAboveGround=self.TARGET_HEIGHT,
                    targetWidth=self.GOAL_RADIUS * 2,
                    targetHeight=0.05,
                )
            )
            self.camera = self.sim_vision_system.cam

        self.camera.setLEDMode(LEDMode.kOn)
        self.max_std_dev = 0.4
        self.has_target = False
        self.distance = -1.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

    def setup(self) -> None:
        self.field_obj = self.field.getObject("vision_pose")

    def execute(self) -> None:
        if wpilib.RobotBase.isSimulation():
            # Create some vision target results
            pose = self.field.getRobotPose()
            range = pose.translation().distance(Translation2d(0, 0))
            if range < 0.1:
                return
            # Offset by the goal radius because the target is assumed to be flat
            x = pose.X()
            y = pose.Y()
            rot = pose.rotation()

            pose = Pose2d(
                x - x / range * self.GOAL_RADIUS, y - y / range * self.GOAL_RADIUS, rot
            )

            # The turret moves the camera so recalculate the transform
            # Also add some noise in the pitch due to the vibration of the flywheels
            self.sim_vision_system.moveCamera(
                newcameraToRobot=self._camera_to_robot(),
                newCamHeight=self.CAMERA_HEIGHT,
                newCamPitch=math.degrees(self.CAMERA_PITCH) + random.gauss(0.0, 0.5),
            )
            self.sim_vision_system.processFrame(pose)

        results = self.camera.getLatestResult()
        self.has_target = results.hasTargets()
        if not self.has_target:
            return
        timestamp = wpilib.Timer.getFPGATimestamp() - results.getLatency()
        self.target_pitch = math.radians(results.getBestTarget().getPitch())
        # PhotonVision has yaw reversed from our RH coordinate system
        self.target_yaw = -math.radians(results.getBestTarget().getYaw())

        # work out our field position when photo was taken
        turret_rotation = self.turret.get_angle_at(timestamp)
        robot_rotation = self.chassis.get_pose_at(timestamp).rotation()

        # angle from the robot to target
        target_angle = turret_rotation + self.target_yaw
        # distance from camera to middle of goal
        self.distance = (
            PhotonUtils.calculateDistanceToTarget(
                self.CAMERA_HEIGHT,
                self.TARGET_HEIGHT,
                self.CAMERA_PITCH,
                self.target_pitch,
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
        self.field_obj.setPose(vision_pose)

        if self.fuse_vision_observations:
            innovation = vision_pose.translation().distance(
                self.chassis.estimator.getEstimatedPosition().translation()
            )
            # Gate on innovation
            if self.gate_innovation and innovation > 2.0:
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
