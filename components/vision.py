import math
from networktables import NetworkTables
from magicbot import feedback, tunable
from components.turret import Turret
from components.chassis import Chassis
import wpilib
from utilities.trajectory_generator import goal_to_field
from photonvision import PhotonCamera, PhotonUtils, LEDMode
from wpimath.geometry import Pose2d, Translation2d


class Vision:
    """Communicates with raspberry pi to get vision data"""

    PONG_DELAY_THRESHOLD = 1.000
    turret: Turret
    chassis: Chassis

    CAMERA_OFFSET = -0.1  # m from camera to centre of turret, measured from CAD
    TURRET_OFFSET = -0.15  # m from robot centre to turret centre, measured from CAD

    # camera angle from horizontal
    CAMERA_PITCH = math.radians(20)
    CAMERA_HEIGHT = 0.7
    TARGET_HEIGHT = 2.66
    GOAL_RAD = 0.61

    field: wpilib.Field2d

    fuse_vision_observations = tunable(True)
    gate_innovation = tunable(True)

    def __init__(self) -> None:
        self.nt = NetworkTables
        self.camera = PhotonCamera(self.nt, "cam_name")
        self.camera.setLEDMode(LEDMode.kOn)
        self.max_std_dev = 0.5
        self.has_target = False

    def execute(self):
        # gets vision data from camera
        results = self.camera.getLatestResult()
        if not results.hasTargets():
            self.has_target = False
            return
        self.has_target = True
        timestamp = wpilib.Timer.getFPGATimestamp() - results.getLatency()
        target_pitch = results.getBestTarget().getPitch()
        target_yaw = results.getBestTarget().getYaw()

        # work out our field position when photo was taken
        turret_rotation = self.turret.get_angle_at(timestamp)
        robot_rotation = self.chassis.get_pose_at(timestamp).rotation()
        # camera translation relative to robot
        camera_offset = Translation2d(
            self.TURRET_OFFSET + math.cos(turret_rotation) * self.CAMERA_OFFSET,
            math.sin(turret_rotation) * self.CAMERA_OFFSET,
        ).rotateBy(robot_rotation)
        # angle from the robot to target field relative
        field_angle = robot_rotation.radians() + turret_rotation + target_yaw
        # distnace from camera to middle of goal
        distance = (
            PhotonUtils.calculateDistanceToTarget(
                self.CAMERA_HEIGHT, self.TARGET_HEIGHT, self.CAMERA_PITCH, target_pitch
            )
            + self.GOAL_RAD
        )
        vision_camera_pose = Translation2d(
            distance=distance, angle=field_angle + math.pi
        )
        vision_pose = vision_camera_pose - camera_offset
        self.field_obj.setPose(goal_to_field(vision_pose))

        if self.fuse_vision_observations:
            innovation = vision_pose.distance(
                self.chassis.estimator.getEstimatedPosition().translation()
            )
            # Gate on innovation
            if self.gate_innovation and innovation > 5.0:
                return
            self.chassis.estimator.addVisionMeasurement(
                Pose2d(vision_pose, self.chassis.get_rotation()),
                timestamp,
                (self.max_std_dev, self.max_std_dev, 0.001),
            )

    @feedback
    def is_ready(self) -> bool:
        return self.has_target
