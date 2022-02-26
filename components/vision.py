import math

from networktables import NetworkTables
from typing import Optional
from dataclasses import dataclass
from magicbot import feedback
from wpilib import Timer
from components.turret import Turret
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import navx
import wpilib

from utilities.functions import constrain_angle
from utilities.scalers import scale_value


@dataclass
class VisionData:
    #: The angle to the target in radians.
    angle: float

    #: The distance to the target in metres.
    distance: float

    # how confident we are that we are correct, is generally lower when the target is smaller or irregular
    fittness: float

    #: An arbitrary timestamp, in seconds,
    #: for when the vision system last obtained data.
    timestamp: float

    __slots__ = ("angle", "distance", "fittness", "timestamp")


class Vision:
    """Communicates with raspberry pi to get vision data"""

    SYSTEM_LAG_THRESHOLD = 0.200
    turret: Turret
    chassis: Chassis
    imu: navx.AHRS

    camera_offset = 0.316

    def __init__(self) -> None:

        self.nt = NetworkTables
        self.table = self.nt.getTable("/vision")
        self.vision_data_entry = self.table.getEntry("data")
        self.ping_time_entry = self.table.getEntry("ping")  # rio time
        self.rio_pong_time_entry = self.table.getEntry(
            "rio_pong"
        )  # rio time sent back with vision data
        self.raspi_pong_time_entry = self.table.getEntry(
            "raspi_pong"
        )  # raspbi time send with data
        self.latency_entry = self.table.getEntry("clock_offset")

        self.last_pong = Timer.getFPGATimestamp()
        self.last_data_timestamp = 0  # timestamp of last data

        self.vision_data: Optional[VisionData] = None

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
        """
        return self.vision_data

    @feedback
    def get_ange(self):
        # just feedback for debugging
        if self.vision_data is None:
            return -100
        return self.vision_data.angle

    def execute(self) -> None:
        self.recive_pong()
        self.ping()
        data = self.vision_data_entry.getDoubleArray(None)
        if data is not None:
            # add clock offset to vision timestamp
            self.vision_data = VisionData(
                data[0], data[1], data[2], data[3] + self.get_clocks_offset()
            )
            if not self.vision_data.timestamp == self.last_data_timestamp:
                # Get vision pose estimate
                # work out where the vision data was taken from based on histories
                camera_pose = self.get_vis_pose_at(self.vision_data.timestamp)
                # angle from target to robot in world space
                angle_from_target = constrain_angle(
                    camera_pose - self.vision_data.angle + math.pi
                )
                # work out where vision though it was when the image was taken
                vis_estimate = Translation2d(
                    distance=self.vision_data.distance,
                    angle=Rotation2d(angle_from_target),
                )
                # calcualte vision std dev
                # trust vision less the more outdated it is
                vis_age = wpilib.Timer.getFPGATimestamp() - self.vision_data.timestamp
                age_fit = max(0, scale_value(vis_age, 0, 0.2, 1, 0))
                # trust vision less the more it thinks we've moved (to reduce impact of false positives)
                innovation = vis_estimate.distance(
                    self.chassis.estimator.getEstimatedPosition().translation()
                )
                # will be 0 if innovation is over 1.5m
                innovation_fit = min(
                    1, max(0, scale_value(innovation, 0.25, 1.5, 1, 0))
                )
                # combined vision confidence is 0-1
                vis_confidence = self.vision_data.fittness * innovation_fit * age_fit
                if vis_confidence > 0.4:
                    vis_std_dev = 0.5 / vis_confidence
                    # pass vision pose estimate to chassis kalman filter
                    self.chassis.estimator.addVisionMeasurement(
                        Pose2d(vis_estimate, self.imu.getRotation2d()),
                        self.vision_data.timestamp,
                        vis_std_dev,
                    )
                self.last_data_timestamp = self.vision_data.timestamp
        self.nt.flush()

    @feedback
    def is_ready(self) -> bool:
        return self.system_lag_calculation() < self.SYSTEM_LAG_THRESHOLD

    @feedback
    def system_lag_calculation(self) -> float:
        if self.vision_data is not None:
            return Timer.getFPGATimestamp() - self.vision_data.timestamp
        else:
            return math.inf

    def ping(self) -> None:
        """Send a ping to the RasPi to determine the connection latency."""
        self.ping_time_entry.setDouble(Timer.getFPGATimestamp())

    def recive_pong(self) -> None:
        """Receive a pong from the RasPi to determine the connection latency."""
        rio_pong_time = self.rio_pong_time_entry.getDouble(0)
        if abs(rio_pong_time - self.last_pong) > 1e-4:  # Floating point comparison
            raspi_pong_time = self.raspi_pong_time_entry.getDouble(0)
            self.latency_entry.setDouble(rio_pong_time - raspi_pong_time)
            self.last_pong = rio_pong_time

    def get_clocks_offset(self) -> float:
        return self.latency_entry.getDouble(0)

    def get_vis_pose_at(self, t: float) -> Pose2d:
        """Gets where the camera was at t"""
        robot_pose = self.chassis.get_pose_at(t)
        turret_translation = self.chassis.robot_to_world(
            self.turret_offset, robot_pose
        ).translation()
        taken_at_angle = Rotation2d(
            robot_pose.rotation().radians() + self.turret.get_angle_at(t)
        )
        camera_translation = turret_translation + Translation2d(
            distance=self.camera_offset, angle=taken_at_angle
        )
        return Pose2d(
            camera_translation,
            taken_at_angle,
        )
