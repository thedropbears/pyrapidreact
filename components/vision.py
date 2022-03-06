import math

from networktables import NetworkTables
from typing import Optional
from dataclasses import dataclass
from magicbot import feedback, tunable
from wpilib import Timer
from components.turret import Turret
from components.chassis import Chassis
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
import wpilib

from utilities.trajectory_generator import goal_to_field


@dataclass
class VisionData:
    #: The angle to the target in radians.
    angle: float

    #: The distance to the target in metres.
    distance: float

    # how confident we are that we are correct, is generally lower when the target is smaller or irregular
    fitness: float

    #: An arbitrary timestamp, in seconds,
    #: for when the vision system last obtained data.
    timestamp: float

    __slots__ = ("angle", "distance", "fitness", "timestamp")


class Vision:
    """Communicates with raspberry pi to get vision data"""

    SYSTEM_LAG_THRESHOLD = 0.200
    turret: Turret
    chassis: Chassis

    CAMERA_OFFSET = 0.35  # m from camera to centre of turret, measured from CAD
    TURRET_OFFSET = 0.15  # m from robot centre to turret centre, measured from CAD

    field: wpilib.Field2d
    gate_innovation = tunable(False)
    # how long we are allowed to not see a target when we expect to before we start scanning
    timeout = 10
    horiz_fov = math.radians(40)

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
        self.last_data_timestamp = Timer.getFPGATimestamp()  # timestamp of last data

        self.vision_data: Optional[VisionData] = None

        self.fuse_vision_observations = tunable(True)
        # if we're sure we lost the target
        self.lost_target = False
        self.has_target = False
        # how long we've gone without losing the target
        self.target_since = Timer.getFPGATimestamp()

    def setup(self) -> None:
        self.field_obj = self.field.getObject("vision_pose")

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
        """
        return self.vision_data

    @feedback
    def angle(self) -> float:
        # just feedback for debugging
        if self.vision_data is None:
            return -100
        return self.vision_data.angle

    def execute(self) -> None:
        self.receive_pong()
        self.ping()
        self.nt.flush()
        data = self.vision_data_entry.getDoubleArray(None)
        if data is None:
            return

        # add clock offset to vision timestamp
        self.vision_data = VisionData(
            data[0], data[1], data[2], data[3] + self.get_clocks_offset()
        )
        if self.vision_data.timestamp == self.last_data_timestamp:
            self.target_since = Timer.getFPGATimestamp()
            self.has_target = False
            if (
                self.expects_target()
                and Timer.getFPGATimestamp() - self.last_data_timestamp > self.timeout
            ):
                self.lost_target = True
            return

        self.lost_target = False
        self.has_target = True
        self.last_data_timestamp = self.vision_data.timestamp

        # Get vision pose estimate
        # work out where the vision data was taken from based on histories
        t = self.vision_data.timestamp
        turret_angle = self.turret.get_angle_at(t)
        chassis_heading = self.chassis.get_pose_at(t).rotation().radians()

        # Ranges from vision are from centre of goal to the camera
        # Add the offset to get range to the centre of the turret
        range = self.vision_data.distance + self.CAMERA_OFFSET

        vision_pose = pose_from_vision(
            range, turret_angle + self.vision_data.angle, chassis_heading
        )

        self.field_obj.setPose(goal_to_field(vision_pose))

        if self.fuse_vision_observations:
            innovation = vision_pose.translation().distance(
                self.chassis.estimator.getEstimatedPosition().translation()
            )
            # Gate on innovation
            if self.gate_innovation and innovation > 1.0:
                return
            # Come up with a position std dev from the fitness reported
            # When the target is near the edge, the estimate of range is worse
            pos_std_dev = 0.1 + 0.5 * (1.0 - self.vision_data.fitness)
            # TODO Can we be smarter and find different values for x and y based on robot orientation?
            self.chassis.estimator.addVisionMeasurement(
                vision_pose,
                self.vision_data.timestamp,
                (pos_std_dev, pos_std_dev, 0.001),
            )

    @feedback
    def is_ready(self) -> bool:
        return self.system_lag_calculation() < self.SYSTEM_LAG_THRESHOLD

    def target_age(self) -> float:
        return Timer.getFPGATimestamp() - self.target_since

    @feedback
    def system_lag_calculation(self) -> float:
        if self.vision_data is not None:
            return Timer.getFPGATimestamp() - self.vision_data.timestamp
        else:
            return math.inf

    def ping(self) -> None:
        """Send a ping to the RasPi to determine the connection latency."""
        self.ping_time_entry.setDouble(Timer.getFPGATimestamp())

    def receive_pong(self) -> None:
        """Receive a pong from the RasPi to determine the connection latency."""
        rio_pong_time = self.rio_pong_time_entry.getDouble(0)
        if abs(rio_pong_time - self.last_pong) > 1e-4:  # Floating point comparison
            raspi_pong_time = self.raspi_pong_time_entry.getDouble(0)
            self.latency_entry.setDouble(rio_pong_time - raspi_pong_time)
            self.last_pong = rio_pong_time

    def get_clocks_offset(self) -> float:
        return self.latency_entry.getDouble(0)

    @feedback
    def expects_target(self):
        """Calculates if we expect to see a target"""
        estimator_pose = self.chassis.get_pose()
        expected_angle = math.atan2(-estimator_pose.Y(), -estimator_pose.X())
        camera_angle = estimator_pose.rotation().radians() + self.turret.get_angle()
        distance = self.chassis.get_pose().translation().norm()
        return abs(camera_angle - expected_angle) < self.horiz_fov and distance > 3


def pose_from_vision(
    range: float, turret_angle: float, chassis_heading: float
) -> Pose2d:
    # Assume robot is at origin
    location = Translation2d(
        -Vision.TURRET_OFFSET + math.cos(turret_angle) * range,
        math.sin(turret_angle) * range,
    )
    # Rotate by the chassis heading
    location = location.rotateBy(Rotation2d(chassis_heading))
    # Now transform the pose so that the target is at the origin
    return Pose2d(-location.X(), -location.Y(), chassis_heading)
