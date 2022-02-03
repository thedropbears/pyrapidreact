import math

from networktables import NetworkTables
from typing import Optional
from dataclasses import dataclass
from magicbot import feedback
from wpilib import Timer


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

        self.vision_data: Optional[VisionData] = None

    def get_data(self) -> Optional[VisionData]:
        """Returns the latest vision data.

        Returns None if there is no vision data.
        """
        return self.vision_data

    def execute(self) -> None:
        self.recive_pong()
        self.ping()
        data = self.vision_data_entry.getDoubleArray(None)
        if data is not None:
            self.vision_data = VisionData(
                data[0], data[1], data[2], data[3] + self.get_clocks_offset()
            )
            # add clock offset to vision timestamp

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