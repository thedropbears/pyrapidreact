from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from typing import List

from utilities.scalers import scale_value


def totalLength(waypoints: List[Pose2d]) -> float:
    return sum(
        last.translation().distance(cur.translation())
        for last, cur in zip(waypoints, waypoints[1:])
    )


def lerpPose(p1: Pose2d, p2: Pose2d, n1: float) -> Pose2d:
    n2 = 1 - n1
    translation = Translation2d(
        x=p1.translation().x * n2 + p2.translation().x * n1,
        y=p1.translation().y * n2 + p2.translation().y * n1,
    )
    rotation = p1.rotation().radians() * n2 + p2.rotation().radians() * n1
    return Pose2d(translation, Rotation2d(rotation))


def lookupLinear(waypoints: List[Pose2d], d: float) -> Pose2d:
    """Finds the translation2d d meters along the path of straight line waypoints"""
    if d < 0:
        return waypoints[0]
    total_dist = 0
    for last, cur in zip(waypoints, waypoints[1:]):
        # distance to next waypoint
        cur_dist = last.translation().distance(cur.translation())
        if total_dist + cur_dist > d:
            return lerpPose(last, cur, (d - total_dist) / cur_dist)
        total_dist += cur_dist
    return waypoints[-1]


def smoothPath(waypoints: List[Pose2d], look_around, d, sample_count=10) -> Pose2d:
    """Samples from a group of waypoints in a way that creates smooth paths"""
    x, y, o = 0, 0, 0
    for sample in range(sample_count):
        sample_d = d + scale_value(sample, 0, sample_count, -look_around, look_around)
        sample_pose = lookupLinear(waypoints, sample_d)
        x += sample_pose.X()
        y += sample_pose.Y()
        o += sample_pose.rotation().radians()
    return Pose2d(x / sample_count, y / sample_count, o / sample_count)
