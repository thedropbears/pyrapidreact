from wpimath.geometry import Pose2d
from typing import List

from utilities.scalers import scale_value


def totalLength(waypoints: List[Pose2d]) -> float:
    return sum(
        last.translation().distance(cur.translation())
        for last, cur in zip(waypoints, waypoints[1:])
    )


def lerpPose(p1: Pose2d, p2: Pose2d, n1: float) -> Pose2d:
    n2 = 1 - n1
    translation = p1.translation() * n2 + p2.translation() * n1
    rotation = p1.rotation() * n2 + p2.rotation() * n1
    return Pose2d(translation, rotation)


def lookupLinear(waypoints: List[Pose2d], dist: float) -> Pose2d:
    """Finds the translation2d dist meters along the path of straight line waypoints"""
    if dist < 0:
        return waypoints[0]
    total_dist = 0
    for last, cur in zip(waypoints, waypoints[1:]):
        # distance to next waypoint
        cur_dist = last.translation().distance(cur.translation())
        if total_dist + cur_dist > dist:
            return lerpPose(last, cur, (dist - total_dist) / cur_dist)
        total_dist += cur_dist
    return waypoints[-1]


def smoothPath(
    waypoints: List[Pose2d], look_around: float, dist: float, sample_count=10
) -> Pose2d:
    """Samples dist meters along waypoints in a way that creates smooth paths"""
    x, y, angle = 0, 0, 0
    for sample in range(sample_count):
        sample_d = dist + scale_value(
            sample, 0, sample_count, -look_around, look_around
        )
        sample_pose = lookupLinear(waypoints, sample_d)
        x += sample_pose.X()
        y += sample_pose.Y()
        angle += sample_pose.rotation().radians()
    return Pose2d(x / sample_count, y / sample_count, angle / sample_count)
