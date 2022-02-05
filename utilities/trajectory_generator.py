from wpimath.geometry import Pose2d, Translation2d
from typing import List


def lerpPose(p1: Pose2d, p2: Pose2d, n1: float) -> Pose2d:
    n2 = 1 - n1
    translation = Translation2d(
        x=p1.translation().x * n2 + p2.translation().x * n1,
        y=p1.translation().y * n2 + p2.translation().y * n1,
    )
    rotation = p1.rotation() * n2 + p2.rotation() * n1
    return Pose2d(translation, rotation)


def lookupLinear(waypoints: List[Pose2d], d: float) -> Pose2d:
    """Finds the translation2d d meters along the path of straight line waypoints"""
    if d < 0:
        return waypoints[0]
    total_dist = 0
    for last, cur in zip(waypoints, waypoints[1:]):
        # distance to next waypoint
        cur_dist = last.translation().distance(cur.translation())
        if total_dist + cur_dist > d:
            return lerpPose(last, cur, d / cur_dist)
        total_dist += cur_dist
    return waypoints[-1]


def smoothPath(waypoints: List[Pose2d], look_around, d) -> Pose2d:
    """Samples from a group of waypoints in a way that creates smooth paths"""
    return lerpPose(
        lookupLinear(waypoints, d + look_around),
        lookupLinear(waypoints, d - look_around),
        0.5,
    )
