from hypothesis.strategies import from_type
from hypothesis import given

from wpimath.geometry import Pose2d
from utilities.trajectory_generator import lerpPose


@given(p1=from_type(Pose2d))
def test_lerp_zero(p1):
    assert lerpPose(p1, Pose2d(1, 1, 1), 0) == p1


@given(p1=from_type(Pose2d))
def test_lerp_one(p1):
    assert lerpPose(p1, Pose2d(1, 1, 1), 1) == Pose2d(1, 1, 1)


@given(p1=from_type(Pose2d))
def test_lerp_half(p1):
    assert lerpPose(p1, Pose2d(1, 1, 1), 0.5) == Pose2d(0.5, 0.5, 0.5)


# @given(x=floats)
# def test_single(x):
#     assert lerpPose(Pose2d(x, x, x), Pose2d(0, 0, 0), y) == Pose2d(x*y, x*y, x*y)
