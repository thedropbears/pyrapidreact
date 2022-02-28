import pytest
import math

from components.vision import pose_from_vision, Vision

def test_pose_calcs():
    # Straight ahead
    pose = pose_from_vision(10, 0, 0)
    assert pose.X() == pytest.approx(-10 + Vision.TURRET_OFFSET)
    assert pose.Y() == pytest.approx(0.0)

    # Looking left
    pose = pose_from_vision(10, math.pi/2, 0)
    assert pose.X() == pytest.approx(Vision.TURRET_OFFSET)
    assert pose.Y() == pytest.approx(-10.0)

    # Rotate the robot 90 degrees
    pose = pose_from_vision(10, 0, math.pi/2)
    assert pose.X() == pytest.approx(0.0)
    assert pose.Y() == pytest.approx(-10 + Vision.TURRET_OFFSET)

    pose = pose_from_vision(10, math.pi/2, math.pi/2)
    assert pose.X() == pytest.approx(10.0)
    assert pose.Y() == pytest.approx(Vision.TURRET_OFFSET)

