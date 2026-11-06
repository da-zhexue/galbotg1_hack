import math

from ctrl.state_utils import (
    orientation_error,
    pose_from_values,
    position_error,
)


def test_pose_error_helpers():
    first = pose_from_values([1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0])
    second = pose_from_values([1.0, 2.0, 4.0, 0.0, 0.0, 0.0, 1.0])
    assert math.isclose(position_error(first, second), 1.0)
    assert math.isclose(orientation_error(first, second), 0.0)


def test_pose_helpers_require_seven_values():
    try:
        pose_from_values([0.0] * 6)
    except ValueError:
        return
    raise AssertionError("pose_from_values accepted six values")
