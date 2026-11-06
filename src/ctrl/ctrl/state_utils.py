"""Utilities for robot state snapshots and pose comparison."""

from __future__ import annotations

import math
from collections.abc import Sequence

from geometry_msgs.msg import Point, Pose, Quaternion


def pose_from_values(values: Sequence[float]) -> Pose:
    """Build a geometry_msgs/Pose from [x, y, z, qx, qy, qz, qw]."""
    if len(values) != 7:
        raise ValueError("pose must contain seven values")
    pose = Pose()
    pose.position = Point(
        x=float(values[0]),
        y=float(values[1]),
        z=float(values[2]),
    )
    pose.orientation = Quaternion(
        x=float(values[3]),
        y=float(values[4]),
        z=float(values[5]),
        w=float(values[6]),
    )
    return pose


def position_error(real: Pose, simulated: Pose) -> float:
    """Return Euclidean position error in meters."""
    return math.sqrt(
        (real.position.x - simulated.position.x) ** 2
        + (real.position.y - simulated.position.y) ** 2
        + (real.position.z - simulated.position.z) ** 2
    )


def orientation_error(real: Pose, simulated: Pose) -> float:
    """Return the shortest quaternion angle error in radians."""
    real_q = real.orientation
    sim_q = simulated.orientation
    dot = (
        real_q.x * sim_q.x
        + real_q.y * sim_q.y
        + real_q.z * sim_q.z
        + real_q.w * sim_q.w
    )
    dot = min(abs(dot), 1.0)
    return 2.0 * math.acos(dot)
