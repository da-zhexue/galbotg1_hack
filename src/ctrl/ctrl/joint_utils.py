"""Shared joint names and conversions for the Galbot control bridge."""

from __future__ import annotations

import math
from collections.abc import Sequence

TOPIC_WHEELS = "/ctrl/wheels"
TOPIC_LEG = "/ctrl/leg"
TOPIC_HEAD = "/ctrl/head"
TOPIC_LEFT_HAND = "/ctrl/left_hand"
TOPIC_RIGHT_HAND = "/ctrl/right_hand"
TOPIC_GRIPPER = "/ctrl/gripper"

WHEEL_JOINT_NAMES = (
    "wheel1_joint",
    "wheel2_joint",
    "wheel3_joint",
    "wheel4_joint",
)
WHEEL_PASSIVE_JOINT_NAMES = tuple(
    f"wheel_{wheel}_passive_{roller}_joint"
    for wheel in range(1, 5)
    for roller in range(10)
)
LEG_JOINT_NAMES = tuple(f"leg_joint{index}" for index in range(1, 6))
HEAD_JOINT_NAMES = ("head_joint1", "head_joint2")
LEFT_ARM_JOINT_NAMES = tuple(
    f"left_arm_joint{index}" for index in range(1, 8)
)
RIGHT_ARM_JOINT_NAMES = tuple(
    f"right_arm_joint{index}" for index in range(1, 8)
)
LEFT_GRIPPER_JOINT_NAME = "left_gripper_joint"
RIGHT_GRIPPER_JOINT_NAME = "right_gripper_joint"

CONTROLLED_JOINT_NAMES = (
    WHEEL_JOINT_NAMES
    + LEG_JOINT_NAMES
    + HEAD_JOINT_NAMES
    + LEFT_ARM_JOINT_NAMES
    + RIGHT_ARM_JOINT_NAMES
    + (LEFT_GRIPPER_JOINT_NAME, RIGHT_GRIPPER_JOINT_NAME)
)

VISUALIZED_JOINT_NAMES = CONTROLLED_JOINT_NAMES + WHEEL_PASSIVE_JOINT_NAMES

GROUP_PREFIXES = {
    TOPIC_LEG: ("leg_joint",),
    TOPIC_HEAD: ("head_joint",),
    TOPIC_LEFT_HAND: ("left_arm_joint",),
    TOPIC_RIGHT_HAND: ("right_arm_joint",),
}

# Geometry used by galbot_one_golf_description:
# wheel centers are at (+/-0.176493, +/-0.176493) m with a 0.071834 m radius.
WHEEL_RADIUS_M = 0.071834

# Pseudoinverse mapping from wheel angular velocities [rad/s] to base velocity.
# The URDF wheel yaw angles are -135, -45, +45, and +135 degrees.
_WHEEL_TO_BASE = (
    (0.025397154259877, 0.025397154259877,
     -0.025397154259877, -0.025397154259877),
    (-0.025397154259877, 0.025397154259877,
     0.025397154259877, -0.025397154259877),
    (-0.071949466154117, -0.071949466154117,
     -0.071949466154117, -0.071949466154117),
)


def validate_joint_names(
    topic: str,
    joint_names: Sequence[str],
    positions: Sequence[float],
) -> str | None:
    """Return an error string when a group command is invalid."""
    if not joint_names:
        return "joint name list is empty"
    if len(joint_names) != len(positions):
        return (
            f"joint name count {len(joint_names)} does not match "
            f"position count {len(positions)}"
        )
    if len(set(joint_names)) != len(joint_names):
        return "joint name list contains duplicates"
    if not all(math.isfinite(value) for value in positions):
        return "joint positions contain NaN or infinity"

    prefixes = GROUP_PREFIXES.get(topic)
    if prefixes is None:
        return f"unsupported joint command topic: {topic}"
    invalid = [
        name for name in joint_names
        if not any(name.startswith(prefix) for prefix in prefixes)
    ]
    if invalid:
        return f"joints do not belong to {topic}: {invalid}"
    return None


def wheel_velocities_to_base(
    wheel_velocity: Sequence[float],
) -> tuple[float, float, float]:
    """Convert four wheel angular velocities to a base planar velocity."""
    if len(wheel_velocity) != 4:
        raise ValueError("wheel_velocity must contain exactly four values")
    if not all(math.isfinite(value) for value in wheel_velocity):
        raise ValueError("wheel_velocity contains NaN or infinity")

    return tuple(
        sum(row[index] * wheel_velocity[index] for index in range(4))
        for row in _WHEEL_TO_BASE
    )


def gripper_width_to_joint(
    width_m: float,
    max_width_m: float,
    open_angle_rad: float,
) -> float:
    """Map a parallel-jaw opening width to the primary gripper joint angle."""
    if max_width_m <= 0.0 or open_angle_rad <= 0.0:
        raise ValueError("gripper limits must be positive")
    if not math.isfinite(width_m):
        raise ValueError("gripper width contains NaN or infinity")
    ratio = min(max(width_m / max_width_m, 0.0), 1.0)
    return ratio * open_angle_rad
