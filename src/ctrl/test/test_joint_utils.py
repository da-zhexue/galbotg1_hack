import math

from ctrl.joint_utils import (
    LEFT_ARM_GRAB_POSITION,
    RIGHT_ARM_GRAB_POSITION,
    TOPIC_HEAD,
    VISUALIZED_JOINT_NAMES,
    WHEEL_PASSIVE_JOINT_NAMES,
    gripper_width_to_joint,
    validate_joint_names,
    wheel_velocities_to_base,
)


def test_wheel_velocity_conversion_round_trip_axes():
    vx, vy, wz = wheel_velocities_to_base(
        [9.84362254, 9.84362254, -9.84362254, -9.84362254]
    )
    assert math.isclose(vx, 1.0, abs_tol=1e-6)
    assert math.isclose(vy, 0.0, abs_tol=1e-6)
    assert math.isclose(wz, 0.0, abs_tol=1e-6)


def test_joint_group_validation():
    assert validate_joint_names(
        TOPIC_HEAD,
        ["head_joint1", "head_joint2"],
        [0.1, 0.2],
    ) is None
    assert validate_joint_names(
        TOPIC_HEAD,
        ["left_arm_joint1"],
        [0.1],
    ) is not None


def test_gripper_width_clamps():
    assert gripper_width_to_joint(-1.0, 0.12, 1.703) == 0.0
    assert math.isclose(
        gripper_width_to_joint(0.06, 0.12, 1.703),
        0.8515,
        abs_tol=1e-6,
    )
    assert math.isclose(
        gripper_width_to_joint(1.0, 0.12, 1.703),
        1.703,
        abs_tol=1e-6,
    )


def test_all_wheel_passive_joints_are_visualized():
    assert len(WHEEL_PASSIVE_JOINT_NAMES) == 40
    assert WHEEL_PASSIVE_JOINT_NAMES[0] == "wheel_1_passive_0_joint"
    assert WHEEL_PASSIVE_JOINT_NAMES[-1] == "wheel_4_passive_9_joint"
    assert set(WHEEL_PASSIVE_JOINT_NAMES).issubset(
        VISUALIZED_JOINT_NAMES
    )


def test_grab_left_arm_is_opposite_of_right_arm():
    assert len(RIGHT_ARM_GRAB_POSITION) == 7
    assert LEFT_ARM_GRAB_POSITION == [
        -value for value in RIGHT_ARM_GRAB_POSITION
    ]
