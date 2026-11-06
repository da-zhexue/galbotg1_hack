from __future__ import annotations

import pytest

from grabbox import GrabboxError, GrabBox
from grabbox.poses import (
    GRAB_HEAD_BASE_HEIGHT_M,
    LEFT_ARM_DROP_OVER_POSITION,
    LEFT_ARM_DROP_WAYPOINTS,
    LEFT_ARM_GRAB_POSITION,
    RIGHT_ARM_DROP_OVER_POSITION,
    RIGHT_ARM_DROP_WAYPOINTS,
    RIGHT_ARM_GRAB_POSITION,
    LEFT_GRAB_TCP_POSITION_M,
    RIGHT_GRAB_TCP_POSITION_M,
)
from galbot_sdk.g1 import ControlStatus, MotionStatus


class FakeRobot:
    def __init__(self) -> None:
        self.calls = []
        self.trajectories = []
        self.initialized = False
        self.shutdown_requested = False
        self.shutdown_waited = False
        self.destroyed = False
        self.transforms = {
            "head_base_link": [
                0.0,
                0.0,
                GRAB_HEAD_BASE_HEIGHT_M,
            ],
            "left_gripper_tcp_link": list(
                LEFT_GRAB_TCP_POSITION_M
            ),
            "right_gripper_tcp_link": list(
                RIGHT_GRAB_TCP_POSITION_M
            ),
        }

    def init(self):
        self.initialized = True
        return True

    def set_joint_positions(self, **kwargs):
        self.calls.append(("joints", kwargs))
        return ControlStatus.SUCCESS

    def set_gripper_command(self, **kwargs):
        self.calls.append(("gripper", kwargs))
        return ControlStatus.SUCCESS

    def execute_joint_trajectory(self, trajectory, is_blocking=True):
        self.trajectories.append((trajectory, is_blocking))
        return ControlStatus.SUCCESS

    def get_transform(
        self,
        target_frame,
        source_frame,
        timestamp_ns=0,
        timeout_ms=100,
    ):
        del target_frame, timestamp_ns, timeout_ms
        position = self.transforms.get(source_frame)
        if position is None:
            return [], 0
        return list(position) + [0.0, 0.0, 0.0, 1.0], 1

    def stop_trajectory_execution(self):
        return ControlStatus.SUCCESS

    def stop_base(self):
        return ControlStatus.SUCCESS

    def request_shutdown(self):
        self.shutdown_requested = True

    def wait_for_shutdown(self):
        self.shutdown_waited = True

    def destroy(self):
        self.destroyed = True


class FakeMotion:
    def __init__(self) -> None:
        self.plan_calls = []

    def init(self):
        return True

    def get_chain_joint_names(self, chain):
        return [f"{chain}_joint{i}" for i in range(1, 8)]

    def traj_plan(self, waypoints, params):
        self.plan_calls.append(("traj_plan", waypoints, params))
        return MotionStatus.SUCCESS, {}

    def status_to_string(self, status):
        return str(status)


def test_default_grab_and_drop():
    robot = FakeRobot()
    motion = FakeMotion()
    box = GrabBox(robot, motion, trajectory_step_s=0.1)

    box.default()
    box.grab()
    box.drop()

    joint_calls = [call for call in robot.calls if call[0] == "joints"]
    gripper_calls = [call for call in robot.calls if call[0] == "gripper"]
    assert len(joint_calls) >= 4
    assert motion.plan_calls
    assert any(
        call[1]["joint_names"] == list(
            [f"head_joint{i}" for i in range(1, 3)]
            + [f"left_arm_joint{i}" for i in range(1, 8)]
            + [f"right_arm_joint{i}" for i in range(1, 8)]
        )
        and call[1]["joint_positions"][2:9] == LEFT_ARM_GRAB_POSITION
        and call[1]["joint_positions"][9:16] == RIGHT_ARM_GRAB_POSITION
        for call in joint_calls
    )
    assert any(
        call[1]["width_m"] == 0.06 for call in gripper_calls
    )
    assert robot.trajectories
    trajectory, blocking = robot.trajectories[0]
    assert blocking is True
    assert len(trajectory.points) == len(LEFT_ARM_DROP_WAYPOINTS)
    assert len(trajectory.points) == len(RIGHT_ARM_DROP_WAYPOINTS)
    assert trajectory.points[0].joint_command_vec[0].position == (
        LEFT_ARM_DROP_WAYPOINTS[0][0]
    )
    assert LEFT_ARM_DROP_WAYPOINTS[0] == LEFT_ARM_GRAB_POSITION
    assert RIGHT_ARM_DROP_WAYPOINTS[0] == RIGHT_ARM_GRAB_POSITION


def test_default_uses_single_point_arm_planning():
    robot = FakeRobot()
    motion = FakeMotion()
    box = GrabBox(robot, motion)

    box.default()

    assert len(motion.plan_calls) == 1
    method, waypoints, params = motion.plan_calls[0]
    assert method == "traj_plan"
    assert len(waypoints) == 1
    assert len(waypoints[0]) == 2
    assert params.get_check_collision() is False
    assert params.enable_env_collision_check is False
    joint_calls = [call for call in robot.calls if call[0] == "joints"]
    assert not any(
        call[1]["joint_names"] == [
            f"left_arm_joint{i}" for i in range(1, 8)
        ]
        for call in joint_calls
    )


def test_drop_over_extends_hands_to_30cm_pose():
    robot = FakeRobot()
    box = GrabBox(robot, FakeMotion(), trajectory_step_s=0.1)

    box.drop_over()

    assert robot.trajectories
    arm_names = [
        f"left_arm_joint{index}" for index in range(1, 8)
    ] + [
        f"right_arm_joint{index}" for index in range(1, 8)
    ]
    assert any(
        call[1]["joint_names"] == arm_names
        and call[1]["joint_positions"]
        == LEFT_ARM_DROP_OVER_POSITION + RIGHT_ARM_DROP_OVER_POSITION
        for call in robot.calls
        if call[0] == "joints"
    )


def test_drop_over_after_drop_does_not_repeat_drop():
    robot = FakeRobot()
    box = GrabBox(robot, FakeMotion(), trajectory_step_s=0.1)

    box.drop()
    trajectory_count = len(robot.trajectories)
    box.drop_over(drop_first=False)

    assert len(robot.trajectories) == trajectory_count


def test_drop_rejects_arms_outside_grab_tolerance():
    robot = FakeRobot()
    robot.transforms["left_gripper_tcp_link"][1] += 0.05
    box = GrabBox(robot, FakeMotion())

    with pytest.raises(GrabboxError, match="left_gripper_tcp_link"):
        box.drop()

    assert not robot.calls


def test_drop_rejects_body_height_outside_grab_tolerance():
    robot = FakeRobot()
    robot.transforms["head_base_link"][2] += 0.05
    box = GrabBox(robot, FakeMotion())

    with pytest.raises(GrabboxError, match="body height"):
        box.drop()

    assert not robot.calls


def test_real_test_sequence_with_fake_robot(monkeypatch):
    from grabbox import real_test

    robot = FakeRobot()
    motion = FakeMotion()
    monkeypatch.setattr(real_test, "GalbotRobot", lambda: robot)
    monkeypatch.setattr(real_test.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(
        real_test,
        "GrabBox",
        lambda robot: GrabBox(robot, motion),
    )

    result = real_test.main(
        ["--yes", "--state", "all", "--settle-s", "0"]
    )

    assert result == 0
    assert robot.initialized
    assert robot.shutdown_requested
    assert robot.shutdown_waited
    assert robot.destroyed
    assert robot.trajectories
