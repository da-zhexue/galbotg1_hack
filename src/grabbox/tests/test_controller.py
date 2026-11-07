from __future__ import annotations

import pytest

from grabbox import GrabboxError, GrabBox
from grabbox.poses import (
    GRAB_HEAD_BASE_HEIGHT_M,
    LEFT_ARM_DROP_WAYPOINTS,
    LEFT_ARM_GRAB_POSITION,
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
        self.add_obstacle_calls = []
        self.remove_obstacle_calls = []
        self.plan_status = MotionStatus.SUCCESS
        self.events = []

    def init(self):
        return True

    def get_chain_joint_names(self, chain):
        if chain == "head":
            return [f"{chain}_joint{i}" for i in range(1, 3)]
        return [f"{chain}_joint{i}" for i in range(1, 8)]

    def traj_plan(self, waypoints, params):
        self.plan_calls.append(("traj_plan", waypoints, params))
        return self.plan_status, {}

    def motion_plan(self, waypoints, params):
        self.events.append("motion_plan")
        self.plan_calls.append(("motion_plan", waypoints, params))
        return self.plan_status, {}

    def add_obstacle(self, **kwargs):
        self.events.append("add_obstacle")
        self.add_obstacle_calls.append(kwargs)
        return MotionStatus.SUCCESS

    def remove_obstacle(self, obstacle_id):
        self.events.append("remove_obstacle")
        self.remove_obstacle_calls.append(obstacle_id)
        return MotionStatus.SUCCESS

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
    grab_plan_calls = [
        call for call in motion.plan_calls if call[0] == "motion_plan"
    ]
    assert len(grab_plan_calls) == 1
    _, waypoints, params = grab_plan_calls[0]
    assert len(waypoints) == 1
    assert [target.chain_name for target in waypoints[0]] == [
        "head",
        "left_arm",
        "right_arm",
    ]
    assert params.get_check_collision() is True
    assert params.enable_env_collision_check is True
    assert motion.add_obstacle_calls == [{
        "obstacle_id": "grab_front_table",
        "obstacle_type": "box",
        "pose": [0.5, 0.0, 0.375, 0.0, 0.0, 0.0, 1.0],
        "scale": [0.6, 2.0, 0.75],
        "target_frame": "base_link",
    }]
    assert motion.remove_obstacle_calls == ["grab_front_table"]
    assert motion.events == [
        "add_obstacle",
        "motion_plan",
        "remove_obstacle",
    ]
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


def test_grab_removes_obstacle_when_planning_fails():
    robot = FakeRobot()
    motion = FakeMotion()
    motion.plan_status = MotionStatus.FAULT
    box = GrabBox(robot, motion)

    with pytest.raises(GrabboxError, match="grab motion planning failed"):
        box.grab()

    assert motion.add_obstacle_calls
    assert motion.remove_obstacle_calls == ["grab_front_table"]


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
