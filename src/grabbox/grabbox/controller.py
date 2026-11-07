"""ROS-free state controller using the official Galbot Python SDK."""

from __future__ import annotations

import math
from collections.abc import Sequence

import galbot_sdk.g1 as gm

from .poses import (
    GRIPPER_DEFAULT_WIDTH_M,
    GRIPPER_EFFORT,
    GRIPPER_GRAB_WIDTH_M,
    GRIPPER_VELOCITY_MPS,
    GRAB_HEAD_BASE_HEIGHT_M,
    HEAD_DEFAULT_POSITION,
    LEFT_ARM_DEFAULT_POSITION,
    LEFT_ARM_DROP_OVER_POSITION,
    LEFT_ARM_DROP_WAYPOINTS,
    LEFT_ARM_GRAB_POSITION,
    LEFT_GRAB_TCP_POSITION_M,
    LEG_DEFAULT_POSITION,
    LEG_GRAB_POSITION,
    LEG_DROP_POSITION,
    RIGHT_ARM_DEFAULT_POSITION,
    RIGHT_ARM_DROP_OVER_POSITION,
    RIGHT_ARM_DROP_WAYPOINTS,
    RIGHT_ARM_GRAB_POSITION,
    RIGHT_GRAB_TCP_POSITION_M,
)

LEG_JOINT_NAMES = tuple(f"leg_joint{index}" for index in range(1, 6))
HEAD_JOINT_NAMES = ("head_joint1", "head_joint2")
LEFT_ARM_JOINT_NAMES = tuple(
    f"left_arm_joint{index}" for index in range(1, 8)
)
RIGHT_ARM_JOINT_NAMES = tuple(
    f"right_arm_joint{index}" for index in range(1, 8)
)

GRAB_OBSTACLE_ID = "grab_front_table"
GRAB_OBSTACLE_POSE = (0.5, 0.0, 0.375, 0.0, 0.0, 0.0, 1.0)
GRAB_OBSTACLE_SCALE = (0.6, 2.0, 0.75)


class GrabboxError(RuntimeError):
    """Raised when an SDK command does not complete successfully."""


class GrabBox:
    """Control default, grab, and drop states through an initialized SDK robot."""

    def __init__(
        self,
        robot: gm.GalbotRobot,
        motion: gm.GalbotMotion | None = None,
        *,
        speed_rad_s: float = 0.2,
        timeout_s: float = 40.0,
        planning_timeout_s: float = 120.0,
        trajectory_step_s: float = 0.1,
        drop_arm_tolerance_m: float = 0.03,
        drop_body_tolerance_m: float = 0.03,
    ) -> None:
        if robot is None:
            raise ValueError("robot must be a GalbotRobot instance")
        self.robot = robot
        self.motion = motion if motion is not None else gm.GalbotMotion()
        if not self.motion.init():
            raise GrabboxError("GalbotMotion initialization failed")
        self.speed_rad_s = float(speed_rad_s)
        self.timeout_s = float(timeout_s)
        self.planning_timeout_s = float(planning_timeout_s)
        self.trajectory_step_s = float(trajectory_step_s)
        self.drop_arm_tolerance_m = float(drop_arm_tolerance_m)
        self.drop_body_tolerance_m = float(drop_body_tolerance_m)
        if self.drop_arm_tolerance_m < 0.0:
            raise ValueError("drop_arm_tolerance_m must be non-negative")
        if self.drop_body_tolerance_m < 0.0:
            raise ValueError("drop_body_tolerance_m must be non-negative")

    def _require_success(self, operation: str, status: gm.ControlStatus) -> None:
        if status != gm.ControlStatus.SUCCESS:
            raise GrabboxError(f"{operation} failed: {status}")

    def _set_joints(
        self,
        joint_names: Sequence[str],
        positions: Sequence[float],
        *,
        blocking: bool = True,
    ) -> None:
        status = self.robot.set_joint_positions(
            joint_positions=list(positions),
            joint_groups=[],
            joint_names=list(joint_names),
            is_blocking=blocking,
            speed_rad_s=self.speed_rad_s,
            timeout_s=self.timeout_s,
        )
        self._require_success(
            f"set_joint_positions({len(joint_names)} joints)", status
        )

    def _set_gripper(self, width_m: float) -> None:
        for side in ("left", "right"):
            status = self.robot.set_gripper_command(
                end_effector=f"{side}_gripper",
                width_m=width_m,
                velocity_mps=GRIPPER_VELOCITY_MPS,
                effort=GRIPPER_EFFORT,
                is_blocking=True,
            )
            self._require_success(f"set_{side}_gripper", status)

    def _set_default_legs(self) -> None:
        self._set_joints(LEG_JOINT_NAMES, LEG_DEFAULT_POSITION)

    def _set_grab_legs(self) -> None:
        self._set_joints(LEG_JOINT_NAMES, LEG_GRAB_POSITION)

    def _set_drop_over_arms(self) -> None:
        names = LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        positions = (
            LEFT_ARM_DROP_OVER_POSITION
            + RIGHT_ARM_DROP_OVER_POSITION
        )
        self._set_joints(names, positions)

    def _read_frame_position(
        self, frame: str
    ) -> tuple[float, float, float]:
        try:
            values, timestamp_ns = self.robot.get_transform(
                "base_link",
                frame,
                timestamp_ns=0,
                timeout_ms=500,
            )
        except Exception as error:
            raise GrabboxError(
                f"failed to read transform base_link -> {frame}: {error}"
            ) from error
        if (
            len(values) != 7
            or timestamp_ns == 0
            or not all(math.isfinite(value) for value in values[:3])
        ):
            raise GrabboxError(
                f"failed to read transform base_link -> {frame}"
            )
        return float(values[0]), float(values[1]), float(values[2])

    def _check_drop_preconditions(self) -> None:
        head_position = self._read_frame_position("head_base_link")
        body_height_error = abs(
            head_position[2] - GRAB_HEAD_BASE_HEIGHT_M
        )
        if body_height_error > self.drop_body_tolerance_m:
            raise GrabboxError(
                "drop precondition failed: body height "
                f"{head_position[2]:.4f} m is not within "
                f"{self.drop_body_tolerance_m:.4f} m of "
                f"{GRAB_HEAD_BASE_HEIGHT_M:.4f} m"
            )

        checkpoints = (
            (
                "left_gripper_tcp_link",
                LEFT_GRAB_TCP_POSITION_M,
            ),
            (
                "right_gripper_tcp_link",
                RIGHT_GRAB_TCP_POSITION_M,
            ),
        )
        for frame, target in checkpoints:
            actual = self._read_frame_position(frame)
            error = math.dist(actual, target)
            if error > self.drop_arm_tolerance_m:
                raise GrabboxError(
                    f"drop precondition failed: {frame} is "
                    f"{error:.4f} m from the grab target, tolerance "
                    f"{self.drop_arm_tolerance_m:.4f} m"
                )

    def _plan_single_point_arms(
        self,
        left_positions: Sequence[float],
        right_positions: Sequence[float],
        *,
        operation: str,
        enable_collisions: bool,
    ) -> None:
        left_names = list(self.motion.get_chain_joint_names("left_arm"))
        right_names = list(self.motion.get_chain_joint_names("right_arm"))
        if len(left_names) != 7 or len(right_names) != 7:
            raise GrabboxError(
                "failed to query left/right arm joint names from GalbotMotion"
            )

        left_target = gm.MotionPlanChainTarget()
        left_target.chain_name = "left_arm"
        left_target.mode = gm.MotionPlanTargetMode.kJoint
        left_target.joint.chain_name = "left_arm"
        left_target.joint.joint_names = left_names
        left_target.joint.joint_positions = list(left_positions)

        right_target = gm.MotionPlanChainTarget()
        right_target.chain_name = "right_arm"
        right_target.mode = gm.MotionPlanTargetMode.kJoint
        right_target.joint.chain_name = "right_arm"
        right_target.joint.joint_names = right_names
        right_target.joint.joint_positions = list(right_positions)

        params = gm.Parameter()
        params.set_direct_execute(True)
        params.set_blocking(True)
        params.set_timeout(self.planning_timeout_s)
        if enable_collisions:
            params.set_check_collision(True)
            params.set_enable_env_collision_check(True)
        else:
            # The official G1 home trajectory example disables collision checking
            # because MPS reports this known home interpolation as FAULT.
            params.set_check_collision(False)
            params.set_enable_env_collision_check(False)
        params.set_reference_frame("base_link")
        params.set_actuate("with_chain_only")

        status, _ = self.motion.traj_plan(
            [[left_target, right_target]],
            params,
        )
        if status != gm.MotionStatus.SUCCESS:
            status_name = self.motion.status_to_string(status)
            raise GrabboxError(
                f"{operation} arm planning failed: "
                f"{status} ({status_name})"
            )

    def _plan_default_arms(self) -> None:
        self._plan_single_point_arms(
            LEFT_ARM_DEFAULT_POSITION,
            RIGHT_ARM_DEFAULT_POSITION,
            operation="single-point default",
            enable_collisions=False,
        )

    def _plan_grab_arms(self) -> None:
        self._plan_single_point_arms(
            LEFT_ARM_GRAB_POSITION,
            RIGHT_ARM_GRAB_POSITION,
            operation="single-point grab",
            enable_collisions=False,
        )

    def _plan_grab_with_obstacle(self) -> None:
        status = self.motion.add_obstacle(
            obstacle_id=GRAB_OBSTACLE_ID,
            obstacle_type="box",
            pose=list(GRAB_OBSTACLE_POSE),
            scale=list(GRAB_OBSTACLE_SCALE),
            target_frame="base_link",
        )
        if status != gm.MotionStatus.SUCCESS:
            raise GrabboxError(f"add grab obstacle failed: {status}")
        try:
            self._plan_grab_arms()
        finally:
            status = self.motion.remove_obstacle(GRAB_OBSTACLE_ID)
            if status != gm.MotionStatus.SUCCESS:
                raise GrabboxError(
                    f"remove grab obstacle failed: {status}"
                )

    def default(self) -> None:
        """Move to the raised-body, arms-down, fully-open default state."""
        self._set_default_legs()
        self._set_joints(HEAD_JOINT_NAMES, HEAD_DEFAULT_POSITION)
        self._plan_default_arms()
        self._set_gripper(GRIPPER_DEFAULT_WIDTH_M)

    def grab(self) -> None:
        """Move to the raised-body grab state using the default planner."""
        self._set_grab_legs()
        self._set_gripper(GRIPPER_GRAB_WIDTH_M)
        self._set_joints(HEAD_JOINT_NAMES, HEAD_DEFAULT_POSITION)
        self._plan_grab_with_obstacle()

    def _build_drop_trajectory(self) -> gm.Trajectory:
        trajectory = gm.Trajectory()
        trajectory.joint_groups = []
        trajectory.joint_names = list(
            LEFT_ARM_JOINT_NAMES + RIGHT_ARM_JOINT_NAMES
        )
        points: list[gm.TrajectoryPoint] = []
        for index, (left, right) in enumerate(
            zip(LEFT_ARM_DROP_WAYPOINTS, RIGHT_ARM_DROP_WAYPOINTS),
            start=1,
        ):
            point = gm.TrajectoryPoint()
            point.time_from_start_second = index * self.trajectory_step_s
            commands: list[gm.JointCommand] = []
            for position in list(left) + list(right):
                command = gm.JointCommand()
                command.position = float(position)
                commands.append(command)
            point.joint_command_vec = commands
            points.append(point)
        trajectory.points = points
        return trajectory

    def drop(self) -> None:
        """Lower the body and follow the synchronized drop path."""
        #self._check_drop_preconditions()
        status = self.robot.execute_joint_trajectory(
            self._build_drop_trajectory(),
            is_blocking=True,
        )
        self._set_joints(
            LEG_JOINT_NAMES,
            LEG_DROP_POSITION,
            blocking=True,
        )
        self._set_gripper(GRIPPER_GRAB_WIDTH_M)

        self._set_drop_over_arms()
        self._require_success("execute_joint_trajectory(drop)", status)
