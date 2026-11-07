#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sequential chassis route for a Galbot G1."""

from __future__ import annotations

import math
import time
from collections.abc import Callable

from galbot_sdk.g1 import ControlStatus, GalbotRobot


class G1RouteError(RuntimeError):
    """Raised when a chassis command does not complete successfully."""


def check_numpy_available() -> None:
    """Raise G1RouteError when the SDK's numpy dependency is broken."""
    try:
        import numpy  # noqa: F401
    except Exception as error:
        raise G1RouteError(
            "numpy is required by the Galbot navigation SDK but failed "
            f"to import: {type(error).__name__}: {error}"
        ) from error


class G1Route:
    """Run the predefined chassis route through an initialized SDK robot."""

    CONTROLLER_NAME = "chassis_pose_ctrl"
    CONTROLLER_GROUP = "chassis"
    RELATIVE_FRAME = "rel(0)"
    ODOM_FRAME = "odom"
    LINEAR_STATIONARY_TOLERANCE_MPS = 0.03
    ANGULAR_STATIONARY_TOLERANCE_RAD_S = 0.05
    TOTAL_PATH_LENGTH_M = 3.8
    NAVIGATION_READY_TIMEOUT_S = 5.0
    NAV_GOAL_X_M = 1.0
    NAV_MIN_DISPLACEMENT_M = 0.5

    def __init__(
        self,
        robot: GalbotRobot,
        *,
        timeout_s: float = 25.0,
        step_settle_s: float = 0.5,
        controller_settle_s: float = 1.0,
        navigation_timeout_s: float = 30.0,
        sleep: Callable[[float], None] = time.sleep,
        navigation=None,
    ) -> None:
        if robot is None:
            raise ValueError("robot must be a GalbotRobot instance")
        self.robot = robot
        self.timeout_s = float(timeout_s)
        self.step_settle_s = float(step_settle_s)
        self.controller_settle_s = float(controller_settle_s)
        self.navigation_timeout_s = float(navigation_timeout_s)
        self.sleep = sleep
        self.navigation = navigation
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if self.step_settle_s < 0.0:
            raise ValueError("step_settle_s must be non-negative")
        if self.controller_settle_s < 0.0:
            raise ValueError("controller_settle_s must be non-negative")
        if self.navigation_timeout_s <= 0.0:
            raise ValueError("navigation_timeout_s must be positive")

    @staticmethod
    def _require_success(
        operation: str,
        status: ControlStatus,
    ) -> None:
        if status != ControlStatus.SUCCESS:
            raise G1RouteError(f"{operation} failed: {status}")

    def check_stationary(self) -> None:
        """Raise when the chassis is translating or rotating."""
        velocity = self.robot.get_base_velocity()
        linear = velocity["linear_velocity"]
        angular = velocity["angular_velocity"]

        if (
            max(abs(float(value)) for value in linear)
            > self.LINEAR_STATIONARY_TOLERANCE_MPS
        ):
            raise G1RouteError("Base is not stationary")

        if (
            max(abs(float(value)) for value in angular)
            > self.ANGULAR_STATIONARY_TOLERANCE_RAD_S
        ):
            raise G1RouteError("Base is rotating")

    def ensure_pose_controller(self) -> None:
        """Switch the chassis to the pose controller when needed."""
        controller = self.robot.get_active_controller(
            self.CONTROLLER_GROUP
        )
        print("Current controller:", controller)

        if controller == self.CONTROLLER_NAME:
            print("Pose controller ready!")
            return

        print(f"Switching to {self.CONTROLLER_NAME}...")
        status = self.robot.switch_controller(self.CONTROLLER_NAME)
        print("Switch status:", status)
        self._require_success("switch_controller", status)

        self.sleep(self.controller_settle_s)
        controller = self.robot.get_active_controller(
            self.CONTROLLER_GROUP
        )
        if controller != self.CONTROLLER_NAME:
            raise G1RouteError("Pose controller not confirmed")
        print("Pose controller ready!")

    @staticmethod
    def _planar_yaw(pose) -> float:
        """Return the yaw angle of a planar pose quaternion."""
        _, _, _, qx, qy, qz, qw = (float(value) for value in pose[:7])
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    def _goal_pose_in_map(self, current_pose) -> list[float]:
        """Convert the relative forward goal into the map frame."""
        yaw = self._planar_yaw(current_pose)
        return [
            float(current_pose[0]) + self.NAV_GOAL_X_M * math.cos(yaw),
            float(current_pose[1]) + self.NAV_GOAL_X_M * math.sin(yaw),
            float(current_pose[2]),
            0.0,
            0.0,
            math.sin(yaw / 2.0),
            math.cos(yaw / 2.0),
        ]

    def _navigation_state(self) -> str:
        try:
            return str(self.navigation.get_navigation_status().name)
        except Exception as error:
            return f"<unknown: {error}>"

    def _navigation_ready(self) -> bool:
        """Return True when navigation is initialized and localized."""
        if getattr(self, "navigation", None) is None:
            return False

        deadline = time.monotonic() + self.NAVIGATION_READY_TIMEOUT_S
        while not self.navigation.is_localized():
            if time.monotonic() >= deadline:
                return False
            self.sleep(0.5)
        return True

    def navigate_to_door(self) -> None:
        """Navigate one meter forward through the SDK navigation service."""
        if not self._navigation_ready():
            raise G1RouteError(
                "Navigation is not ready: create and init GalbotNavigation, "
                "then wait until the robot is localized"
            )

        current_pose = self.navigation.get_current_pose()
        goal_map = self._goal_pose_in_map(current_pose)
        print("[g1-route] current map pose:")
        print(current_pose)
        print("[g1-route] goal map pose:")
        print(goal_map)
        print("Navigation state before:", self._navigation_state())

        reachable: bool | None = None
        try:
            reachable = bool(
                self.navigation.check_path_reachability(
                    goal_map,
                    current_pose,
                )
            )
            print("Path reachable:", reachable)
        except Exception as error:
            print(f"Path reachability check failed: {error}")

        odom_before = self.robot.get_odom()
        success, status = self.navigation.navigate_to_goal_v2(
            [self.NAV_GOAL_X_M, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [0.2, 0.2, 0.3],
            pose_frame="base_link",
            enable_collision_check=True,
            is_blocking=True,
            timeout=self.navigation_timeout_s,
        )
        print("Navigation result:", success, status)
        if not success:
            state = self._navigation_state()
            reachable_text = "unknown" if reachable is None else reachable
            raise G1RouteError(
                f"navigate_to_goal_v2 failed: {status} "
                f"(state={state}, path_reachable={reachable_text})"
            )
        print("Navigation state after:", self._navigation_state())

        self.sleep(self.step_settle_s)
        arrived = bool(self.navigation.check_goal_arrival())
        odom_after = self.robot.get_odom()
        dx = float(odom_after["position"][0]) - float(
            odom_before["position"][0]
        )
        dy = float(odom_after["position"][1]) - float(
            odom_before["position"][1]
        )
        moved = (dx * dx + dy * dy) ** 0.5
        print(f"Goal arrival: {arrived}, base displacement: {moved:.3f} m")
        if moved < self.NAV_MIN_DISPLACEMENT_M:
            raise G1RouteError(
                "navigate_to_goal_v2 reported success but the base "
                f"only moved {moved:.3f} m"
            )

        print("Current odometry:")
        print(odom_after)

    def _execute_action(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float,
        duration_s: float,
    ) -> None:
        print("\n================================")
        print("Action:", name)
        print("X:", x, "m")
        print("Y:", y, "m")
        print("Yaw:", yaw, "rad")
        print("Duration:", duration_s, "s")
        print("================================")

        status = self.robot.set_base_pose(
            x,
            y,
            yaw,
            self.RELATIVE_FRAME,
            self.ODOM_FRAME,
            duration_s,
            True,
            self.timeout_s,
        )
        print("Motion status:", status)
        self._require_success(f"set_base_pose({name})", status)
        print("Action completed:", name)

        self.sleep(self.step_settle_s)
        print("Current odometry:")
        print(self.robot.get_odom())

    def forward_80cm(self) -> None:
        self._execute_action("Forward 80cm", 0.80, 0.00, 0.0, 6.0)

    def right_50cm_1(self) -> None:
        self._execute_action("Right 50cm - 1", 0.00, -0.50, 0.0, 5.0)

    def right_50cm_2(self) -> None:
        self._execute_action("Right 50cm - 2", 0.00, -0.50, 0.0, 5.0)

    def right_50cm_3(self) -> None:
        self._execute_action("Right 50cm - 3", 0.00, -0.50, 0.0, 5.0)

    def right_50cm_4(self) -> None:
        self._execute_action("Right 50cm - 4", 0.00, -0.50, 0.0, 5.0)

    def forward_100cm(self) -> None:
        self._execute_action("Forward 100cm", 1.00, 0.00, 0.0, 8.0)

    def stop_base(self) -> None:
        """Request a base stop and verify the SDK result."""
        print("\nStopping base...")
        status = self.robot.stop_base()
        print("Stop status:", status)
        self._require_success("stop_base", status)
