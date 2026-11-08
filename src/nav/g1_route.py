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
    TABLE_EXIT_SPEED_MPS = 0.2
    FIXED_POINT_SPEED_MPS = 0.2
    FIXED_POINT_YAW_SPEED_RAD_S = 0.4
    RETURN_POSITION_TOLERANCE_M = 0.02
    RETURN_YAW_TOLERANCE_RAD = 0.02
    # From the table-exit pose back to the origin: the robot is 0.60 m
    # forward and 2.05 m right of the origin at that point.
    RETURN_ORIGIN_BACKWARD_M = 0.60
    RETURN_ORIGIN_LEFT_M = 2.05
    NAVIGATION_READY_TIMEOUT_S = 30.0
    NAV_GOAL_FORWARD_M = 2.0
    NAV_GOAL_LEFT_M = 2.0
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
        # Odom poses recorded before the first move and after every move;
        # the return uses them to undo the door segment.
        self._path: list[tuple[float, float, float]] = []
        self._table_exit_index: int | None = None
        self._forward_200_index: int | None = None
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
        """Convert the relative forward/left goal into the map frame."""
        yaw = self._planar_yaw(current_pose)
        forward = self.NAV_GOAL_FORWARD_M
        left = self.NAV_GOAL_LEFT_M
        return [
            float(current_pose[0])
            + forward * math.cos(yaw)
            - left * math.sin(yaw),
            float(current_pose[1])
            + forward * math.sin(yaw)
            + left * math.cos(yaw),
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
        last_report = 0.0
        while not self.navigation.is_localized():
            now = time.monotonic()
            if now >= deadline:
                return False
            if now - last_report >= 1.0:
                print(
                    "[g1-route] waiting for localization: "
                    f"state={self._navigation_state()}"
                )
                last_report = now
            self.sleep(0.5)
        print("[g1-route] navigation localized")
        return True

    def navigate_to_door(self) -> None:
        """Navigate 2 m forward and 2 m left through the navigation service."""
        if not self._navigation_ready():
            raise G1RouteError(
                "Navigation is not ready: create and init GalbotNavigation, "
                "then wait until the robot is localized"
            )

        current_pose = self.navigation.get_current_pose()
        goal_map = self._goal_pose_in_map(current_pose)
        print("[g1-route] current map pose:")
        print(current_pose)
        print(
            "[g1-route] relative goal: "
            f"forward={self.NAV_GOAL_FORWARD_M:.2f} m, "
            f"left={self.NAV_GOAL_LEFT_M:.2f} m"
        )
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
        # Navigate to the same map-frame goal that was checked above.
        success, status = self.navigation.navigate_to_goal_v2(
            goal_map,
            [0.2, 0.2, 0.3],
            pose_frame="map",
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

    @staticmethod
    def _pose_xyyaw(odom) -> tuple[float, float, float]:
        """Return (x, y, yaw) from an SDK odometry dictionary."""
        position = odom["position"]
        orientation = odom["orientation"]
        pose = [
            float(position[0]),
            float(position[1]),
            float(position[2]),
            float(orientation[0]),
            float(orientation[1]),
            float(orientation[2]),
            float(orientation[3]),
        ]
        return pose[0], pose[1], G1Route._planar_yaw(pose)

    def _execute_action(
        self,
        name: str,
        x: float,
        y: float,
        yaw: float,
        duration_s: float,
        *,
        record: bool = True,
    ) -> None:
        if record and not self._path:
            self._path.append(self._pose_xyyaw(self.robot.get_odom()))

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
        odom = self.robot.get_odom()
        if record:
            self._path.append(self._pose_xyyaw(odom))
        print("Current odometry:")
        print(odom)

    def forward_60cm(self) -> None:
        self._execute_action("Forward 60cm", 0.60, 0.00, 0.0, 6.0)

    def forward_40cm(self) -> None:
        self._execute_action("Forward 40cm", 0.40, 0.00, 0.0, 4.0)

    def backward_40cm(self) -> None:
        """Back away from the table after lifting the parcel."""
        self._execute_action("Backward 40cm", -0.40, 0.00, 0.0, 4.0)

    def right_50cm(self) -> None:
        self._execute_action("Right 50cm", 0.00, -0.50, 0.0, 5.0)

    def right_55cm(self) -> None:
        self._execute_action("Right 55cm", 0.00, -0.55, 0.0, 5.5)

    def leave_table(self, distance_m: float) -> None:
        """Move right by distance_m to clear the right edge of the table."""
        distance_m = float(distance_m)
        if distance_m <= 0.0:
            raise ValueError("distance_m must be positive")
        self._execute_action(
            "Leave table",
            0.00,
            -distance_m,
            0.0,
            distance_m / self.TABLE_EXIT_SPEED_MPS,
        )
        self._table_exit_index = len(self._path) - 1

    def forward_200cm(self) -> None:
        self._execute_action("Forward 200cm", 2.00, 0.00, 0.0, 8.0)
        self._forward_200_index = len(self._path) - 1

    def _fixed_point_duration(
        self,
        x: float,
        y: float,
        yaw: float,
    ) -> float:
        """Return a duration that keeps translation and yaw below limits."""
        return max(
            math.hypot(x, y) / self.FIXED_POINT_SPEED_MPS,
            abs(yaw) / self.FIXED_POINT_YAW_SPEED_RAD_S,
        )

    def fixed_point_1(self) -> None:
        """Leave the pickup area: drive 2.00 m forward."""
        x, y, yaw = 2.00, 0.00, 0.0
        self._execute_action(
            "Fixed point 1",
            x,
            y,
            yaw,
            self._fixed_point_duration(x, y, yaw),
        )

    def fixed_point_2(self) -> None:
        """Enter the corridor: drive 0.80 m to the left."""
        x, y, yaw = 2.00, 0.80, 0.0
        self._execute_action(
            "Fixed point 2",
            x,
            y,
            yaw,
            self._fixed_point_duration(x, y, yaw),
        )

    def fixed_point_3(self) -> None:
        """Stop at the door: drive 0.80 m forward and turn 90 deg left."""
        x, y, yaw = 2.00, -0.80, math.pi / 2.0
        self._execute_action(
            "Fixed point 3",
            x,
            y,
            yaw,
            self._fixed_point_duration(x, y, yaw),
        )

    def _drive_to_waypoint(
        self,
        target: tuple[float, float, float],
    ) -> None:
        """Drive to an odom waypoint with one relative pose command."""
        current_x, current_y, current_yaw = self._pose_xyyaw(
            self.robot.get_odom()
        )
        target_x, target_y, target_yaw = target
        dx = target_x - current_x
        dy = target_y - current_y
        dyaw = math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )

        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        x = cos_yaw * dx + sin_yaw * dy
        y = -sin_yaw * dx + cos_yaw * dy
        if (
            math.hypot(x, y) < self.RETURN_POSITION_TOLERANCE_M
            and abs(dyaw) < self.RETURN_YAW_TOLERANCE_RAD
        ):
            return

        self._execute_action(
            "Return to origin",
            x,
            y,
            dyaw,
            self._fixed_point_duration(x, y, dyaw),
            record=False,
        )

    def return_to_origin(self) -> None:
        """Reverse the door segment, then drive a fixed way home."""
        if not self._path:
            print("[g1-route] no recorded route; already at the origin")
            return

        fixed_tail = (
            self._table_exit_index is not None
            and self._forward_200_index is not None
            and self._forward_200_index == self._table_exit_index + 1
        )
        if fixed_tail:
            # Undo the destination fixed points and the 2 m forward move,
            # stopping at the last pose recorded before the 2 m move.
            waypoints = list(
                reversed(self._path[self._table_exit_index:-1])
            )
        else:
            waypoints = list(reversed(self._path[:-1]))
        print(
            "[g1-route] returning to origin through "
            f"{len(waypoints)} recorded waypoints"
        )
        for index, waypoint in enumerate(waypoints, start=1):
            print(
                f"[g1-route] return waypoint {index}/{len(waypoints)}: "
                f"x={waypoint[0]:.3f}, y={waypoint[1]:.3f}, "
                f"yaw={waypoint[2]:.3f}"
            )
            self._drive_to_waypoint(waypoint)

        if fixed_tail:
            x = -self.RETURN_ORIGIN_BACKWARD_M
            y = self.RETURN_ORIGIN_LEFT_M
            print(
                "[g1-route] fixed return from the table exit: backward "
                f"{self.RETURN_ORIGIN_BACKWARD_M:.2f} m, left "
                f"{self.RETURN_ORIGIN_LEFT_M:.2f} m"
            )
            self._execute_action(
                "Fixed return to origin",
                x,
                y,
                0.0,
                self._fixed_point_duration(x, y, 0.0),
                record=False,
            )

        origin = self._path[0]
        current = self._pose_xyyaw(self.robot.get_odom())
        error = math.hypot(current[0] - origin[0], current[1] - origin[1])
        print(
            "[g1-route] returned to origin, remaining position error: "
            f"{error:.3f} m"
        )

    def stop_base(self) -> None:
        """Request a base stop and verify the SDK result."""
        print("\nStopping base...")
        status = self.robot.stop_base()
        print("Stop status:", status)
        self._require_success("stop_base", status)
