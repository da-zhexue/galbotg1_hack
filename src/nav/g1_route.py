#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Sequential chassis route for a Galbot G1."""

from __future__ import annotations

import time
from collections.abc import Callable

from galbot_sdk.g1 import ControlStatus, GalbotRobot


class G1RouteError(RuntimeError):
    """Raised when a chassis command does not complete successfully."""


class G1Route:
    """Run the predefined chassis route through an initialized SDK robot."""

    CONTROLLER_NAME = "chassis_pose_ctrl"
    CONTROLLER_GROUP = "chassis"
    RELATIVE_FRAME = "rel(0)"
    ODOM_FRAME = "odom"
    LINEAR_STATIONARY_TOLERANCE_MPS = 0.03
    ANGULAR_STATIONARY_TOLERANCE_RAD_S = 0.05
    TOTAL_PATH_LENGTH_M = 3.8

    def __init__(
        self,
        robot: GalbotRobot,
        *,
        timeout_s: float = 25.0,
        step_settle_s: float = 0.5,
        controller_settle_s: float = 1.0,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if robot is None:
            raise ValueError("robot must be a GalbotRobot instance")
        self.robot = robot
        self.timeout_s = float(timeout_s)
        self.step_settle_s = float(step_settle_s)
        self.controller_settle_s = float(controller_settle_s)
        self.sleep = sleep
        if self.timeout_s <= 0.0:
            raise ValueError("timeout_s must be positive")
        if self.step_settle_s < 0.0:
            raise ValueError("step_settle_s must be non-negative")
        if self.controller_settle_s < 0.0:
            raise ValueError("controller_settle_s must be non-negative")

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
