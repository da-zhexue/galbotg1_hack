"""Control a real Galbot G1 through galbot_sdk using ROS 2 topics."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from ctrl_msgs.msg import GripperCommand, JointGroupCommand, WheelCommand

from .joint_utils import (
    GROUP_PREFIXES,
    TOPIC_GRIPPER,
    TOPIC_WHEELS,
    validate_joint_names,
    wheel_velocities_to_base,
)


class RealRobotBridge(Node):
    """Consume the shared control topics and forward them to the robot SDK."""

    def __init__(self) -> None:
        super().__init__("real_robot_bridge")

        self.declare_parameter("allow_motion", True)
        self.declare_parameter("speed_rad_s", 0.2)
        self.declare_parameter("command_timeout_s", 15.0)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("wheel_command_timeout_s", 0.5)
        self.declare_parameter("max_wheel_speed_rad_s", 25.0)
        self.declare_parameter("max_linear_speed_mps", 1.0)
        self.declare_parameter("max_angular_speed_rad_s", 1.5)

        self._allow_motion = bool(
            self.get_parameter("allow_motion").value
        )
        self._speed_rad_s = float(
            self.get_parameter("speed_rad_s").value
        )
        self._command_timeout_s = float(
            self.get_parameter("command_timeout_s").value
        )
        self._command_period_s = 1.0 / max(
            float(self.get_parameter("command_rate_hz").value), 1.0
        )
        self._wheel_timeout_default_s = float(
            self.get_parameter("wheel_command_timeout_s").value
        )
        self._max_wheel_speed = float(
            self.get_parameter("max_wheel_speed_rad_s").value
        )
        self._max_linear_speed = float(
            self.get_parameter("max_linear_speed_mps").value
        )
        self._max_angular_speed = float(
            self.get_parameter("max_angular_speed_rad_s").value
        )

        self._robot = None
        self._shutdown = False
        self._last_group_command: dict[str, float] = {}
        self._last_gripper_command = {"left": 0.0, "right": 0.0}
        self._last_wheel_message_s = 0.0
        self._wheel_timeout_s = self._wheel_timeout_default_s
        self._base_moving = False

        if self._allow_motion:
            self._initialize_sdk()

        for topic in GROUP_PREFIXES:
            self.create_subscription(
                JointGroupCommand,
                topic,
                lambda msg, topic=topic: self._on_joint_group(msg, topic),
                10,
            )
        self.create_subscription(
            WheelCommand, TOPIC_WHEELS, self._on_wheels, 10
        )
        self.create_subscription(
            GripperCommand, TOPIC_GRIPPER, self._on_gripper, 10
        )
        self.create_timer(0.05, self._watchdog)

        if not self._allow_motion:
            self.get_logger().warning(
                "allow_motion is false; commands are accepted but ignored. "
                "Set allow_motion:=true after confirming the robot is safe."
            )

    def _initialize_sdk(self) -> None:
        try:
            from galbot_sdk.g1 import GalbotRobot
        except ImportError as error:
            raise RuntimeError(
                "galbot_sdk is unavailable. Source the SDK setup.sh first."
            ) from error

        robot = GalbotRobot()
        if not robot.init():
            robot.destroy()
            raise RuntimeError("GalbotRobot initialization failed")
        self._robot = robot
        self.get_logger().info("GalbotRobot initialized")

    def _on_joint_group(
        self, msg: JointGroupCommand, topic: str
    ) -> None:
        if not self._allow_motion or self._robot is None:
            return

        names = list(msg.name)
        positions = list(msg.position)
        error = validate_joint_names(topic, names, positions)
        if error is not None:
            self.get_logger().error(f"{topic}: {error}")
            return

        now = time.monotonic()
        last = self._last_group_command.get(topic, 0.0)
        if now - last < self._command_period_s:
            return
        self._last_group_command[topic] = now

        status = self._robot.set_joint_positions(
            joint_positions=positions,
            joint_groups=[],
            joint_names=names,
            is_blocking=False,
            speed_rad_s=self._speed_rad_s,
            timeout_s=self._command_timeout_s,
        )
        if not getattr(status, "name", "") == "SUCCESS":
            self.get_logger().error(
                f"{topic}: set_joint_positions failed: {status}"
            )

    def _on_wheels(self, msg: WheelCommand) -> None:
        if not self._allow_motion or self._robot is None:
            return

        wheel_velocity = list(msg.wheel_velocity)
        if any(
            not math.isfinite(value) or abs(value) > self._max_wheel_speed
            for value in wheel_velocity
        ):
            self.get_logger().error(
                "wheel velocity is invalid or exceeds max_wheel_speed_rad_s"
            )
            return

        try:
            vx, vy, wz = wheel_velocities_to_base(wheel_velocity)
        except ValueError as error:
            self.get_logger().error(f"wheel conversion failed: {error}")
            return

        if (
            math.hypot(vx, vy) > self._max_linear_speed
            or abs(wz) > self._max_angular_speed
        ):
            self.get_logger().error(
                "converted base velocity exceeds configured safety limits"
            )
            return

        status = self._robot.set_base_velocity(
            linear_velocity=[vx, vy, 0.0],
            angular_velocity=[0.0, 0.0, wz],
            duration_s=0.0,
        )
        if not getattr(status, "name", "") == "SUCCESS":
            self.get_logger().error(
                f"set_base_velocity failed: {status}"
            )
            return

        self._last_wheel_message_s = time.monotonic()
        requested_timeout = float(msg.timeout_s)
        self._wheel_timeout_s = (
            requested_timeout
            if requested_timeout > 0.0
            else self._wheel_timeout_default_s
        )
        self._base_moving = any(abs(value) > 1e-6 for value in wheel_velocity)

    def _on_gripper(self, msg: GripperCommand) -> None:
        if not self._allow_motion or self._robot is None:
            return

        velocity = float(msg.velocity_mps)
        effort = float(msg.effort)
        if not math.isfinite(velocity) or not math.isfinite(effort):
            self.get_logger().error(
                "gripper velocity and effort must be finite"
            )
            return
        velocity = max(min(velocity, 0.2), 1e-3)
        effort = max(min(effort, 100.0), 1e-3)
        commands = (
            ("left", bool(msg.control_left), float(msg.left_width_m)),
            ("right", bool(msg.control_right), float(msg.right_width_m)),
        )
        now = time.monotonic()
        for side, enabled, width in commands:
            if not enabled:
                continue
            if not math.isfinite(width) or not 0.0 <= width <= 0.12:
                self.get_logger().error(
                    f"{side} gripper width must be in [0.0, 0.12] m"
                )
                continue
            if now - self._last_gripper_command[side] < self._command_period_s:
                continue
            self._last_gripper_command[side] = now
            status = self._robot.set_gripper_command(
                end_effector=f"{side}_gripper",
                width_m=width,
                velocity_mps=velocity,
                effort=effort,
                is_blocking=False,
            )
            if not getattr(status, "name", "") == "SUCCESS":
                self.get_logger().error(
                    f"{side} gripper command failed: {status}"
                )

    def _watchdog(self) -> None:
        if (
            not self._base_moving
            or self._robot is None
            or time.monotonic() - self._last_wheel_message_s
            <= self._wheel_timeout_s
        ):
            return

        status = self._robot.stop_base()
        if not getattr(status, "name", "") == "SUCCESS":
            self.get_logger().error(f"stop_base failed: {status}")
        else:
            self._base_moving = False
            self.get_logger().warning(
                "wheel command timeout reached; base stopped"
            )

    def shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True
        if self._robot is None:
            return

        self._robot.stop_base()
        self._robot.stop_trajectory_execution()
        self._robot.request_shutdown()
        self._robot.wait_for_shutdown()
        self._robot.destroy()
        self._robot = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RealRobotBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
