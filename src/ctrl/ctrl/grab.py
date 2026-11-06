"""Move both arms to the configured grab pose through shared topics."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from ctrl_msgs.msg import GripperCommand, JointGroupCommand

from .joint_utils import (
    GRIPPER_DEFAULT_EFFORT,
    GRIPPER_DEFAULT_VELOCITY_MPS,
    GRIPPER_HALF_CLOSED_WIDTH_M,
    LEFT_ARM_GRAB_POSITION,
    LEFT_ARM_JOINT_NAMES,
    LEG_GRAB_POSITION,
    LEG_JOINT_NAMES,
    RIGHT_ARM_GRAB_POSITION,
    RIGHT_ARM_JOINT_NAMES,
    TOPIC_GRIPPER,
    TOPIC_LEFT_HAND,
    TOPIC_LEG,
    TOPIC_RIGHT_HAND,
)


class GrabNode(Node):
    """Publish the left/right arm grab pose."""

    def __init__(self) -> None:
        super().__init__("grab")

        self.declare_parameter("confirm", False)
        self.declare_parameter("discovery_wait_s", 1.5)
        self.declare_parameter("repeat_count", 3)
        self.declare_parameter("repeat_period_s", 0.1)
        self.declare_parameter("settle_s", 5.0)

        self._left_publisher = self.create_publisher(
            JointGroupCommand, TOPIC_LEFT_HAND, 10
        )
        self._right_publisher = self.create_publisher(
            JointGroupCommand, TOPIC_RIGHT_HAND, 10
        )
        self._leg_publisher = self.create_publisher(
            JointGroupCommand, TOPIC_LEG, 10
        )
        self._gripper_publisher = self.create_publisher(
            GripperCommand, TOPIC_GRIPPER, 10
        )

    def _make_message(
        self,
        joint_names: tuple[str, ...],
        positions: list[float],
    ) -> JointGroupCommand:
        message = JointGroupCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(joint_names)
        message.position = list(positions)
        message.velocity = []
        message.effort = []
        return message

    def _make_gripper_message(self) -> GripperCommand:
        message = GripperCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.control_left = True
        message.left_width_m = GRIPPER_HALF_CLOSED_WIDTH_M
        message.control_right = True
        message.right_width_m = GRIPPER_HALF_CLOSED_WIDTH_M
        message.velocity_mps = GRIPPER_DEFAULT_VELOCITY_MPS
        message.effort = GRIPPER_DEFAULT_EFFORT
        return message

    def _publish_pose(self) -> None:
        repeat_count = max(
            int(self.get_parameter("repeat_count").value), 1
        )
        repeat_period_s = max(
            float(self.get_parameter("repeat_period_s").value), 0.01
        )
        left_message = self._make_message(
            LEFT_ARM_JOINT_NAMES, LEFT_ARM_GRAB_POSITION
        )
        right_message = self._make_message(
            RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_GRAB_POSITION
        )
        leg_message = self._make_message(
            LEG_JOINT_NAMES, LEG_GRAB_POSITION
        )
        gripper_message = self._make_gripper_message()

        for _ in range(repeat_count):
            self._left_publisher.publish(left_message)
            self._right_publisher.publish(right_message)
            self._leg_publisher.publish(leg_message)
            self._gripper_publisher.publish(gripper_message)
            rclpy.spin_once(
                self, timeout_sec=max(repeat_period_s, 0.01)
            )

    def run(self) -> None:
        if not bool(self.get_parameter("confirm").value):
            self.get_logger().error(
                "Motion is disabled. Re-run with "
                "'--ros-args -p confirm:=true' after confirming the "
                "emergency stop is released and the workspace is clear."
            )
            return

        discovery_wait_s = max(
            float(self.get_parameter("discovery_wait_s").value), 0.0
        )
        settle_s = max(
            float(self.get_parameter("settle_s").value), 0.0
        )

        self.get_logger().info(
            "Waiting for control bridge discovery before publishing"
        )
        end_time = time.monotonic() + discovery_wait_s
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().warning(
            "Publishing grab pose, half-closed grippers, and default leg pose"
        )
        self._publish_pose()
        time.sleep(settle_s)
        self.get_logger().info("Grab pose command finished")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GrabNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
