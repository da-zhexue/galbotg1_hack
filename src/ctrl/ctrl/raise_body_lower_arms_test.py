"""Publish a safe pose test: lower both arms, then raise the body."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from ctrl_msgs.msg import JointGroupCommand

from .joint_utils import (
    HEAD_JOINT_NAMES,
    LEFT_ARM_JOINT_NAMES,
    LEG_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
    TOPIC_HEAD,
    TOPIC_LEFT_HAND,
    TOPIC_LEG,
    TOPIC_RIGHT_HAND,
)

# Official G1 preset pose. Its head_base_link height is about 1.444 m.
LEG_RAISED_POSITION = [0.5, 1.5, 1.0, 0.0, 0.0]
HEAD_NEUTRAL_POSITION = [0.0, 0.0]
LEFT_ARM_DOWN_POSITION = [
    2.0,
    -1.5,
    -0.6,
    -1.7,
    0.0,
    -0.8,
    0.0,
]
RIGHT_ARM_DOWN_POSITION = [
    -2.0,
    1.5,
    0.6,
    1.7,
    0.0,
    0.8,
    0.0,
]


class RaiseBodyLowerArmsTest(Node):
    """Publish the same commands to real hardware or the RViz backend."""

    def __init__(self) -> None:
        super().__init__("raise_body_lower_arms_test")

        self.declare_parameter("confirm", False)
        self.declare_parameter("discovery_wait_s", 1.5)
        self.declare_parameter("repeat_count", 3)
        self.declare_parameter("repeat_period_s", 0.1)
        self.declare_parameter("arm_settle_s", 2.0)
        self.declare_parameter("leg_settle_s", 8.0)

        self._command_publishers = {
            TOPIC_HEAD: self.create_publisher(
                JointGroupCommand, TOPIC_HEAD, 10
            ),
            TOPIC_LEFT_HAND: self.create_publisher(
                JointGroupCommand, TOPIC_LEFT_HAND, 10
            ),
            TOPIC_RIGHT_HAND: self.create_publisher(
                JointGroupCommand, TOPIC_RIGHT_HAND, 10
            ),
            TOPIC_LEG: self.create_publisher(
                JointGroupCommand, TOPIC_LEG, 10
            ),
        }

    def _message(
        self,
        names: tuple[str, ...],
        positions: list[float],
    ) -> JointGroupCommand:
        message = JointGroupCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(names)
        message.position = list(positions)
        message.velocity = []
        message.effort = []
        return message

    def _publish_repeated(
        self,
        topic: str,
        message: JointGroupCommand,
        repeat_count: int,
        repeat_period_s: float,
    ) -> None:
        for _ in range(max(repeat_count, 1)):
            self._command_publishers[topic].publish(message)
            rclpy.spin_once(self, timeout_sec=max(repeat_period_s, 0.01))

    def run_test(self) -> None:
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
        repeat_count = int(self.get_parameter("repeat_count").value)
        repeat_period_s = max(
            float(self.get_parameter("repeat_period_s").value), 0.01
        )
        arm_settle_s = max(
            float(self.get_parameter("arm_settle_s").value), 0.0
        )
        leg_settle_s = max(
            float(self.get_parameter("leg_settle_s").value), 0.0
        )

        self.get_logger().info(
            "Waiting for control bridge discovery before publishing"
        )
        end_time = time.monotonic() + discovery_wait_s
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

        self.get_logger().warning("Step 1/2: lowering both arms")
        head = self._message(
            HEAD_JOINT_NAMES, HEAD_NEUTRAL_POSITION
        )
        left_arm = self._message(
            LEFT_ARM_JOINT_NAMES, LEFT_ARM_DOWN_POSITION
        )
        right_arm = self._message(
            RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_DOWN_POSITION
        )
        self._publish_repeated(
            TOPIC_HEAD, head, repeat_count, repeat_period_s
        )
        self._publish_repeated(
            TOPIC_LEFT_HAND, left_arm, repeat_count, repeat_period_s
        )
        self._publish_repeated(
            TOPIC_RIGHT_HAND, right_arm, repeat_count, repeat_period_s
        )
        time.sleep(arm_settle_s)

        self.get_logger().warning("Step 2/2: raising the body")
        leg = self._message(LEG_JOINT_NAMES, LEG_RAISED_POSITION)
        self._publish_repeated(
            TOPIC_LEG, leg, repeat_count, repeat_period_s
        )
        time.sleep(leg_settle_s)
        self.get_logger().info("Raise-body/lower-arms test finished")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RaiseBodyLowerArmsTest()
    try:
        node.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
