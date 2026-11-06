"""Run Drop, then spread both gripper TCPs to 30 cm spacing."""

from __future__ import annotations

import rclpy

from .drop import DropNode
from .joint_utils import (
    LEFT_ARM_DROP_OVER_POSITION,
    LEFT_ARM_JOINT_NAMES,
    RIGHT_ARM_DROP_OVER_POSITION,
    RIGHT_ARM_JOINT_NAMES,
)


class DropOverNode(DropNode):
    """Execute Drop followed by the wider release pose."""

    def __init__(self) -> None:
        super().__init__("drop_over")
        self.declare_parameter("over_hold_s", 3.0)

    def _publish_over_pose(self) -> None:
        repeat_count = max(
            int(self.get_parameter("repeat_count").value), 1
        )
        repeat_period_s = max(
            float(self.get_parameter("repeat_period_s").value), 0.01
        )
        left_message = self._joint_message(
            LEFT_ARM_JOINT_NAMES, LEFT_ARM_DROP_OVER_POSITION
        )
        right_message = self._joint_message(
            RIGHT_ARM_JOINT_NAMES, RIGHT_ARM_DROP_OVER_POSITION
        )
        for _ in range(repeat_count):
            self._left_publisher.publish(left_message)
            self._right_publisher.publish(right_message)
            self._spin_for(repeat_period_s)

    def run(self) -> None:
        super().run()
        if not bool(self.get_parameter("confirm").value):
            return

        self.get_logger().warning(
            "Drop complete; spreading both gripper TCPs to 0.30 m"
        )
        self._publish_over_pose()
        self._spin_for(
            float(self.get_parameter("over_hold_s").value)
        )
        self.get_logger().info("Drop-over pose command finished")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DropOverNode()
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
