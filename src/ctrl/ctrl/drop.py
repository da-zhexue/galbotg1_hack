"""Lower the body, then move both gripper TCPs to 10 cm above ground."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from ctrl_msgs.msg import GripperCommand, JointGroupCommand

from .drop_trajectory import (
    END_EFFECTOR_ROLL_RAD,
    FINAL_END_EFFECTOR_HEIGHT_M,
    FINAL_FORWARD_OFFSET_M,
    GRAB_END_EFFECTOR_DISTANCE_M,
    LEFT_ARM_DROP_WAYPOINTS,
    LEG_DROP_POSITION,
    RIGHT_ARM_DROP_WAYPOINTS,
)
from .joint_utils import (
    GRIPPER_DEFAULT_EFFORT,
    GRIPPER_DEFAULT_VELOCITY_MPS,
    GRIPPER_HALF_CLOSED_WIDTH_M,
    LEFT_ARM_JOINT_NAMES,
    LEG_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
    TOPIC_GRIPPER,
    TOPIC_LEFT_HAND,
    TOPIC_LEG,
    TOPIC_RIGHT_HAND,
)


class DropNode(Node):
    """Execute the synchronized drop pose trajectory."""

    def __init__(self, node_name: str = "drop") -> None:
        super().__init__(node_name)

        self.declare_parameter("confirm", False)
        self.declare_parameter("discovery_wait_s", 1.5)
        self.declare_parameter("repeat_count", 3)
        self.declare_parameter("repeat_period_s", 0.1)
        self.declare_parameter("leg_settle_s", 10.0)
        self.declare_parameter("interpolation_period_s", 0.1)
        self.declare_parameter("final_hold_s", 3.0)

        self._leg_publisher = self.create_publisher(
            JointGroupCommand, TOPIC_LEG, 10
        )
        self._left_publisher = self.create_publisher(
            JointGroupCommand, TOPIC_LEFT_HAND, 10
        )
        self._right_publisher = self.create_publisher(
            JointGroupCommand, TOPIC_RIGHT_HAND, 10
        )
        self._gripper_publisher = self.create_publisher(
            GripperCommand, TOPIC_GRIPPER, 10
        )

    def _joint_message(
        self,
        joint_names: tuple[str, ...],
        positions: list[float],
    ) -> JointGroupCommand:
        message = JointGroupCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(joint_names)
        message.position = list(positions)
        return message

    def _gripper_message(self) -> GripperCommand:
        message = GripperCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.control_left = True
        message.left_width_m = GRIPPER_HALF_CLOSED_WIDTH_M
        message.control_right = True
        message.right_width_m = GRIPPER_HALF_CLOSED_WIDTH_M
        message.velocity_mps = GRIPPER_DEFAULT_VELOCITY_MPS
        message.effort = GRIPPER_DEFAULT_EFFORT
        return message

    def _spin_for(self, duration_s: float) -> None:
        end_time = time.monotonic() + max(duration_s, 0.0)
        while time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_discovery(self) -> None:
        wait_s = max(
            float(self.get_parameter("discovery_wait_s").value), 0.0
        )
        self.get_logger().info(
            "Waiting for control bridge discovery before publishing"
        )
        self._spin_for(wait_s)

    def _publish_leg(self) -> None:
        repeat_count = max(
            int(self.get_parameter("repeat_count").value), 1
        )
        repeat_period_s = max(
            float(self.get_parameter("repeat_period_s").value), 0.01
        )
        message = self._joint_message(
            LEG_JOINT_NAMES, LEG_DROP_POSITION
        )
        for _ in range(repeat_count):
            self._leg_publisher.publish(message)
            self._spin_for(repeat_period_s)

    def _publish_gripper(self) -> None:
        repeat_count = max(
            int(self.get_parameter("repeat_count").value), 1
        )
        repeat_period_s = max(
            float(self.get_parameter("repeat_period_s").value), 0.01
        )
        message = self._gripper_message()
        for _ in range(repeat_count):
            self._gripper_publisher.publish(message)
            self._spin_for(repeat_period_s)

    def _publish_arm_trajectory(self) -> None:
        interpolation_period_s = max(
            float(self.get_parameter("interpolation_period_s").value),
            0.01,
        )
        for left, right in zip(
            LEFT_ARM_DROP_WAYPOINTS,
            RIGHT_ARM_DROP_WAYPOINTS,
        ):
            self._left_publisher.publish(
                self._joint_message(LEFT_ARM_JOINT_NAMES, left)
            )
            self._right_publisher.publish(
                self._joint_message(RIGHT_ARM_JOINT_NAMES, right)
            )
            self._spin_for(interpolation_period_s)

    def run(self) -> None:
        if not bool(self.get_parameter("confirm").value):
            self.get_logger().error(
                "Motion is disabled. Re-run with "
                "'--ros-args -p confirm:=true' after confirming the "
                "emergency stop is released and the workspace is clear."
            )
            return

        self._wait_for_discovery()

        self.get_logger().warning(
            "Step 1/3: lowering legs to [0, 0, 0.3, 0, 0]"
        )
        self._publish_leg()
        self._spin_for(
            float(self.get_parameter("leg_settle_s").value)
        )

        self.get_logger().warning(
            "Step 2/3: keeping grippers half closed"
        )
        self._publish_gripper()

        self.get_logger().warning(
            "Step 3/3: lowering both arm TCPs to 0.10 m "
            f"while moving forward {FINAL_FORWARD_OFFSET_M:.2f} m "
            f"and preserving {GRAB_END_EFFECTOR_DISTANCE_M:.6f} m spacing "
            f"(end-effector roll {END_EFFECTOR_ROLL_RAD:.3f} rad)"
        )
        self._publish_arm_trajectory()
        self._spin_for(float(self.get_parameter("final_hold_s").value))
        self.get_logger().info(
            "Drop finished at TCP height "
            f"{FINAL_END_EFFECTOR_HEIGHT_M:.2f} m, forward "
            f"{FINAL_FORWARD_OFFSET_M:.2f} m"
        )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DropNode()
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
