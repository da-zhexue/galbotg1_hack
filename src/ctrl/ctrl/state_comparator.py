"""Compare unified real-robot and simulator state snapshots."""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node

from ctrl_msgs.msg import RobotStateError, RobotStateSnapshot

from .state_utils import orientation_error, position_error


class StateComparator(Node):
    """Publish joint and end-effector errors between real and simulated data."""

    def __init__(self) -> None:
        super().__init__("state_comparator")

        self.declare_parameter("real_state_topic", "/ctrl/state/real")
        self.declare_parameter("sim_state_topic", "/ctrl/state/sim")
        self.declare_parameter("error_topic", "/ctrl/state/error")
        self.declare_parameter("compare_rate_hz", 10.0)
        self.declare_parameter("log_rate_hz", 1.0)
        self.declare_parameter("max_state_age_s", 1.0)

        self._compare_rate_hz = max(
            float(self.get_parameter("compare_rate_hz").value), 1.0
        )
        self._log_period_s = 1.0 / max(
            float(self.get_parameter("log_rate_hz").value), 1.0
        )
        self._last_log_s = 0.0
        self._max_state_age_s = max(
            float(self.get_parameter("max_state_age_s").value), 0.1
        )
        self._real_received_s = 0.0
        self._sim_received_s = 0.0
        self._real_snapshot: RobotStateSnapshot | None = None
        self._sim_snapshot: RobotStateSnapshot | None = None

        self._publisher = self.create_publisher(
            RobotStateError,
            str(self.get_parameter("error_topic").value),
            10,
        )
        self.create_subscription(
            RobotStateSnapshot,
            str(self.get_parameter("real_state_topic").value),
            self._on_real_state,
            10,
        )
        self.create_subscription(
            RobotStateSnapshot,
            str(self.get_parameter("sim_state_topic").value),
            self._on_sim_state,
            10,
        )
        self.create_timer(
            1.0 / self._compare_rate_hz, self._publish_comparison
        )

    def _on_real_state(self, message: RobotStateSnapshot) -> None:
        self._real_snapshot = message
        self._real_received_s = time.monotonic()

    def _on_sim_state(self, message: RobotStateSnapshot) -> None:
        self._sim_snapshot = message
        self._sim_received_s = time.monotonic()

    def _publish_comparison(self) -> None:
        real = self._real_snapshot
        simulated = self._sim_snapshot
        if real is None or simulated is None:
            return
        now_s = time.monotonic()
        if (
            now_s - self._real_received_s > self._max_state_age_s
            or now_s - self._sim_received_s > self._max_state_age_s
        ):
            return

        real_joints = dict(
            zip(real.joint_names, real.joint_positions)
        )
        sim_joints = dict(
            zip(simulated.joint_names, simulated.joint_positions)
        )
        common_names = [
            name for name in real.joint_names if name in sim_joints
        ]

        message = RobotStateError()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = common_names
        message.joint_errors = [
            abs(real_joints[name] - sim_joints[name])
            for name in common_names
        ]
        if message.joint_errors:
            max_index = max(
                range(len(message.joint_errors)),
                key=message.joint_errors.__getitem__,
            )
            message.max_joint_error = message.joint_errors[max_index]
            message.max_joint_name = common_names[max_index]

        message.left_end_effector_valid = (
            real.left_end_effector_valid
            and simulated.left_end_effector_valid
        )
        if message.left_end_effector_valid:
            message.left_position_error_m = position_error(
                real.left_end_effector_pose,
                simulated.left_end_effector_pose,
            )
            message.left_orientation_error_rad = orientation_error(
                real.left_end_effector_pose,
                simulated.left_end_effector_pose,
            )

        message.right_end_effector_valid = (
            real.right_end_effector_valid
            and simulated.right_end_effector_valid
        )
        if message.right_end_effector_valid:
            message.right_position_error_m = position_error(
                real.right_end_effector_pose,
                simulated.right_end_effector_pose,
            )
            message.right_orientation_error_rad = orientation_error(
                real.right_end_effector_pose,
                simulated.right_end_effector_pose,
            )

        self._publisher.publish(message)
        if now_s - self._last_log_s >= self._log_period_s:
            self._last_log_s = now_s
            self.get_logger().info(
                "max joint error "
                f"{message.max_joint_error:.5f} rad "
                f"({message.max_joint_name or 'n/a'}), "
                "left EE "
                f"{message.left_position_error_m:.5f} m / "
                f"{message.left_orientation_error_rad:.5f} rad, "
                "right EE "
                f"{message.right_position_error_m:.5f} m / "
                f"{message.right_orientation_error_rad:.5f} rad"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StateComparator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
