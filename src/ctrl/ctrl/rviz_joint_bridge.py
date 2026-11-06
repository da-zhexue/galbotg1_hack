"""Drive RViz joint states from the shared Galbot control topics."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from ctrl_msgs.msg import (
    GripperCommand,
    JointGroupCommand,
    RobotStateSnapshot,
    WheelCommand,
)

from .joint_utils import (
    COMPARABLE_JOINT_NAMES,
    GROUP_PREFIXES,
    LEFT_END_EFFECTOR_FRAME,
    LEFT_GRIPPER_JOINT_NAME,
    RIGHT_END_EFFECTOR_FRAME,
    RIGHT_GRIPPER_JOINT_NAME,
    STATE_REFERENCE_FRAME,
    TOPIC_GRIPPER,
    TOPIC_WHEELS,
    WHEEL_JOINT_NAMES,
    VISUALIZED_JOINT_NAMES,
    gripper_width_to_joint,
    validate_joint_names,
)
from .state_utils import pose_from_values


class RvizJointBridge(Node):
    """Translate control topics into sensor_msgs/JointState for RViz."""

    def __init__(self) -> None:
        super().__init__("rviz_joint_bridge")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("wheel_command_timeout_s", 0.5)
        self.declare_parameter("gripper_max_width_m", 0.12)
        self.declare_parameter("gripper_open_angle_rad", 1.703)
        self.declare_parameter("state_topic", "/ctrl/state/sim")
        self.declare_parameter("state_publish_rate_hz", 10.0)
        self.declare_parameter("reference_frame", STATE_REFERENCE_FRAME)
        self.declare_parameter(
            "left_end_effector_frame", LEFT_END_EFFECTOR_FRAME
        )
        self.declare_parameter(
            "right_end_effector_frame", RIGHT_END_EFFECTOR_FRAME
        )

        self._publish_rate_hz = max(
            float(self.get_parameter("publish_rate_hz").value), 1.0
        )
        self._wheel_timeout_s = float(
            self.get_parameter("wheel_command_timeout_s").value
        )
        self._gripper_max_width_m = float(
            self.get_parameter("gripper_max_width_m").value
        )
        self._gripper_open_angle_rad = float(
            self.get_parameter("gripper_open_angle_rad").value
        )
        self._state_publish_rate_hz = max(
            float(self.get_parameter("state_publish_rate_hz").value), 1.0
        )
        self._reference_frame = str(
            self.get_parameter("reference_frame").value
        )
        self._left_ee_frame = str(
            self.get_parameter("left_end_effector_frame").value
        )
        self._right_ee_frame = str(
            self.get_parameter("right_end_effector_frame").value
        )

        self._position = {name: 0.0 for name in VISUALIZED_JOINT_NAMES}
        self._velocity = {name: 0.0 for name in VISUALIZED_JOINT_NAMES}
        self._effort = {name: 0.0 for name in VISUALIZED_JOINT_NAMES}
        self._wheel_velocity = [0.0, 0.0, 0.0, 0.0]
        self._last_wheel_message_s = 0.0
        self._last_publish_s = time.monotonic()

        topic = str(self.get_parameter("joint_state_topic").value)
        self._publisher = self.create_publisher(JointState, topic, 10)
        self._state_publisher = self.create_publisher(
            RobotStateSnapshot,
            str(self.get_parameter("state_topic").value),
            10,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        for command_topic in GROUP_PREFIXES:
            self.create_subscription(
                JointGroupCommand,
                command_topic,
                lambda msg, command_topic=command_topic: self._on_joint_group(
                    msg, command_topic
                ),
                10,
            )
        self.create_subscription(
            WheelCommand, TOPIC_WHEELS, self._on_wheels, 10
        )
        self.create_subscription(
            GripperCommand, TOPIC_GRIPPER, self._on_gripper, 10
        )
        self.create_timer(
            1.0 / self._publish_rate_hz, self._publish_joint_state
        )
        self.create_timer(
            1.0 / self._state_publish_rate_hz, self._publish_state
        )
        self.get_logger().info(
            f"publishing {len(VISUALIZED_JOINT_NAMES)} joints on {topic}"
        )

    def _on_joint_group(
        self, msg: JointGroupCommand, topic: str
    ) -> None:
        names = list(msg.name)
        positions = list(msg.position)
        error = validate_joint_names(topic, names, positions)
        if error is not None:
            self.get_logger().error(f"{topic}: {error}")
            return

        velocity = list(msg.velocity)
        effort = list(msg.effort)
        if velocity and len(velocity) != len(names):
            self.get_logger().error(
                f"{topic}: velocity length must be zero or match names"
            )
            return
        if effort and len(effort) != len(names):
            self.get_logger().error(
                f"{topic}: effort length must be zero or match names"
            )
            return

        for index, name in enumerate(names):
            self._position[name] = float(positions[index])
            self._velocity[name] = (
                float(velocity[index]) if velocity else 0.0
            )
            self._effort[name] = float(effort[index]) if effort else 0.0

    def _on_wheels(self, msg: WheelCommand) -> None:
        values = [float(value) for value in msg.wheel_velocity]
        if len(values) != 4 or not all(math.isfinite(v) for v in values):
            self.get_logger().error(
                "wheel command must contain four finite velocities"
            )
            return
        self._wheel_velocity = values
        self._last_wheel_message_s = time.monotonic()
        requested_timeout = float(msg.timeout_s)
        if requested_timeout > 0.0:
            self._wheel_timeout_s = requested_timeout

    def _on_gripper(self, msg: GripperCommand) -> None:
        velocity = float(msg.velocity_mps)
        effort = float(msg.effort)
        if not math.isfinite(velocity) or not math.isfinite(effort):
            self.get_logger().error(
                "gripper velocity and effort must be finite"
            )
            return
        velocity = max(velocity, 0.0)
        width_to_angle_velocity = (
            velocity / self._gripper_max_width_m
            * self._gripper_open_angle_rad
        )
        commands = (
            (
                LEFT_GRIPPER_JOINT_NAME,
                bool(msg.control_left),
                float(msg.left_width_m),
            ),
            (
                RIGHT_GRIPPER_JOINT_NAME,
                bool(msg.control_right),
                float(msg.right_width_m),
            ),
        )
        for name, enabled, width in commands:
            if not enabled:
                continue
            try:
                angle = gripper_width_to_joint(
                    width,
                    self._gripper_max_width_m,
                    self._gripper_open_angle_rad,
                )
            except ValueError as error:
                self.get_logger().error(
                    f"{name} command failed: {error}"
                )
                continue
            self._position[name] = angle
            self._velocity[name] = width_to_angle_velocity
            self._effort[name] = effort

    def _publish_joint_state(self) -> None:
        now_s = time.monotonic()
        dt = max(now_s - self._last_publish_s, 0.0)
        self._last_publish_s = now_s

        if (
            now_s - self._last_wheel_message_s > self._wheel_timeout_s
        ):
            self._wheel_velocity = [0.0, 0.0, 0.0, 0.0]

        for index, name in enumerate(WHEEL_JOINT_NAMES):
            velocity = self._wheel_velocity[index]
            self._position[name] = math.remainder(
                self._position[name] + velocity * dt, 2.0 * math.pi
            )
            self._velocity[name] = velocity

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(VISUALIZED_JOINT_NAMES)
        message.position = [
            self._position[name] for name in VISUALIZED_JOINT_NAMES
        ]
        message.velocity = [
            self._velocity[name] for name in VISUALIZED_JOINT_NAMES
        ]
        message.effort = [
            self._effort[name] for name in VISUALIZED_JOINT_NAMES
        ]
        self._publisher.publish(message)

    def _lookup_pose(self, frame: str):
        try:
            transform = self._tf_buffer.lookup_transform(
                self._reference_frame, frame, Time()
            )
        except TransformException:
            return None
        values = [
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ]
        return pose_from_values(values)

    def _publish_state(self) -> None:
        message = RobotStateSnapshot()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._reference_frame
        message.source = "sim"
        message.reference_frame = self._reference_frame
        message.joint_names = list(COMPARABLE_JOINT_NAMES)
        message.joint_positions = [
            self._position[name] for name in COMPARABLE_JOINT_NAMES
        ]
        message.left_end_effector_frame = self._left_ee_frame
        message.right_end_effector_frame = self._right_ee_frame

        left_pose = self._lookup_pose(self._left_ee_frame)
        message.left_end_effector_valid = left_pose is not None
        if left_pose is not None:
            message.left_end_effector_pose = left_pose

        right_pose = self._lookup_pose(self._right_ee_frame)
        message.right_end_effector_valid = right_pose is not None
        if right_pose is not None:
            message.right_end_effector_pose = right_pose

        self._state_publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = RvizJointBridge()
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
