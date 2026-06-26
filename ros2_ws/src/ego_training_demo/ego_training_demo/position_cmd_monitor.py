"""Monitor EGO-Planner /position_cmd output for training demos."""

from __future__ import annotations

import math

import rclpy
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node

from ego_training_demo.core import DEFAULT_FRAME_ID, DEFAULT_POSITION_CMD_TOPIC


class PositionCommandMonitor(Node):
    def __init__(self) -> None:
        super().__init__("ego_training_position_cmd_monitor")
        self.topic = self.declare_parameter("position_cmd_topic", DEFAULT_POSITION_CMD_TOPIC).value
        self.expected_frame_id = self.declare_parameter("expected_frame_id", DEFAULT_FRAME_ID).value
        self.silent_timeout_s = float(self.declare_parameter("silent_timeout_s", 2.0).value)
        self.last_rx_ns = 0
        self.count = 0
        self.create_subscription(PositionCommand, self.topic, self.command_callback, 20)
        self.create_timer(1.0, self.health_check)

    def command_callback(self, msg: PositionCommand) -> None:
        self.count += 1
        self.last_rx_ns = self.get_clock().now().nanoseconds
        if msg.header.frame_id and msg.header.frame_id != self.expected_frame_id:
            self.get_logger().error(
                f"/position_cmd frame mismatch: expected {self.expected_frame_id}, got {msg.header.frame_id}"
            )
        if self.count == 1 or self.count % 20 == 0:
            self.get_logger().info(
                "position_cmd "
                f"pos=({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}) "
                f"vel=({msg.velocity.x:.2f}, {msg.velocity.y:.2f}, {msg.velocity.z:.2f})"
            )

    def health_check(self) -> None:
        if self.last_rx_ns == 0:
            self.get_logger().warn("Waiting for /position_cmd from EGO-Planner")
            return
        age_s = (self.get_clock().now().nanoseconds - self.last_rx_ns) / 1e9
        if age_s > self.silent_timeout_s and math.isfinite(age_s):
            self.get_logger().warn(
                f"/position_cmd has been silent for {age_s:.2f}s"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PositionCommandMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
