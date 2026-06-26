"""Publish a one-shot or repeated goal for EGO-Planner training demos."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from ego_training_demo.core import DEFAULT_FRAME_ID, DEFAULT_GOAL_TOPIC


class TrainingGoalSender(Node):
    def __init__(self) -> None:
        super().__init__("ego_training_goal_sender")
        self.goal_topic = self.declare_parameter("goal_topic", DEFAULT_GOAL_TOPIC).value
        self.frame_id = self.declare_parameter("frame_id", DEFAULT_FRAME_ID).value
        self.goal_x = float(self.declare_parameter("x", 6.0).value)
        self.goal_y = float(self.declare_parameter("y", 0.0).value)
        self.goal_z = float(self.declare_parameter("z", 1.0).value)
        self.delay_s = float(self.declare_parameter("delay_s", 4.0).value)
        self.repeat_count = int(self.declare_parameter("repeat_count", 3).value)
        self.repeat_interval_s = float(self.declare_parameter("repeat_interval_s", 0.3).value)
        self._remaining = max(self.repeat_count, 1)
        self.pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.timer = self.create_timer(self.delay_s, self._publish_first)

    def _goal_msg(self) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = self.goal_z
        msg.pose.orientation.w = 1.0
        return msg

    def _publish_first(self) -> None:
        self.timer.cancel()
        self._publish_and_count()
        if self._remaining > 0:
            self.timer = self.create_timer(self.repeat_interval_s, self._publish_repeat)

    def _publish_repeat(self) -> None:
        self._publish_and_count()
        if self._remaining <= 0:
            self.timer.cancel()
            self.get_logger().info("Goal sender finished publishing default goal")
            self.destroy_timer(self.timer)
            self.create_timer(0.1, self._shutdown_once)

    def _publish_and_count(self) -> None:
        msg = self._goal_msg()
        self.pub.publish(msg)
        self._remaining -= 1
        self.get_logger().info(
            f"Published goal to {self.goal_topic}: "
            f"({self.goal_x:.2f}, {self.goal_y:.2f}, {self.goal_z:.2f}) frame={self.frame_id}"
        )

    def _shutdown_once(self) -> None:
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrainingGoalSender()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
