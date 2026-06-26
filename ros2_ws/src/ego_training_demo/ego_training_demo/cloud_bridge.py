"""Transform and filter Isaac LiDAR point clouds for EGO-Planner."""

from __future__ import annotations

from collections import deque

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener

from ego_training_demo.core import (
    DEFAULT_EGO_CLOUD_TOPIC,
    DEFAULT_FRAME_ID,
    DEFAULT_INPUT_CLOUD_TOPIC,
    coerce_bool,
)
from ego_training_demo.pointcloud_utils import (
    combine_point_clouds,
    filter_points_near_position,
    point_cloud_stamp_to_seconds,
    prune_accumulated_clouds,
    transform_point_cloud,
)


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
)


class TrainingPointCloudBridge(Node):
    """Bridge Isaac LiDAR data into the topic layout expected by EGO-Planner."""

    def __init__(self) -> None:
        super().__init__("ego_training_cloud_bridge")
        self.input_topic = self.declare_parameter("input_topic", DEFAULT_INPUT_CLOUD_TOPIC).value
        self.output_topic = self.declare_parameter("output_topic", DEFAULT_EGO_CLOUD_TOPIC).value
        self.target_frame = self.declare_parameter("target_frame", DEFAULT_FRAME_ID).value
        self.lookup_timeout_s = float(self.declare_parameter("lookup_timeout_s", 0.05).value)
        self.accumulate_cloud = coerce_bool(
            self.declare_parameter("accumulate_cloud", True).value
        )
        self.accumulation_window_s = float(
            self.declare_parameter("accumulation_window_s", 0.12).value
        )
        self.max_accumulated_frames = int(
            self.declare_parameter("max_accumulated_frames", 8).value
        )
        self.self_filter_min_range = float(
            self.declare_parameter("self_filter_min_range", 0.45).value
        )
        self.self_filter_xy_radius = float(
            self.declare_parameter("self_filter_xy_radius", 0.45).value
        )
        self.self_filter_z_radius = float(
            self.declare_parameter("self_filter_z_radius", 0.75).value
        )
        self.accumulated_clouds = deque(maxlen=max(self.max_accumulated_frames, 1))
        self.tf_failures = 0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cloud_pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.create_subscription(PointCloud2, self.input_topic, self.cloud_callback, SENSOR_QOS)

    def cloud_callback(self, msg: PointCloud2) -> None:
        if not msg.header.frame_id:
            self.get_logger().error("Point cloud frame_id is empty; cannot transform to map")
            return

        transform = self._lookup_transform(msg)
        if transform is None:
            return

        transformed = transform_point_cloud(msg, transform, self.target_frame)
        transformed.header.stamp = transform.header.stamp
        transformed = filter_points_near_position(
            transformed,
            center_xyz=(
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ),
            min_range=self.self_filter_min_range,
            xy_radius=self.self_filter_xy_radius,
            z_radius=self.self_filter_z_radius,
        )
        accumulated = self._accumulate(transformed)
        self.cloud_pub.publish(accumulated)

    def _lookup_transform(self, msg: PointCloud2):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                msg.header.stamp,
                timeout=Duration(seconds=self.lookup_timeout_s),
            )
            self.tf_failures = 0
            return transform
        except TransformException as exc:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=self.lookup_timeout_s),
                )
                self.tf_failures = 0
                self.get_logger().warn(
                    "Point cloud timestamp is ahead of TF; using latest transform instead",
                    throttle_duration_sec=1.0,
                )
                return transform
            except TransformException as latest_exc:
                self.tf_failures += 1
                if self.tf_failures >= 5:
                    self.get_logger().error(
                        "TF tree incomplete for point cloud bridge: "
                        f"need {self.target_frame} <- {msg.header.frame_id}; last error: {latest_exc}"
                    )
                else:
                    self.get_logger().warn(
                        f"Skipping point cloud, TF unavailable: {exc}",
                        throttle_duration_sec=1.0,
                    )
                return None

    def _accumulate(self, cloud: PointCloud2) -> PointCloud2:
        if not self.accumulate_cloud:
            return cloud

        latest_stamp_s = point_cloud_stamp_to_seconds(cloud)
        self.accumulated_clouds.append(cloud)
        pruned = prune_accumulated_clouds(
            list(self.accumulated_clouds),
            latest_stamp_s=latest_stamp_s,
            window_s=self.accumulation_window_s,
            max_frames=self.max_accumulated_frames,
        )
        self.accumulated_clouds = deque(pruned, maxlen=max(self.max_accumulated_frames, 1))

        try:
            return combine_point_clouds(self.accumulated_clouds, self.target_frame)
        except ValueError as exc:
            self.get_logger().warn(f"Resetting point cloud accumulator: {exc}")
            self.accumulated_clouds = deque([cloud], maxlen=max(self.max_accumulated_frames, 1))
            return cloud


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrainingPointCloudBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
