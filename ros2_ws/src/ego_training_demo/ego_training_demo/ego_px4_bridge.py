"""Convert EGO-Planner position commands into PX4 Offboard setpoints."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from ego_training_demo.core import (
    DEFAULT_EGO_ODOM_TOPIC,
    DEFAULT_POSITION_CMD_TOPIC,
    EgoPositionCommand,
    FrameHome,
    PX4_LOCAL_POSITION_TOPICS,
    coerce_bool,
    px4_setpoint_components,
)


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

PX4_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
)


class EgoPx4Bridge(Node):
    COMMAND_PERIOD_S = 0.05
    STALE_TIMEOUT_S = 0.5

    def __init__(self) -> None:
        super().__init__("ego_training_px4_bridge")
        self.position_cmd_topic = self.declare_parameter(
            "position_cmd_topic", DEFAULT_POSITION_CMD_TOPIC
        ).value
        self.auto_offboard = coerce_bool(self.declare_parameter("auto_offboard", True).value)
        self.auto_arm = coerce_bool(self.declare_parameter("auto_arm", True).value)
        self.odom_topic = self.declare_parameter("odom_topic", DEFAULT_EGO_ODOM_TOPIC).value

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS
        )

        self.create_subscription(
            PositionCommand, self.position_cmd_topic, self.position_command_callback, 20
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, SENSOR_QOS)
        for topic in PX4_LOCAL_POSITION_TOPICS:
            self.create_subscription(
                VehicleLocalPosition, topic, self.local_position_callback, PX4_OUT_QOS
            )

        self.latest_command: EgoPositionCommand | None = None
        self.latest_command_ns = 0
        self.latest_map_position: tuple[float, float, float] | None = None
        self.latest_px4_position: tuple[float, float, float] | None = None
        self.frame_home: FrameHome | None = None
        self.offboard_counter = 0
        self.mode_sent = False
        self.arm_sent = False
        self.start_ns = self.get_clock().now().nanoseconds
        self.create_timer(self.COMMAND_PERIOD_S, self.command_once)
        self.get_logger().info(
            "PX4 local position subscriptions: " + ", ".join(PX4_LOCAL_POSITION_TOPICS)
        )

    def position_command_callback(self, msg: PositionCommand) -> None:
        self.latest_command = EgoPositionCommand(
            position=(float(msg.position.x), float(msg.position.y), float(msg.position.z)),
            velocity=(float(msg.velocity.x), float(msg.velocity.y), float(msg.velocity.z)),
            acceleration=(
                float(msg.acceleration.x),
                float(msg.acceleration.y),
                float(msg.acceleration.z),
            ),
            yaw=float(msg.yaw),
            yaw_dot=float(msg.yaw_dot),
        )
        self.latest_command_ns = self.get_clock().now().nanoseconds

    def odom_callback(self, msg: Odometry) -> None:
        self.latest_map_position = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            float(msg.pose.pose.position.z),
        )
        self.establish_frame_home_if_ready()

    def local_position_callback(self, msg: VehicleLocalPosition) -> None:
        finite = math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)
        xy_valid = bool(getattr(msg, "xy_valid", True))
        z_valid = bool(getattr(msg, "z_valid", True))
        if not finite or not xy_valid or not z_valid:
            self.latest_px4_position = None
            self.get_logger().error(
                f"PX4 local position invalid: finite={finite} xy_valid={xy_valid} z_valid={z_valid}"
            )
            return
        self.latest_px4_position = (float(msg.x), float(msg.y), float(msg.z))
        self.establish_frame_home_if_ready()

    def establish_frame_home_if_ready(self) -> None:
        if self.frame_home is not None:
            return
        if self.latest_map_position is None or self.latest_px4_position is None:
            return
        self.frame_home = FrameHome(
            map_position=self.latest_map_position, px4_position=self.latest_px4_position
        )
        self.get_logger().info(
            "EGO/PX4 home established: "
            f"map={self.frame_home.map_position} px4={self.frame_home.px4_position}"
        )

    def command_once(self) -> None:
        if self.latest_command is None or self.command_stale():
            self._check_missing_inputs()
            return
        if self.frame_home is None:
            self.establish_frame_home_if_ready()
        if self.frame_home is None:
            self._check_missing_inputs()
            return

        self.publish_offboard_control_mode()
        self.offboard_counter += 1
        self.publish_trajectory_setpoint()

        if self.auto_offboard and not self.mode_sent and self.offboard_counter >= 20:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
            )
            self.mode_sent = True
        if self.auto_arm and not self.arm_sent and self.offboard_counter >= 25:
            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
            )
            self.arm_sent = True

    def _check_missing_inputs(self) -> None:
        age_s = (self.get_clock().now().nanoseconds - self.start_ns) / 1e9
        if age_s < 5.0:
            return
        if self.latest_map_position is None:
            self.get_logger().error(f"Missing odometry on {self.odom_topic}")
        if self.latest_px4_position is None:
            self.get_logger().error(
                "PX4 topics not ready; expected one of "
                + ", ".join(PX4_LOCAL_POSITION_TOPICS)
            )

    def command_stale(self) -> bool:
        now_ns = self.get_clock().now().nanoseconds
        return ((now_ns - self.latest_command_ns) / 1e9) > self.STALE_TIMEOUT_S

    def publish_offboard_control_mode(self) -> None:
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp_us()
        msg.position = True
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self) -> None:
        assert self.latest_command is not None
        assert self.frame_home is not None
        components = px4_setpoint_components(self.latest_command, self.frame_home)
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp_us()
        msg.position = list(components["position"])
        msg.velocity = list(components["velocity"])
        msg.acceleration = list(components["acceleration"])
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(components["yaw"])
        msg.yawspeed = float(components["yawspeed"])
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.timestamp_us()
        msg.command = int(command)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.command_pub.publish(msg)

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EgoPx4Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
