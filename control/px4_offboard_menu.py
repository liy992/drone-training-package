#!/usr/bin/env python3
"""Interactive PX4 Offboard menu for training.

This script assumes PX4 ROS 2 topics are already available. It publishes only
standard PX4 input topics and is intentionally independent from any mission
runtime.
"""

from __future__ import annotations

import math
import shlex
import threading
import time
from dataclasses import dataclass
from typing import Iterable

import rclpy
from rclpy._rclpy_pybind11 import RCLError
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


PX4_OFFBOARD_TOPIC = "/fmu/in/offboard_control_mode"
PX4_TRAJECTORY_TOPIC = "/fmu/in/trajectory_setpoint"
PX4_COMMAND_TOPIC = "/fmu/in/vehicle_command"
PX4_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
PX4_STATUS_TOPIC = "/fmu/out/vehicle_status"
PX4_LOCAL_POSITION_TOPICS = (
    PX4_LOCAL_POSITION_TOPIC,
    "/fmu/out/vehicle_local_position_v1",
)
PX4_STATUS_TOPICS = (
    PX4_STATUS_TOPIC,
    "/fmu/out/vehicle_status_v3",
)

PX4_IN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

PX4_OUT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


@dataclass
class LocalPosition:
    x: float
    y: float
    z: float
    heading: float


ARMING_STATE_NAMES = {
    VehicleStatus.ARMING_STATE_DISARMED: "DISARMED",
    VehicleStatus.ARMING_STATE_ARMED: "ARMED",
}

NAV_STATE_NAMES = {
    VehicleStatus.NAVIGATION_STATE_MANUAL: "MANUAL",
    VehicleStatus.NAVIGATION_STATE_ALTCTL: "ALTCTL",
    VehicleStatus.NAVIGATION_STATE_POSCTL: "POSCTL",
    VehicleStatus.NAVIGATION_STATE_AUTO_MISSION: "AUTO_MISSION",
    VehicleStatus.NAVIGATION_STATE_AUTO_LOITER: "AUTO_LOITER",
    VehicleStatus.NAVIGATION_STATE_AUTO_RTL: "AUTO_RTL",
    VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF: "AUTO_TAKEOFF",
    VehicleStatus.NAVIGATION_STATE_AUTO_LAND: "AUTO_LAND",
    VehicleStatus.NAVIGATION_STATE_OFFBOARD: "OFFBOARD",
}


def arming_state_name(value: int) -> str:
    return ARMING_STATE_NAMES.get(int(value), f"UNKNOWN({int(value)})")


def nav_state_name(value: int) -> str:
    return NAV_STATE_NAMES.get(int(value), f"UNKNOWN({int(value)})")


class Px4OffboardMenu(Node):
    def __init__(self) -> None:
        super().__init__("training_px4_offboard_menu")
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, PX4_OFFBOARD_TOPIC, PX4_IN_QOS
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, PX4_TRAJECTORY_TOPIC, PX4_IN_QOS
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, PX4_COMMAND_TOPIC, PX4_IN_QOS
        )
        self.local_position_topic: str | None = None
        self.status_topic: str | None = None
        self._training_subscriptions = []
        for topic in PX4_LOCAL_POSITION_TOPICS:
            self._training_subscriptions.append(
                self.create_subscription(
                    VehicleLocalPosition,
                    topic,
                    lambda msg, topic=topic: self.local_position_cb(msg, topic),
                    PX4_OUT_QOS,
                )
            )
        for topic in PX4_STATUS_TOPICS:
            self._training_subscriptions.append(
                self.create_subscription(
                    VehicleStatus,
                    topic,
                    lambda msg, topic=topic: self.vehicle_status_cb(msg, topic),
                    PX4_OUT_QOS,
                )
            )

        self.local_position: LocalPosition | None = None
        self.vehicle_status: VehicleStatus | None = None
        self.target_position: list[float] | None = None
        self.target_yaw: float | None = None
        self.stream_enabled = False
        self.running = True
        self.state_lock = threading.Lock()
        self.timer = self.create_timer(0.05, self.publish_stream)

    def timestamp_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def local_position_cb(self, msg: VehicleLocalPosition, topic: str) -> None:
        if not (math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)):
            return
        with self.state_lock:
            self.local_position = LocalPosition(
                x=float(msg.x),
                y=float(msg.y),
                z=float(msg.z),
                heading=float(msg.heading) if math.isfinite(msg.heading) else 0.0,
            )
            self.local_position_topic = topic

    def vehicle_status_cb(self, msg: VehicleStatus, topic: str) -> None:
        with self.state_lock:
            self.vehicle_status = msg
            self.status_topic = topic

    def publish_stream(self) -> None:
        with self.state_lock:
            enabled = self.stream_enabled
            target = list(self.target_position) if self.target_position is not None else None
            yaw = self.target_yaw
        if not enabled or target is None:
            return

        now = self.timestamp_us()
        mode = OffboardControlMode()
        mode.timestamp = now
        mode.position = True
        self.offboard_pub.publish(mode)

        setpoint = TrajectorySetpoint()
        setpoint.timestamp = now
        setpoint.position = [float(target[0]), float(target[1]), float(target[2])]
        if yaw is not None:
            setpoint.yaw = float(yaw)
        self.setpoint_pub.publish(setpoint)

    def send_vehicle_command(
        self, command: int, param1: float = 0.0, param2: float = 0.0
    ) -> None:
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

    def send_vehicle_command_repeated(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
        count: int = 3,
        interval_s: float = 0.1,
    ) -> None:
        for _ in range(max(1, int(count))):
            self.send_vehicle_command(command, param1, param2)
            time.sleep(max(0.0, float(interval_s)))

    def wait_for_position(self, timeout_s: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self.state_lock:
                ready = self.local_position is not None
            if ready:
                return True
            time.sleep(0.05)
        return False

    def current_position(self, timeout_s: float = 5.0) -> LocalPosition:
        self.wait_for_position(timeout_s=timeout_s)
        with self.state_lock:
            position = self.local_position
        if position is None:
            raise RuntimeError(
                "No PX4 local position received on "
                f"{', '.join(PX4_LOCAL_POSITION_TOPICS)}"
            )
        return position

    def current_status(self, timeout_s: float = 1.0) -> VehicleStatus | None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            with self.state_lock:
                status = self.vehicle_status
            if status is not None:
                return status
            time.sleep(0.05)
        with self.state_lock:
            return self.vehicle_status

    def is_offboard(self) -> bool:
        status = self.current_status(timeout_s=0.2)
        return status is not None and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def is_armed(self) -> bool:
        status = self.current_status(timeout_s=0.2)
        return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_ARMED

    def wait_for_offboard(self, timeout_s: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            if self.is_offboard():
                return True
            time.sleep(0.05)
        return self.is_offboard()

    def wait_for_armed(self, timeout_s: float = 3.0) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and rclpy.ok():
            if self.is_armed():
                return True
            time.sleep(0.05)
        return self.is_armed()

    def set_target(self, position: Iterable[float], yaw: float | None = None) -> None:
        values = [float(value) for value in position]
        if len(values) != 3:
            raise ValueError("Target position must contain exactly x y z")
        with self.state_lock:
            self.target_position = values
            self.target_yaw = yaw
            self.stream_enabled = True

    def enable_offboard_mode(self) -> None:
        self.send_vehicle_command_repeated(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0,
            count=5,
            interval_s=0.1,
        )

    def arm(self) -> None:
        self.send_vehicle_command_repeated(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            count=3,
            interval_s=0.1,
        )

    def arm_and_enable_offboard(self) -> None:
        self.enable_offboard_mode()
        self.arm()
        self.enable_offboard_mode()

    def require_offboard_for_movement(self) -> None:
        if self.is_offboard():
            return
        status = self.current_status(timeout_s=0.2)
        if status is None:
            state_text = "no vehicle status received"
        else:
            state_text = (
                f"arming={arming_state_name(status.arming_state)}, "
                f"nav={nav_state_name(status.nav_state)}"
            )
        raise RuntimeError(
            "PX4 is not in OFFBOARD mode. "
            f"Current state: {state_text}. "
            "Run 'takeoff 1.0' first, or run 'offboard' if the drone is already airborne."
        )

    def land(self) -> None:
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def status_text(self) -> str:
        with self.state_lock:
            position = self.local_position
            position_topic = self.local_position_topic
            status = self.vehicle_status
            status_topic = self.status_topic
            target = self.target_position
            streaming = self.stream_enabled
        lines = ["PX4 status:"]
        if position is None:
            lines.append(
                "  local position: no data on "
                f"{', '.join(PX4_LOCAL_POSITION_TOPICS)}"
            )
        else:
            lines.append(
                "  local position NED: "
                f"x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}, "
                f"heading={position.heading:.3f}"
            )
            lines.append(f"  local position topic: {position_topic}")
        if target is None:
            lines.append("  target: not set")
        else:
            lines.append(
                f"  target NED: x={target[0]:.3f}, y={target[1]:.3f}, z={target[2]:.3f}"
            )
        if status is None:
            lines.append(f"  vehicle status: no data on {', '.join(PX4_STATUS_TOPICS)}")
        else:
            lines.append(
                "  vehicle status: "
                f"arming={arming_state_name(status.arming_state)}({status.arming_state}), "
                f"nav={nav_state_name(status.nav_state)}({status.nav_state})"
            )
            lines.append(f"  vehicle status topic: {status_topic}")
        lines.append(f"  setpoint stream: {'on' if streaming else 'off'}")
        return "\n".join(lines)


def parse_xyz(parts: list[str]) -> list[float]:
    if len(parts) != 3:
        raise ValueError("Expected three numbers: x y z")
    return [float(parts[0]), float(parts[1]), float(parts[2])]


def print_help() -> None:
    print(
        """
Commands:
  status                  Show PX4 local position, vehicle status, and current target.
  takeoff [height]         Take off by relative height in meters. Default: 1.0.
  offboard                 Hold current position and switch PX4 to Offboard.
  demo                     Take off 1m, move +1m in PX4 local x, then hover.
  goto <x> <y> <z>         Set absolute PX4 local/NED target.
  move <dx> <dy> <dz>      Move relative to current target/current position in PX4 local/NED.
  hover                   Hold current PX4 local position.
  land                    Send PX4 land command.
  help                    Show this help.
  quit                    Exit this menu.

Coordinate reminder:
  PX4 local uses NED: x North, y East, z Down.
  Up is negative z. Example: takeoff 1.0 sends current_z - 1.0.
"""
    )


def command_loop(node: Px4OffboardMenu) -> None:
    print_help()
    while node.running and rclpy.ok():
        try:
            raw = input("px4> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            node.running = False
            break
        if not raw:
            continue
        try:
            parts = shlex.split(raw)
        except ValueError as exc:
            print(f"Input error: {exc}")
            continue
        command = parts[0].lower()
        args = parts[1:]

        try:
            if command in {"quit", "exit"}:
                node.running = False
                break
            if command == "help":
                print_help()
            elif command == "status":
                node.wait_for_position(timeout_s=1.0)
                print(node.status_text())
            elif command == "takeoff":
                height = float(args[0]) if args else 1.0
                if height <= 0.0:
                    raise ValueError("Takeoff height must be positive")
                if not node.wait_for_position():
                    raise RuntimeError("PX4 local position is not ready")
                current = node.current_position()
                node.set_target([current.x, current.y, current.z - height], current.heading)
                print("Streaming initial setpoint for 1.5 seconds before Offboard/arm...")
                time.sleep(1.5)
                node.arm_and_enable_offboard()
                offboard_ok = node.wait_for_offboard(timeout_s=3.0)
                armed_ok = node.wait_for_armed(timeout_s=3.0)
                print(
                    "Takeoff target set: "
                    f"x={current.x:.3f}, y={current.y:.3f}, z={current.z - height:.3f}"
                )
                if not offboard_ok or not armed_ok:
                    print(
                        "Warning: PX4 did not confirm ARMED/OFFBOARD yet. "
                        "Run 'status' to check current mode before using move/goto."
                    )
            elif command == "offboard":
                current = node.current_position()
                node.set_target([current.x, current.y, current.z], current.heading)
                print("Streaming current position setpoint for 1.5 seconds...")
                time.sleep(1.5)
                node.enable_offboard_mode()
                if node.wait_for_offboard(timeout_s=3.0):
                    print("PX4 confirmed OFFBOARD mode. You can now use move/goto.")
                else:
                    print(
                        "Warning: PX4 did not confirm OFFBOARD. "
                        "Run 'status' and check QGC/PX4 preflight state."
                    )
            elif command == "demo":
                current = node.current_position()
                takeoff_target = [current.x, current.y, current.z - 1.0]
                node.set_target(takeoff_target, current.heading)
                print("Demo: streaming takeoff target for 1.5 seconds...")
                time.sleep(1.5)
                node.arm_and_enable_offboard()
                if not node.wait_for_offboard(timeout_s=3.0):
                    raise RuntimeError("PX4 did not enter OFFBOARD; run 'status' and check QGC")
                if not node.wait_for_armed(timeout_s=3.0):
                    raise RuntimeError("PX4 did not arm; run 'status' and check QGC")
                print("Demo: holding takeoff target for 3 seconds...")
                time.sleep(3.0)
                move_target = [takeoff_target[0] + 1.0, takeoff_target[1], takeoff_target[2]]
                node.set_target(move_target, current.heading)
                print(f"Demo: moved target to PX4 local/NED: {move_target}")
            elif command == "goto":
                node.require_offboard_for_movement()
                target = parse_xyz(args)
                node.set_target(target)
                print(f"Target set to PX4 local/NED: {target}")
            elif command == "move":
                node.require_offboard_for_movement()
                delta = parse_xyz(args)
                with node.state_lock:
                    base = (
                        list(node.target_position)
                        if node.target_position is not None
                        else None
                    )
                if base is None:
                    current = node.current_position()
                    base = [current.x, current.y, current.z]
                target = [base[0] + delta[0], base[1] + delta[1], base[2] + delta[2]]
                node.set_target(target)
                print(f"Moved target to PX4 local/NED: {target}")
            elif command == "hover":
                current = node.current_position()
                node.set_target([current.x, current.y, current.z], current.heading)
                print("Hover target set to current PX4 local position")
            elif command == "land":
                node.land()
                print("Land command sent")
            else:
                print(f"Unknown command: {command}. Type 'help' for commands.")
        except Exception as exc:
            print(f"Command failed: {exc}")


def spin_node(node: Px4OffboardMenu) -> None:
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, rclpy.exceptions.ROSInterruptException):
        pass


def main() -> None:
    rclpy.init()
    node = Px4OffboardMenu()
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()
    try:
        command_loop(node)
    finally:
        node.running = False
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except RCLError:
            pass
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
