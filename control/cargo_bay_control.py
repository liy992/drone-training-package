#!/usr/bin/env python3
"""Publish commands to the standalone cargo bay ROS 2 String interface."""

from __future__ import annotations

import argparse
import time

import rclpy
from std_msgs.msg import String


COMMAND_TOPIC = "/cargo_bay/command"
STATUS_TOPIC = "/cargo_bay/status"
COMMANDS = {
    "left_open",
    "left_close",
    "side_open",
    "side_close",
    "bottom_open",
    "bottom_close",
    "status",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=sorted(COMMANDS))
    parser.add_argument("--topic", default=COMMAND_TOPIC)
    parser.add_argument("--status-topic", default=STATUS_TOPIC)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--interval", type=float, default=0.15)
    parser.add_argument("--wait-status", action="store_true")
    parser.add_argument("--timeout", type=float, default=2.0)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    statuses: list[str] = []

    rclpy.init(args=None)
    node = rclpy.create_node("training_cargo_bay_control")
    publisher = node.create_publisher(String, args.topic, 10)

    def on_status(msg: String) -> None:
        statuses.append(msg.data)

    wait_status = args.wait_status or args.command == "status"
    if wait_status:
        node.create_subscription(String, args.status_topic, on_status, 10)

    try:
        for _ in range(max(1, args.repeat)):
            msg = String()
            msg.data = args.command
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.0, args.interval))

        if wait_status:
            deadline = time.monotonic() + max(0.0, args.timeout)
            while time.monotonic() < deadline and not statuses:
                rclpy.spin_once(node, timeout_sec=0.05)

        print(f"published command={args.command} topic={args.topic}")
        if statuses:
            print(f"status: {statuses[-1]}")
        elif wait_status:
            print(f"no status received on {args.status_topic} within {args.timeout:.1f}s")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
