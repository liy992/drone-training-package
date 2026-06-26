"""Pure helpers shared by the training ROS 2 nodes and tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path


DEFAULT_DRONE_ID = 0
DEFAULT_FRAME_ID = "map"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_LIDAR_FRAME = "lidar_link"
DEFAULT_INPUT_CLOUD_TOPIC = "/training/lidar/pointcloud"
DEFAULT_EGO_ODOM_TOPIC = "/drone_0_ego_odom"
DEFAULT_EGO_CLOUD_TOPIC = "/drone_0_ego_cloud"
DEFAULT_GOAL_TOPIC = "/move_base_simple/goal"
DEFAULT_POSITION_CMD_TOPIC = "/position_cmd"
PX4_LOCAL_POSITION_TOPICS = (
    "/fmu/out/vehicle_local_position",
    "/fmu/out/vehicle_local_position_v1",
)


@dataclass(frozen=True)
class EgoPositionCommand:
    position: tuple[float, float, float]
    velocity: tuple[float, float, float]
    acceleration: tuple[float, float, float]
    yaw: float
    yaw_dot: float


@dataclass(frozen=True)
class FrameHome:
    map_position: tuple[float, float, float]
    px4_position: tuple[float, float, float]

    def map_to_px4_position(self, map_position: tuple[float, float, float]) -> list[float]:
        map_dx = float(map_position[0]) - float(self.map_position[0])
        map_dy = float(map_position[1]) - float(self.map_position[1])
        relative_up = float(map_position[2]) - float(self.map_position[2])
        return [
            float(self.px4_position[0]) + map_dy,
            float(self.px4_position[1]) + map_dx,
            float(self.px4_position[2]) - relative_up,
        ]


def coerce_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"true", "1", "yes", "on"}:
            return True
        if normalized in {"false", "0", "no", "off", ""}:
            return False
    return bool(value)


def px4_setpoint_components(command: EgoPositionCommand, frame_home: FrameHome) -> dict[str, list[float] | float]:
    return {
        "position": frame_home.map_to_px4_position(command.position),
        "velocity": [
            float(command.velocity[1]),
            float(command.velocity[0]),
            -float(command.velocity[2]),
        ],
        "acceleration": [
            float(command.acceleration[1]),
            float(command.acceleration[0]),
            -float(command.acceleration[2]),
        ],
        "yaw": math.nan,
        "yawspeed": math.nan,
    }


def ego_topic_name(suffix: str, drone_id: int = DEFAULT_DRONE_ID) -> str:
    normalized = suffix.lstrip("/")
    return f"/drone_{int(drone_id)}_{normalized}"


def ensure_ros_overlay_sourced(overlay_root: Path | str = "/home/robot-a/ros2_ws/install") -> Path:
    overlay_path = Path(overlay_root)
    if not overlay_path.exists():
        raise RuntimeError(
            "ROS 2 overlay is missing. Expected existing workspace install at "
            f"{overlay_path}"
        )
    return overlay_path
