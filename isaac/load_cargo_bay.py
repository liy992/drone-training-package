#!/usr/bin/env python3
"""Load a standalone cargo bay USD in Isaac Sim and expose a ROS 2 command topic.

Run this file with Isaac Sim's ``--exec`` path. It is intentionally independent
from the original cargo-delivery project runtime.
"""

from __future__ import annotations

import argparse
import asyncio
import atexit
import os
import traceback
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET = PACKAGE_ROOT / "assets/cargo_bay/cargo_bay_resized_0p2_0p14_0p06.usd"
DEFAULT_PRIM_PATH = "/World/transparent_cargo_bay"
COMMAND_TOPIC = "/cargo_bay/command"
STATUS_TOPIC = "/cargo_bay/status"

LEFT_CLOSED_DEG = 0.0
LEFT_OPEN_DEG = 80.0
BOTTOM_CLOSED_DEG = 0.0
BOTTOM_OPEN_DEG = -70.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("--asset", type=Path, default=DEFAULT_ASSET)
    parser.add_argument("--prim-path", default=DEFAULT_PRIM_PATH)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--no-ros", action="store_true")
    args, _unknown = parser.parse_known_args()
    return args


ARGS = parse_args()

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from omni.kit.async_engine import run_coroutine
from pxr import Gf, Sdf, UsdGeom, UsdPhysics

try:
    from isaacsim.core.utils.extensions import enable_extension
except Exception:  # Isaac Sim 4.x compatibility
    from omni.isaac.core.utils.extensions import enable_extension


APP = None


def _path_text(path: str | Sdf.Path) -> str:
    return str(path)


async def get_or_create_stage_async():
    context = omni.usd.get_context()
    stage = context.get_stage()
    if stage is not None:
        return stage

    if hasattr(context, "new_stage_async"):
        await context.new_stage_async()
    else:
        context.new_stage()
        await omni.kit.app.get_app().next_update_async()

    stage = context.get_stage()
    if stage is None:
        raise RuntimeError("Isaac USD stage is unavailable")
    return stage


def ensure_world_and_physics(stage) -> None:
    world = stage.GetPrimAtPath("/World")
    if not world or not world.IsValid():
        world = stage.DefinePrim("/World", "Xform")
    if not stage.GetDefaultPrim():
        stage.SetDefaultPrim(world)

    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr(9.81)


def set_or_add_xform_op(xformable, op_type, value) -> None:
    for op in xformable.GetOrderedXformOps():
        if op.GetOpType() == op_type:
            op.Set(value)
            return
    xformable.AddXformOp(op_type).Set(value)


def set_cargo_transform(stage, prim_path: str, x: float, y: float, z: float, yaw_deg: float, scale: float) -> None:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Cargo bay prim does not exist after load: {prim_path}")

    xformable = UsdGeom.Xformable(prim)
    set_or_add_xform_op(xformable, UsdGeom.XformOp.TypeTranslate, Gf.Vec3d(x, y, z))
    set_or_add_xform_op(xformable, UsdGeom.XformOp.TypeRotateZ, float(yaw_deg))
    set_or_add_xform_op(xformable, UsdGeom.XformOp.TypeScale, Gf.Vec3d(scale, scale, scale))


def load_cargo_reference(stage, asset_path: Path, prim_path: str) -> None:
    if prim_path != DEFAULT_PRIM_PATH:
        carb.log_warn(
            "The standalone cargo USD uses absolute joint relationships. "
            f"Default prim path {DEFAULT_PRIM_PATH} is safest; current prim path is {prim_path}."
        )

    existing = stage.GetPrimAtPath(prim_path)
    if existing and existing.IsValid():
        stage.RemovePrim(prim_path)

    prim = stage.DefinePrim(prim_path, "Xform")
    prim.GetReferences().AddReference(str(asset_path), DEFAULT_PRIM_PATH)


def find_joint(stage, root_path: str, exact_suffix: str, hints: tuple[str, ...]) -> str:
    exact = f"{root_path}/joints/{exact_suffix}"
    prim = stage.GetPrimAtPath(exact)
    if prim and prim.IsValid():
        return exact

    root = Sdf.Path(root_path)
    candidates: list[str] = []
    for prim in stage.Traverse():
        path = prim.GetPath()
        if path == root or not path.HasPrefix(root):
            continue
        name = prim.GetName().lower()
        type_name = str(prim.GetTypeName()).lower()
        if "joint" not in name and "joint" not in type_name:
            continue
        if any(hint in name for hint in hints):
            candidates.append(_path_text(path))

    if not candidates:
        raise RuntimeError(f"Could not find joint under {root_path} with hints={hints}")
    candidates.sort(key=len)
    return candidates[0]


class CargoBayRuntime:
    def __init__(self, stage, prim_path: str):
        self.stage = stage
        self.prim_path = prim_path
        self.left_joint = find_joint(stage, prim_path, "left_door_hinge_joint", ("left", "side"))
        self.bottom_joint = find_joint(stage, prim_path, "bottom_door_hinge_joint", ("bottom",))
        self.left_state = "closed"
        self.bottom_state = "closed"

    def set_door_angle(self, joint_path: str, angle_deg: float) -> None:
        joint = self.stage.GetPrimAtPath(joint_path)
        if not joint or not joint.IsValid():
            raise RuntimeError(f"Cargo bay joint not found: {joint_path}")
        attr = joint.GetAttribute("drive:angular:physics:targetPosition")
        if not attr or not attr.IsValid():
            attr = joint.CreateAttribute("drive:angular:physics:targetPosition", Sdf.ValueTypeNames.Float)
        attr.Set(float(angle_deg))

    def close_all(self) -> None:
        self.set_door_angle(self.left_joint, LEFT_CLOSED_DEG)
        self.set_door_angle(self.bottom_joint, BOTTOM_CLOSED_DEG)
        self.left_state = "closed"
        self.bottom_state = "closed"

    def handle_command(self, command: str) -> str:
        command = command.strip().lower()
        if command in {"left_open", "side_open"}:
            self.set_door_angle(self.left_joint, LEFT_OPEN_DEG)
            self.left_state = "open"
            return "left_opened"
        if command in {"left_close", "side_close"}:
            self.set_door_angle(self.left_joint, LEFT_CLOSED_DEG)
            self.left_state = "closed"
            return "left_closed"
        if command == "bottom_open":
            self.set_door_angle(self.bottom_joint, BOTTOM_OPEN_DEG)
            self.bottom_state = "open"
            return "bottom_opened"
        if command == "bottom_close":
            self.set_door_angle(self.bottom_joint, BOTTOM_CLOSED_DEG)
            self.bottom_state = "closed"
            return "bottom_closed"
        if command == "status":
            return self.status_text()
        return f"unknown_command {command}"

    def status_text(self) -> str:
        return (
            f"cargo_bay prim={self.prim_path} left={self.left_state} "
            f"bottom={self.bottom_state} left_joint={self.left_joint} "
            f"bottom_joint={self.bottom_joint}"
        )


class RosCargoBayInterface:
    def __init__(self, runtime: CargoBayRuntime):
        self.runtime = runtime
        self.rclpy = None
        self.node = None
        self.executor = None
        self.String = None
        self.status_pub = None
        self.owns_rclpy = False

    def start(self) -> bool:
        bridge_enabled = False
        for extension_id in ("isaacsim.ros2.bridge", "omni.isaac.ros2_bridge"):
            try:
                enable_extension(extension_id)
                bridge_enabled = True
                break
            except Exception as exc:
                carb.log_warn(f"Could not enable ROS 2 bridge extension {extension_id}: {exc}")

        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from std_msgs.msg import String
        except Exception as exc:
            carb.log_warn(f"Cargo bay ROS 2 topic interface disabled: {exc}")
            return False

        self.rclpy = rclpy
        self.String = String
        if not rclpy.ok():
            rclpy.init(args=None)
            self.owns_rclpy = True
        self.node = rclpy.create_node(f"training_cargo_bay_{os.getpid()}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.node.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
        self.status_pub = self.node.create_publisher(String, STATUS_TOPIC, 10)
        carb.log_warn(
            "Cargo bay ROS 2 interface ready: "
            f"subscribe={COMMAND_TOPIC} publish={STATUS_TOPIC} bridge_enabled={bridge_enabled}"
        )
        self.publish_status(self.runtime.status_text())
        return True

    def spin_once(self) -> None:
        if self.executor is not None:
            self.executor.spin_once(timeout_sec=0.0)

    def publish_status(self, text: str) -> None:
        if self.status_pub is None:
            return
        msg = self.String()
        msg.data = text
        self.status_pub.publish(msg)

    def _on_command(self, msg) -> None:
        command = msg.data.strip().lower()
        try:
            status = self.runtime.handle_command(command)
        except Exception as exc:
            carb.log_error(f"Cargo bay command failed: {exc}")
            status = f"error {exc}"
        self.publish_status(status)
        carb.log_warn(f"Cargo bay command={command} status={status}")

    def shutdown(self) -> None:
        if self.executor is not None and self.node is not None:
            self.executor.remove_node(self.node)
        if self.node is not None:
            self.node.destroy_node()
        if self.owns_rclpy and self.rclpy is not None and self.rclpy.ok():
            self.rclpy.shutdown()


class StandaloneCargoBayApp:
    def __init__(self, ros_interface: RosCargoBayInterface | None):
        self.ros_interface = ros_interface
        self.update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(self._on_update, name="training_cargo_bay_update")
        )

    def _on_update(self, _event) -> None:
        if self.ros_interface is not None:
            self.ros_interface.spin_once()

    def shutdown(self) -> None:
        if self.ros_interface is not None:
            self.ros_interface.shutdown()
        self.update_sub = None


async def main_async() -> None:
    global APP

    asset_path = ARGS.asset.expanduser()
    if not asset_path.is_absolute():
        asset_path = (PACKAGE_ROOT / asset_path).resolve()
    if not asset_path.exists():
        raise FileNotFoundError(f"Cargo bay asset does not exist: {asset_path}")

    stage = await get_or_create_stage_async()
    ensure_world_and_physics(stage)
    load_cargo_reference(stage, asset_path, ARGS.prim_path)
    set_cargo_transform(stage, ARGS.prim_path, ARGS.x, ARGS.y, ARGS.z, ARGS.yaw_deg, ARGS.scale)

    runtime = CargoBayRuntime(stage, ARGS.prim_path)
    runtime.close_all()

    ros_interface = None
    if not ARGS.no_ros:
        ros_interface = RosCargoBayInterface(runtime)
        if not ros_interface.start():
            ros_interface = None

    APP = StandaloneCargoBayApp(ros_interface)
    atexit.register(APP.shutdown)

    omni.timeline.get_timeline_interface().play()
    carb.log_warn(
        "Loaded standalone cargo bay "
        f"asset={asset_path} prim={ARGS.prim_path} command_topic={COMMAND_TOPIC} "
        f"status_topic={STATUS_TOPIC} ros_enabled={ros_interface is not None}"
    )


def on_done(task) -> None:
    try:
        exc = task.exception()
    except asyncio.CancelledError:
        return
    if exc is not None:
        carb.log_error("".join(traceback.format_exception(type(exc), exc, exc.__traceback__)))


task = run_coroutine(main_async())
task.add_done_callback(on_done)
