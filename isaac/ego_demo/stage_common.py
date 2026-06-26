#!/usr/bin/env python3
"""Shared Isaac Sim stage builder for the EGO-Planner training demos."""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence


PACKAGE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ASSET = (
    PACKAGE_ROOT
    / "assets/robots/sunray150_with_mid360_cargo/sunray150_with_mid360_cargo.usda"
)
DEFAULT_PRIM_PATH = "/World/quadrotor"
DEFAULT_PEGASUS_EXTENSION = (
    "/home/robot-a/Documents/PegasusSimulator/extensions/pegasus.simulator"
)

WORLD_PRIM = "/World"
GROUND_PRIM = "/World/ground"
OBSTACLE_ROOT = "/World/training_obstacles"
DRONE_BODY_PRIM = "/World/quadrotor/body"
MAP_FRAME = "map"
BASE_FRAME = "base_link"
LIDAR_FRAME = "lidar_link"
POINTCLOUD_TOPIC = "/training/lidar/pointcloud"
EGO_ODOM_TOPIC = "/drone_0_ego_odom"
LIDAR_CONFIG = "OS0_REV7_128ch10hz512res"
LIDAR_TRANSLATION = (0.0, 0.0, 0.12)
LIDAR_ROTATION_XYZW = (0.0, 0.0, 0.0, 1.0)

CLOCK_GRAPH_PATH = "/World/EgoTrainingClockGraph"
LIDAR_GRAPH_PATH = "/World/EgoTrainingLidarGraph"
STATE_GRAPH_PATH = "/World/EgoTrainingStateGraph"


def _parse_args(default_no_px4_autolaunch: bool) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, add_help=False)
    parser.add_argument("--asset", type=Path, default=DEFAULT_ASSET)
    parser.add_argument("--prim-path", default=DEFAULT_PRIM_PATH)
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=1.0)
    parser.add_argument("--yaw-deg", type=float, default=0.0)
    parser.add_argument("--no-px4-autolaunch", action="store_true")
    parser.add_argument(
        "--pegasus-extension",
        default=os.environ.get("PEGASUS_EXTENSION", DEFAULT_PEGASUS_EXTENSION),
    )
    args, _unknown = parser.parse_known_args()
    if default_no_px4_autolaunch:
        args.no_px4_autolaunch = True
    return args


ARGS = None


def _ensure_python_paths() -> None:
    if str(PACKAGE_ROOT) not in sys.path:
        sys.path.insert(0, str(PACKAGE_ROOT))
    pegasus_extension = Path(ARGS.pegasus_extension).expanduser()
    if pegasus_extension.exists() and str(pegasus_extension) not in sys.path:
        sys.path.insert(0, str(pegasus_extension))


@dataclass(frozen=True)
class StageConfig:
    stage_name: str
    enable_lidar: bool
    enable_pointcloud: bool
    enable_state_bridge: bool
    default_no_px4_autolaunch: bool


def yaw_to_quat_xyzw(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(float(yaw_deg)) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def _vec3d(values: Sequence[float]):
    from pxr import Gf

    return Gf.Vec3d(float(values[0]), float(values[1]), float(values[2]))


def _quatd_xyzw(values: Sequence[float]):
    from pxr import Gf

    return Gf.Quatd(float(values[3]), float(values[0]), float(values[1]), float(values[2]))


class TrainingSceneApp:
    def __init__(self, config: StageConfig):
        self.config = config
        self.timeline = None
        self.pg = None
        self.world = None
        self.vehicle = None
        self.lidar_prim_path = None
        self._update_sub = None

    def get_or_create_world(self):
        from isaacsim.core.api.world import World

        world = World.instance()
        if world is not None and hasattr(world, "scene"):
            return world

        if world is not None:
            World.clear_instance()
        return World()

    def has_usable_physics_context(self, world) -> bool:
        try:
            physics_context = world.get_physics_context()
        except Exception:
            return False
        if physics_context is None:
            return False
        return getattr(physics_context, "_physx_interface", None) is not None

    async def ensure_world_ready_async(self, world) -> None:
        import carb

        if self.has_usable_physics_context(world):
            return
        carb.log_warn("Initializing Isaac World physics context for EGO training stage")
        await world.initialize_simulation_context_async()
        if not self.has_usable_physics_context(world):
            raise RuntimeError("Isaac World physics context is unavailable")

    def define_box(self, stage, path: str, translation, scale, color):
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics

        prim = stage.DefinePrim(path, "Cube")
        cube = UsdGeom.Cube(prim)
        cube.CreateSizeAttr(1.0)
        xformable = UsdGeom.Xformable(prim)
        xformable.ClearXformOpOrder()
        xformable.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translation]))
        xformable.AddScaleOp().Set(Gf.Vec3d(*[float(v) for v in scale]))
        cube.GetDisplayColorAttr().Set([Gf.Vec3f(*[float(v) for v in color])])
        UsdPhysics.CollisionAPI.Apply(prim)
        return prim

    def build_simple_scene(self, stage) -> None:
        from pxr import UsdGeom

        world = stage.GetPrimAtPath(WORLD_PRIM)
        if not world or not world.IsValid():
            stage.DefinePrim(WORLD_PRIM, "Xform")
        if stage.GetPrimAtPath(OBSTACLE_ROOT).IsValid():
            stage.RemovePrim(OBSTACLE_ROOT)
        stage.DefinePrim(OBSTACLE_ROOT, "Xform")

        self.define_box(
            stage,
            GROUND_PRIM,
            translation=(0.0, 0.0, -0.025),
            scale=(20.0, 20.0, 0.05),
            color=(0.85, 0.85, 0.85),
        )
        self.define_box(
            stage,
            f"{OBSTACLE_ROOT}/box_center",
            translation=(2.8, 0.0, 1.0),
            scale=(0.9, 0.9, 2.0),
            color=(0.85, 0.35, 0.35),
        )
        self.define_box(
            stage,
            f"{OBSTACLE_ROOT}/box_left",
            translation=(4.2, 1.5, 1.0),
            scale=(0.9, 0.9, 2.0),
            color=(0.35, 0.55, 0.85),
        )
        self.define_box(
            stage,
            f"{OBSTACLE_ROOT}/box_right",
            translation=(4.2, -1.5, 1.0),
            scale=(0.9, 0.9, 2.0),
            color=(0.35, 0.75, 0.45),
        )
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    def spawn_vehicle(self):
        from pegasus.simulator.logic.backends.px4_mavlink_backend import (
            PX4MavlinkBackend,
            PX4MavlinkBackendConfig,
        )
        from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
        from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

        import carb

        self.pg = PegasusInterface()
        self.world = self.get_or_create_world()
        self.pg._world = self.world
        multirotor_config = MultirotorConfig()
        if ARGS.no_px4_autolaunch:
            multirotor_config.backends = []
            carb.log_warn("PX4 backend disabled for this stage")
        else:
            mavlink_config = PX4MavlinkBackendConfig(
                {
                    "vehicle_id": 0,
                    "px4_autolaunch": True,
                    "px4_dir": self.pg.px4_path,
                    "px4_vehicle_model": self.pg.px4_default_airframe,
                }
            )
            multirotor_config.backends = [PX4MavlinkBackend(mavlink_config)]

        asset_path = Path(ARGS.asset).expanduser()
        if not asset_path.is_absolute():
            asset_path = (PACKAGE_ROOT / asset_path).resolve()
        if not asset_path.exists():
            raise FileNotFoundError(f"Drone asset does not exist: {asset_path}")

        orientation = list(yaw_to_quat_xyzw(ARGS.yaw_deg))
        self.vehicle = Multirotor(
            ARGS.prim_path,
            str(asset_path),
            0,
            [float(ARGS.x), float(ARGS.y), float(ARGS.z)],
            orientation,
            config=multirotor_config,
        )

    def hide_lidar_prims(self, lidar_prim) -> None:
        from pxr import UsdGeom

        stack = [lidar_prim]
        while stack:
            prim = stack.pop()
            if not prim or not prim.IsValid():
                continue
            imageable = UsdGeom.Imageable(prim)
            if imageable:
                imageable.MakeInvisible()
            stack.extend(list(prim.GetChildren()))

    def attach_hidden_lidar(self) -> str:
        import carb
        import omni.kit.commands
        from pxr import Gf

        try:
            from isaacsim.core.utils.extensions import enable_extension
        except Exception:
            from omni.isaac.core.utils.extensions import enable_extension

        enable_extension("isaacsim.core.nodes")
        enable_extension("isaacsim.ros2.bridge")
        _, lidar_prim = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path="/training_lidar",
            parent=DRONE_BODY_PRIM,
            config=LIDAR_CONFIG,
            translation=Gf.Vec3d(*LIDAR_TRANSLATION),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            visibility=False,
        )
        self.hide_lidar_prims(lidar_prim)
        self.lidar_prim_path = str(lidar_prim.GetPath())
        carb.log_warn(f"Hidden RTX LiDAR attached at {self.lidar_prim_path}")
        return self.lidar_prim_path

    def build_ros_clock_graph(self) -> None:
        import omni.graph.core as og

        og.Controller.edit(
            {"graph_path": CLOCK_GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ReadSimTime.inputs:resetOnStop", False),
                    ("PublishClock.inputs:topicName", "clock"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                    ("Context.outputs:context", "PublishClock.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishClock.inputs:timeStamp",
                    ),
                ],
            },
        )

    def build_lidar_publish_graph(self, lidar_prim_path: str) -> None:
        import omni.graph.core as og
        import omni.replicator.core as rep

        render_product = rep.create.render_product(
            lidar_prim_path, resolution=(1, 1), name="training_lidar"
        )
        og.Controller.edit(
            {"graph_path": LIDAR_GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("PointCloudPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("PointCloudPublish.inputs:renderProductPath", render_product.path),
                    ("PointCloudPublish.inputs:frameId", LIDAR_FRAME),
                    ("PointCloudPublish.inputs:nodeNamespace", ""),
                    ("PointCloudPublish.inputs:topicName", POINTCLOUD_TOPIC),
                    ("PointCloudPublish.inputs:type", "point_cloud"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PointCloudPublish.inputs:execIn"),
                ],
            },
        )

    def build_state_graph(self) -> None:
        import omni.graph.core as og

        og.Controller.edit(
            {"graph_path": STATE_GRAPH_PATH, "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                    ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("PublishBaseTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("PublishLidarTf", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
                    ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("ReadSimTime.inputs:resetOnStop", False),
                    ("PublishBaseTf.inputs:topicName", "tf"),
                    ("PublishBaseTf.inputs:parentFrameId", MAP_FRAME),
                    ("PublishBaseTf.inputs:childFrameId", BASE_FRAME),
                    ("PublishBaseTf.inputs:rotation", [0.0, 0.0, 0.0, 1.0]),
                    ("PublishLidarTf.inputs:topicName", "tf_static"),
                    ("PublishLidarTf.inputs:parentFrameId", BASE_FRAME),
                    ("PublishLidarTf.inputs:childFrameId", LIDAR_FRAME),
                    ("PublishLidarTf.inputs:translation", list(LIDAR_TRANSLATION)),
                    ("PublishLidarTf.inputs:rotation", list(LIDAR_ROTATION_XYZW)),
                    ("PublishLidarTf.inputs:staticPublisher", True),
                    ("PublishOdom.inputs:topicName", EGO_ODOM_TOPIC),
                    ("PublishOdom.inputs:odomFrameId", MAP_FRAME),
                    ("PublishOdom.inputs:chassisFrameId", BASE_FRAME),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnPlaybackTick.outputs:tick", "PublishBaseTf.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishLidarTf.inputs:execIn"),
                    ("OnPlaybackTick.outputs:tick", "PublishOdom.inputs:execIn"),
                    ("Context.outputs:context", "PublishBaseTf.inputs:context"),
                    ("Context.outputs:context", "PublishLidarTf.inputs:context"),
                    ("Context.outputs:context", "PublishOdom.inputs:context"),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishBaseTf.inputs:timeStamp",
                    ),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishLidarTf.inputs:timeStamp",
                    ),
                    (
                        "ReadSimTime.outputs:simulationTime",
                        "PublishOdom.inputs:timeStamp",
                    ),
                ],
            },
        )
        self.update_state_graph()

    def set_state_graph_input(self, node_name: str, input_name: str, value) -> None:
        import omni.graph.core as og

        og.Controller.attribute(
            f"{STATE_GRAPH_PATH}/{node_name}.inputs:{input_name}"
        ).set(value)

    def update_state_graph(self) -> None:
        if self.vehicle is None:
            return
        state = self.vehicle.state
        position = [float(state.position[0]), float(state.position[1]), float(state.position[2])]
        attitude = [float(state.attitude[0]), float(state.attitude[1]), float(state.attitude[2]), float(state.attitude[3])]
        linear_velocity = [
            float(state.linear_velocity[0]),
            float(state.linear_velocity[1]),
            float(state.linear_velocity[2]),
        ]
        angular_velocity = [
            float(state.angular_velocity[0]),
            float(state.angular_velocity[1]),
            float(state.angular_velocity[2]),
        ]
        self.set_state_graph_input("PublishBaseTf", "translation", position)
        self.set_state_graph_input("PublishBaseTf", "rotation", attitude)
        self.set_state_graph_input("PublishOdom", "position", position)
        self.set_state_graph_input("PublishOdom", "orientation", attitude)
        self.set_state_graph_input("PublishOdom", "linearVelocity", linear_velocity)
        self.set_state_graph_input("PublishOdom", "angularVelocity", angular_velocity)

    async def setup_async(self) -> None:
        import carb
        import omni.kit.app
        import omni.timeline

        self.timeline = omni.timeline.get_timeline_interface()
        self.world = self.get_or_create_world()
        await self.ensure_world_ready_async(self.world)
        self.build_simple_scene(self.world.stage)
        await omni.kit.app.get_app().next_update_async()
        self.spawn_vehicle()
        await omni.kit.app.get_app().next_update_async()

        if self.config.enable_lidar or self.config.enable_pointcloud or self.config.enable_state_bridge:
            self.attach_hidden_lidar()

        if self.config.enable_pointcloud or self.config.enable_state_bridge:
            self.build_ros_clock_graph()
        if self.config.enable_pointcloud:
            self.build_lidar_publish_graph(self.lidar_prim_path)
        if self.config.enable_state_bridge:
            self.build_state_graph()

        await self.world.reset_async()
        self._update_sub = (
            omni.kit.app.get_app()
            .get_update_event_stream()
            .create_subscription_to_pop(
                self._on_update,
                name=f"{self.config.stage_name}_update",
            )
        )
        self.timeline.play()
        if self.pg is not None:
            self.pg.set_viewport_camera(
                (float(ARGS.x) - 2.5, float(ARGS.y) - 2.5, float(ARGS.z) + 1.4),
                (float(ARGS.x), float(ARGS.y), float(ARGS.z)),
            )
        carb.log_warn(f"{self.config.stage_name} ready")
        carb.log_warn(
            "Obstacle layout ready: 3 boxes between start (0, 0, 1.0) and goal (6, 0, 1.0)"
        )
        if self.config.enable_pointcloud:
            carb.log_warn(f"Point cloud topic: {POINTCLOUD_TOPIC}")
        if self.config.enable_state_bridge:
            carb.log_warn(f"Odometry topic: {EGO_ODOM_TOPIC}")
            carb.log_warn(f"TF tree: {MAP_FRAME} -> {BASE_FRAME} -> {LIDAR_FRAME}")

    def _on_update(self, _event) -> None:
        if self.config.enable_state_bridge:
            self.update_state_graph()

    def shutdown(self) -> None:
        if self.timeline is not None:
            self.timeline.stop()
        self._update_sub = None


SCENE_APP = None
SCENE_TASK = None


def run_stage(config: StageConfig) -> None:
    global ARGS, SCENE_APP, SCENE_TASK

    ARGS = _parse_args(config.default_no_px4_autolaunch)
    _ensure_python_paths()

    import carb
    from omni.kit.async_engine import run_coroutine

    async def main_async() -> None:
        global SCENE_APP
        if SCENE_APP is not None:
            SCENE_APP.shutdown()
        app = TrainingSceneApp(config)
        await app.setup_async()
        SCENE_APP = app

    def on_done(task) -> None:
        try:
            exc = task.exception()
        except asyncio.CancelledError:
            return
        if exc is not None:
            carb.log_error(
                "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
            )

    SCENE_TASK = run_coroutine(main_async())
    SCENE_TASK.add_done_callback(on_done)
