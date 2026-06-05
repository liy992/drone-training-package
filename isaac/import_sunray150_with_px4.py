#!/usr/bin/env python3
"""Load the training Sunray150 drone in Isaac Sim through Pegasus.

Run this file with Isaac Sim's Python execution path, not with system Python.
The launcher at the package root shows the expected command.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import os
import sys
import traceback
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ASSET = (
    PACKAGE_ROOT
    / "assets/robots/sunray150_with_mid360_cargo/sunray150_with_mid360_cargo.usda"
)
DEFAULT_PRIM_PATH = "/World/quadrotor"
DEFAULT_PEGASUS_EXTENSION = (
    "/home/robot-a/Documents/PegasusSimulator/extensions/pegasus.simulator"
)


def parse_args() -> argparse.Namespace:
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
    return args


ARGS = parse_args()

pegasus_extension = Path(ARGS.pegasus_extension).expanduser()
if pegasus_extension.exists() and str(pegasus_extension) not in sys.path:
    sys.path.insert(0, str(pegasus_extension))

import carb
import omni.kit.app
import omni.timeline
from isaacsim.core.api.world import World
from omni.kit.async_engine import run_coroutine
from pegasus.simulator.logic.backends.px4_mavlink_backend import (
    PX4MavlinkBackend,
    PX4MavlinkBackendConfig,
)
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig


def yaw_to_quat_xyzw(yaw_deg: float) -> tuple[float, float, float, float]:
    half = math.radians(float(yaw_deg)) * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def get_or_create_world(pg: PegasusInterface):
    world = World.instance()
    if world is not None and hasattr(world, "scene"):
        pg._world = world
        return pg.world

    if world is not None:
        World.clear_instance()

    if pg.world is not None and hasattr(pg.world, "scene"):
        return pg.world

    pg._world = World(**pg._world_settings)
    return pg.world


def has_usable_physics_context(world) -> bool:
    try:
        physics_context = world.get_physics_context()
    except Exception:
        return False
    if physics_context is None:
        return False
    return getattr(physics_context, "_physx_interface", None) is not None


async def ensure_world_ready_async(world) -> None:
    if has_usable_physics_context(world):
        return
    carb.log_warn("Initializing Isaac World physics context before loading training drone")
    await world.initialize_simulation_context_async()
    if not has_usable_physics_context(world):
        raise RuntimeError("Isaac World physics context is still unavailable")


async def main_async() -> None:
    asset_path = Path(ARGS.asset).expanduser()
    if not asset_path.is_absolute():
        asset_path = (PACKAGE_ROOT / asset_path).resolve()
    if not asset_path.exists():
        raise FileNotFoundError(f"Drone asset does not exist: {asset_path}")

    pg = PegasusInterface()
    world = get_or_create_world(pg)
    await ensure_world_ready_async(world)
    stage = world.stage

    existing = stage.GetPrimAtPath(ARGS.prim_path)
    if existing.IsValid():
        stage.RemovePrim(ARGS.prim_path)

    multirotor_config = MultirotorConfig()
    if ARGS.no_px4_autolaunch:
        multirotor_config.backends = []
    else:
        mavlink_config = PX4MavlinkBackendConfig(
            {
                "vehicle_id": 0,
                "px4_autolaunch": True,
                "px4_dir": pg.px4_path,
                "px4_vehicle_model": pg.px4_default_airframe,
            }
        )
        multirotor_config.backends = [PX4MavlinkBackend(mavlink_config)]

    spawn = [float(ARGS.x), float(ARGS.y), float(ARGS.z)]
    orientation = list(yaw_to_quat_xyzw(ARGS.yaw_deg))
    Multirotor(
        ARGS.prim_path,
        str(asset_path),
        0,
        spawn,
        orientation,
        config=multirotor_config,
    )

    await world.reset_async()
    omni.timeline.get_timeline_interface().play()
    pg.set_viewport_camera((spawn[0] - 2.0, spawn[1] - 2.0, spawn[2] + 1.2), spawn)
    carb.log_warn(
        "Loaded training drone "
        f"asset={asset_path} prim={ARGS.prim_path} spawn={spawn} "
        f"px4_autolaunch={not ARGS.no_px4_autolaunch}"
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
