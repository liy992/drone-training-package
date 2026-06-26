#!/usr/bin/env python3
"""Stage 04: publish pointcloud, TF, and odometry for EGO-Planner."""

from pathlib import Path
import sys

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from stage_common import StageConfig, run_stage


if __name__ == "__main__":
    run_stage(
        StageConfig(
            stage_name="stage_04_publish_tf_odom",
            enable_lidar=True,
            enable_pointcloud=True,
            enable_state_bridge=True,
            default_no_px4_autolaunch=False,
        )
    )
