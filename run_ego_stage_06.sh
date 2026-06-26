#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_setup /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/foxy/setup.bash ]]; then
  source_setup /opt/ros/foxy/setup.bash
fi

if [[ ! -f /home/robot-a/ros2_ws/install/setup.bash ]]; then
  echo "Missing required base ROS 2 overlay: /home/robot-a/ros2_ws/install/setup.bash" >&2
  exit 1
fi

source_setup /home/robot-a/ros2_ws/install/setup.bash

if [[ -f "${PACKAGE_ROOT}/ros2_ws/install/setup.bash" ]]; then
  source_setup "${PACKAGE_ROOT}/ros2_ws/install/setup.bash"
else
  echo "Training overlay not built yet. Run:" >&2
  echo "  cd ${PACKAGE_ROOT}/ros2_ws && colcon build --packages-select ego_training_demo" >&2
  exit 1
fi

exec ros2 launch ego_training_demo stage_06_ego_px4.launch.py "$@"
