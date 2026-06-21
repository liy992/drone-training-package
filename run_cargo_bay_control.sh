#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRAINING_PYTHON="${TRAINING_PYTHON:-/usr/bin/python3}"

source_setup() {
  local setup_file="$1"
  set +u
  # shellcheck disable=SC1090
  source "${setup_file}"
  set -u
}

if [[ -f /opt/ros/foxy/setup.bash ]]; then
  source_setup /opt/ros/foxy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  source_setup /opt/ros/humble/setup.bash
elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_setup /opt/ros/jazzy/setup.bash
fi

for ws in "${PX4_ROS_WS:-}" /root/px4_ros_ws /home/robot-a/ros2_ws /home/robot-a/px4_ros_ws; do
  if [[ -n "${ws}" && -f "${ws}/install/setup.bash" ]]; then
    source_setup "${ws}/install/setup.bash"
  fi
done

exec "${TRAINING_PYTHON}" "${PACKAGE_ROOT}/control/cargo_bay_control.py" "$@"
