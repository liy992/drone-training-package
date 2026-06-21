#!/usr/bin/env bash
set -euo pipefail

PACKAGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_BIN="${ISAACSIM_BIN:-/home/robot-a/miniconda3/envs/env_isaacsim/bin/isaacsim}"

if [[ ! -x "${ISAACSIM_BIN}" ]]; then
  echo "Isaac Sim executable not found: ${ISAACSIM_BIN}" >&2
  echo "Set ISAACSIM_BIN=/path/to/isaacsim and run again." >&2
  exit 1
fi

exec "${ISAACSIM_BIN}" --exec "${PACKAGE_ROOT}/isaac/load_cargo_bay.py" "$@"
