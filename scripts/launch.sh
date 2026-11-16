#!/usr/bin/env bash
# Launch the UR3 + Robotiq 2F-85 scene in Isaac Sim or MuJoCo, from any directory.
#
#   scripts/launch.sh isaacsim [--paused] [--headless --steps N]
#   scripts/launch.sh mujoco   [--model FILE.xml] [--headless --steps N]
#
# Interpreters (override with env vars):
#   ISAACSIM_PYTHON  default ~/isaacsim/python.sh
#   MUJOCO_PYTHON    default ~/anaconda3/envs/mujoco/bin/python, else Isaac Sim's python (also has mujoco)
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAACSIM_PYTHON="${ISAACSIM_PYTHON:-$HOME/isaacsim/python.sh}"
if [[ -z "${MUJOCO_PYTHON:-}" ]]; then
  if [[ -x "$HOME/anaconda3/envs/mujoco/bin/python" ]]; then
    MUJOCO_PYTHON="$HOME/anaconda3/envs/mujoco/bin/python"
  else
    MUJOCO_PYTHON="$ISAACSIM_PYTHON"
  fi
fi

sim="${1:-}"
shift || true
case "$sim" in
  isaacsim|isaac) exec "$ISAACSIM_PYTHON" "$HERE/launch_isaacsim.py" "$@" ;;
  mujoco|mj)      exec "$MUJOCO_PYTHON" "$HERE/launch_mujoco.py" "$@" ;;
  *)
    echo "usage: $0 {isaacsim|mujoco} [args]   (add --help for the simulator's options)" >&2
    exit 1 ;;
esac
