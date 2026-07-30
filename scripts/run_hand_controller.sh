#!/usr/bin/env bash
# 统一 RH56F2 手控：订阅 /hand_cmd 与 /io_teleop → 串口；发布 /puppet/hand_*
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_ACTIVATE="${PROJECT_DIR}/.venv/bin/activate"
CTRL_PY="${PROJECT_DIR}/controllers/io_hand/rh56f2_controller.py"

if [[ ! -f "${CTRL_PY}" ]]; then
  echo "[错误] 未找到 ${CTRL_PY}"
  exit 1
fi

set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[错误] 未找到 ROS2 setup.bash"
  exit 1
fi
set -u

if [[ -f "${VENV_ACTIVATE}" ]]; then
  # shellcheck disable=SC1091
  source "${VENV_ACTIVATE}"
fi

cd "${PROJECT_DIR}"

# -p / --param 必须挂在 --ros-args 下，否则会被当成 remap（参数不生效）
args=("$@")
if [[ ${#args[@]} -gt 0 ]]; then
  has_ros_args=0
  needs_ros_args=0
  for a in "${args[@]}"; do
    if [[ "$a" == "--ros-args" ]]; then
      has_ros_args=1
    fi
    if [[ "$a" == "-p" || "$a" == "--param" || "$a" == --param=* || "$a" == -p=* || "$a" == "--params-file" ]]; then
      needs_ros_args=1
    fi
  done
  if [[ "${needs_ros_args}" -eq 1 && "${has_ros_args}" -eq 0 ]]; then
    exec python3 "${CTRL_PY}" --ros-args "${args[@]}"
  fi
fi

exec python3 "${CTRL_PY}" "${args[@]}"
