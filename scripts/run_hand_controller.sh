#!/usr/bin/env bash
# 统一 Inspire 手控：订阅 /hand_cmd 与 /io_teleop → 串口；发布 /puppet/hand_*
# 手型：--model rh5dg2 或 export IO_HAND_MODEL=rh5dg2（默认 rh56f2）
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${PROJECT_DIR}/scripts/io_hand_env.sh"
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

# 与一键启动一致：本机默认 Domain 42
export ROS_DOMAIN_ID="${LOCAL_ROS_DOMAIN_ID:-${ROS_DOMAIN_ID:-42}}"

if [[ -f "${VENV_ACTIVATE}" ]]; then
  # shellcheck disable=SC1091
  source "${VENV_ACTIVATE}"
fi

cd "${PROJECT_DIR}"

args=()
cli_model=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model|-model)
      cli_model="$2"
      shift 2
      ;;
    --model=*|-model=*)
      cli_model="${1#*=}"
      shift
      ;;
    *)
      args+=("$1")
      shift
      ;;
  esac
done

if [[ -n "${cli_model}" ]]; then
  export IO_HAND_MODEL="$(io_hand_normalize_model "${cli_model}")"
fi
resolved_model="$(io_hand_resolve_model)"
export IO_HAND_MODEL="${resolved_model}"
echo "[hand_controller] model=${resolved_model} io=$(io_hand_io_name "${resolved_model}")"

has_hand_model_param=0
for a in "${args[@]+"${args[@]}"}"; do
  if [[ "$a" == hand_model:=* || "$a" == -p=*hand_model:=* || "$a" == --param=*hand_model:=* ]]; then
    has_hand_model_param=1
  fi
done
if [[ "${has_hand_model_param}" -eq 0 ]]; then
  args=(-p "hand_model:=${resolved_model}" "${args[@]+"${args[@]}"}")
fi

# -p / --param 必须挂在 --ros-args 下，否则会被当成 remap（参数不生效）
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
