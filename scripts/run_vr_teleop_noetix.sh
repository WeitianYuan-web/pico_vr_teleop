#!/usr/bin/env bash
# 本机：Noetix M1 VR 遥操作（CycloneDDS → 机器人笛卡尔接口）
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="${PROJECT_DIR}/.venv/bin/python"
ENTRY="${PROJECT_DIR}/entrypoints/noetix_dual_webxr.py"
CARTESIAN_WS="${PROJECT_DIR}/third_party/cartesian_min_ws"
CYCLONE_XML="${CARTESIAN_WS}/src/noetix_python_controller/config/cyclonedds.xml"

if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "[错误] 未找到 ${VENV_PYTHON}，请先 ./scripts/setup_env.sh"
  exit 1
fi
if [[ ! -f "${CARTESIAN_WS}/install/setup.bash" ]]; then
  echo "[错误] 未找到 ${CARTESIAN_WS}/install/setup.bash，请先 colcon build cartesian_min_ws"
  exit 1
fi

resolve_ros_setup() {
  if [[ -n "${ROS_SETUP:-}" && -f "${ROS_SETUP}" ]]; then
    echo "${ROS_SETUP}"
    return
  fi
  local distro
  for distro in jazzy humble iron rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      echo "/opt/ros/${distro}/setup.bash"
      return
    fi
  done
  echo "[错误] 未找到 /opt/ros/*/setup.bash" >&2
  exit 1
}

set +u
# shellcheck disable=SC1090
source "$(resolve_ros_setup)"
# shellcheck disable=SC1091
source "${CARTESIAN_WS}/install/setup.bash"
set -u

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
if [[ -f "${CYCLONE_XML}" ]]; then
  export CYCLONEDDS_URI="file://${CYCLONE_XML}"
fi
# Do not force Domain 42 onto the robot DDS link.
unset ROS_DOMAIN_ID || true
unset FASTRTPS_DEFAULT_PROFILES_FILE || true
unset FASTDDS_DEFAULT_PROFILES_FILE || true

export PYTHONPATH="${PROJECT_DIR}:${PROJECT_DIR}/backends/noetix:${PYTHONPATH:-}"

echo "[Noetix] RMW=${RMW_IMPLEMENTATION} CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
exec "${VENV_PYTHON}" "${ENTRY}" "$@"
