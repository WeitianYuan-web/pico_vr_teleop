#!/usr/bin/env bash
# Zenoh → ROS2：把 IO gateway 的 finger joint_cmd 转到 /io_teleop/... 话题
#
# ROS Jazzy 是 Python 3.12，不能用 bundle 的 Python 3.10（会触发
# rclpy._rclpy_pybind11 版本错乱）。此处按官方「方案 B」：系统/venv Python 3.12
# + 仅挂载 abi3 的 zenoh 包 + 纯 Python io_bus_codec。
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IO_ROOT="${PROJECT_DIR}/third_party/io_exotrans2hand"
BRIDGE="${IO_ROOT}/tools/zenoh2ros_bridge.py"
ZENOH_SRC="${IO_ROOT}/bundle/python/lib/python3.10/site-packages/zenoh"
ZENOH_LINK_DIR="${IO_ROOT}/.cache/zenoh_py312_path"

if [[ ! -f "${BRIDGE}" ]]; then
  echo "[错误] 未找到: ${BRIDGE}"
  echo "请先执行: ./scripts/sync_io_exotrans2hand.sh"
  exit 1
fi

if [[ ! -d "${ZENOH_SRC}" ]]; then
  echo "[错误] 缺少 zenoh 包: ${ZENOH_SRC}"
  echo "请确认 third_party/io_exotrans2hand/bundle 已同步完整"
  exit 1
fi

# 与 ROS Jazzy 匹配的 Python 3.12：优先项目 venv（protobuf>=5.28），
# 系统 dist-packages 的 protobuf 往往过旧，无法加载 io_msgs/messages_pb2。
if [[ -n "${IO_ZENOH2ROS_PYTHON:-}" ]]; then
  PYTHON="${IO_ZENOH2ROS_PYTHON}"
elif [[ -x "${PROJECT_DIR}/.venv/bin/python3" ]]; then
  PYTHON="${PROJECT_DIR}/.venv/bin/python3"
else
  PYTHON="$(command -v python3)"
fi

PY_VER="$("${PYTHON}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [[ "${PY_VER}" != "3.12" ]]; then
  echo "[警告] 当前 Python=${PYTHON} (${PY_VER})；Jazzy 通常需要 3.12"
fi

# protobuf：messages_pb2 需要 >=5.28（含 runtime_version）
if ! "${PYTHON}" -c 'from google.protobuf import runtime_version' >/dev/null 2>&1; then
  echo "[错误] ${PYTHON} 缺少 protobuf>=5.28。请执行:"
  echo "  ${PROJECT_DIR}/.venv/bin/pip install 'protobuf>=5.28,<6'"
  echo "  # 或: ${PYTHON} -m pip install --user --break-system-packages 'protobuf>=5.28,<6'"
  exit 1
fi

# 仅暴露 zenoh（abi3），勿把整个 bundle site-packages 塞给 3.12
mkdir -p "${ZENOH_LINK_DIR}"
ln -sfn "${ZENOH_SRC}" "${ZENOH_LINK_DIR}/zenoh"

export IO_EXOTRANS2HAND_ROOT="${IO_ROOT}"
cd "${IO_ROOT}"

# 清掉可能残留的 bundle PYTHONPATH，再 source ROS
unset PYTHONPATH
set +u
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
else
  echo "[错误] 未找到 /opt/ros/jazzy 或 /opt/ros/humble"
  exit 1
fi
set -u

export PYTHONPATH="${IO_ROOT}/src:${IO_ROOT}/src/io_bus_proto/generated/python:${PYTHONPATH:-}:${ZENOH_LINK_DIR}"
export LD_LIBRARY_PATH="${IO_ROOT}/bundle/opt/zenoh/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

echo "[zenoh2ros] ROOT=${IO_ROOT}"
echo "[zenoh2ros] PYTHON=${PYTHON} (${PY_VER})"
echo "[zenoh2ros] ROS_DISTRO=${ROS_DISTRO:-unknown}"

# 快速自检，失败信息更友好
"${PYTHON}" - <<'PY'
import rclpy  # noqa: F401
import zenoh  # noqa: F401
from io_bus_proto.io_bus_codec import proto_to_dict  # noqa: F401
print("[zenoh2ros] imports ok (rclpy, zenoh, io_bus_codec)")
PY

# 默认显式订阅 Inspire_RH56F2，避免扫描窗口未等到 tf_hand 时 hands=[]
HANDS_ARGS=()
if [[ $# -eq 0 ]]; then
  HANDS_ARGS=(--hands "${IO_HANDS:-Inspire_RH56F2}")
fi

exec "${PYTHON}" "${BRIDGE}" "${HANDS_ARGS[@]}" "$@"
