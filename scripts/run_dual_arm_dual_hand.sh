#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
CAN_BITRATE="${CAN_BITRATE:-1000000}"
DO_CAN_ACTIVATE=1
ARGS=()
prev=""

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误: 未找到可用 Python 环境: ${PYTHON_BIN}"
  echo "请先在项目根目录创建并安装依赖:"
  echo "  cd ${PROJECT_DIR}"
  echo "  ./scripts/setup_env.sh"
  exit 1
fi

for arg in "$@"; do
  case "${arg}" in
    --no-can-activate)
      DO_CAN_ACTIVATE=0
      prev="${arg}"
      continue
      ;;
  esac
  ARGS+=("${arg}")
  prev="${arg}"
done

if [[ "${DO_CAN_ACTIVATE}" -eq 1 ]]; then
  for port in can0 can1; do
    state="$(ip -br link show "${port}" 2>/dev/null | awk '{print $2}' || true)"
    if [[ "${state}" != "UP" ]]; then
      echo "[Launcher] 激活 CAN ${port} (bitrate=${CAN_BITRATE}) ..."
      sudo ip link set "${port}" down
      sudo ip link set "${port}" type can bitrate "${CAN_BITRATE}"
      sudo ip link set "${port}" up
    else
      echo "[Launcher] CAN ${port} 已激活，跳过"
    fi
  done
  echo "[Launcher] CAN 激活后等待 1.5s，确保总线/机械臂就绪 ..."
  sleep 1.5
fi

echo "[Launcher] 使用解释器: ${PYTHON_BIN}"
exec "${PYTHON_BIN}" "${PROJECT_DIR}/control/dual_arm_dual_hand_webxr.py" "${ARGS[@]}"
