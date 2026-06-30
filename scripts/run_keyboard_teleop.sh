#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
CAN_ACTIVATE="${PROJECT_DIR}/pyAgxArm/pyAgxArm/scripts/ubuntu/can_activate.sh"
if [[ ! -x "${CAN_ACTIVATE}" ]]; then
  CAN_ACTIVATE="${PROJECT_DIR}/pyAgxArm/pyAgxArm/scripts/linux/can_activate.sh"
fi
CAN_BITRATE="${CAN_BITRATE:-1000000}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误: 未找到虚拟环境 ${PYTHON_BIN}"
  echo "请先执行: ${PROJECT_DIR}/scripts/setup_env.sh"
  exit 1
fi

DO_CAN_ACTIVATE=1
CAN_PORT=""
ARGS=()
prev=""
for arg in "$@"; do
  case "${arg}" in
    --no-can-activate)
      DO_CAN_ACTIVATE=0
      prev="${arg}"
      continue
      ;;
    --can_port=*|--can-port=*)
      CAN_PORT="${arg#*=}"
      ;;
  esac
  if [[ "${prev}" == "--can_port" || "${prev}" == "--can-port" ]]; then
    CAN_PORT="${arg}"
  fi
  ARGS+=("${arg}")
  prev="${arg}"
done

if [[ -z "${CAN_PORT}" ]]; then
  CAN_PORT="can0"
fi

if [[ "${DO_CAN_ACTIVATE}" -eq 1 ]]; then
  if [[ -x "${CAN_ACTIVATE}" ]]; then
    state="$(ip -br link show "${CAN_PORT}" 2>/dev/null | awk '{print $2}' || true)"
    if [[ "${state}" != "UP" ]]; then
      echo "[Launcher] 激活 CAN ${CAN_PORT} (bitrate=${CAN_BITRATE}) ..."
      sudo bash "${CAN_ACTIVATE}" "${CAN_PORT}" "${CAN_BITRATE}"
    else
      echo "[Launcher] CAN ${CAN_PORT} 已激活，跳过"
    fi
  else
    echo "[Launcher] 警告: 未找到 CAN 激活脚本 ${CAN_ACTIVATE}"
  fi
fi

echo "[Launcher] 使用解释器: ${PYTHON_BIN}"
exec "${PYTHON_BIN}" "${PROJECT_DIR}/pyAgxArm/run_piper_keyboard_teleop.py" "${ARGS[@]}"
