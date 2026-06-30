#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${ROOT_DIR}/.." && pwd)"
PYAGXARM_DIR="${PROJECT_DIR}/pyAgxArm"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
CAN_ACTIVATE="${PYAGXARM_DIR}/pyAgxArm/scripts/ubuntu/can_activate.sh"
if [[ ! -x "${CAN_ACTIVATE}" ]]; then
  CAN_ACTIVATE="${PYAGXARM_DIR}/pyAgxArm/scripts/linux/can_activate.sh"
fi
CAN_BITRATE="${CAN_BITRATE:-1000000}"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误: 未找到可用 Python 环境: ${PYTHON_BIN}"
  echo "请先在项目根目录创建并安装依赖:"
  echo "  cd ${PROJECT_DIR}"
  echo "  python3 -m venv .venv"
  echo "  source .venv/bin/activate"
  echo "  pip install -r pyAgxArm/requirements-teleop.txt"
  echo "  pip install -e pyAgxArm"
  exit 1
fi

# 解析参数：收集 CAN 端口；支持 --no-can-activate 跳过自动激活
DO_CAN_ACTIVATE=1
CAN_PORTS=()
ARGS=()
prev=""
for arg in "$@"; do
  case "${prev}" in
    --left-can-port|--right-can-port)
      CAN_PORTS+=("${arg}")
      ;;
  esac
  case "${arg}" in
    --no-can-activate)
      DO_CAN_ACTIVATE=0
      prev="${arg}"
      continue
      ;;
    --left-can-port=*|--right-can-port=*)
      CAN_PORTS+=("${arg#*=}")
      ;;
  esac
  ARGS+=("${arg}")
  prev="${arg}"
done

# 自动激活 CAN 口（DOWN 时拉起，设置波特率）
if [[ "${DO_CAN_ACTIVATE}" -eq 1 && ${#CAN_PORTS[@]} -gt 0 ]]; then
  if [[ ! -x "${CAN_ACTIVATE}" ]]; then
    echo "[Launcher] 警告: 未找到 CAN 激活脚本 ${CAN_ACTIVATE}，跳过自动激活"
  else
    for port in "${CAN_PORTS[@]}"; do
      state="$(ip -br link show "${port}" 2>/dev/null | awk '{print $2}' || true)"
      if [[ "${state}" == "UP" ]]; then
        echo "[Launcher] CAN ${port} 已激活，跳过"
        continue
      fi
      echo "[Launcher] 激活 CAN ${port} (bitrate=${CAN_BITRATE}) ..."
      sudo bash "${CAN_ACTIVATE}" "${port}" "${CAN_BITRATE}"
    done
  fi
fi

cd "${ROOT_DIR}"
echo "[Launcher] 使用解释器: ${PYTHON_BIN}"
exec "${PYTHON_BIN}" "${ROOT_DIR}/scripts/teleop_piper_webxr.py" "${ARGS[@]}"
