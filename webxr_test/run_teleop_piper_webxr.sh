#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${ROOT_DIR}/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
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
HANDS_MODE="both"
prev=""
for arg in "$@"; do
  case "${prev}" in
    --left-can-port|--right-can-port)
      CAN_PORTS+=("${arg}")
      ;;
    --hands)
      HANDS_MODE="${arg}"
      ;;
  esac
  case "${arg}" in
    --no-can-activate)
      DO_CAN_ACTIVATE=0
      prev="${arg}"
      continue
      ;;
    --hands=*)
      HANDS_MODE="${arg#*=}"
      ;;
    --left-can-port=*|--right-can-port=*)
      CAN_PORTS+=("${arg#*=}")
      ;;
  esac
  ARGS+=("${arg}")
  prev="${arg}"
done

# 未显式指定端口时，按控制模式补全默认 CAN 口并自动激活
if [[ ${#CAN_PORTS[@]} -eq 0 ]]; then
  case "${HANDS_MODE}" in
    both)
      CAN_PORTS=("can0" "can1")
      ;;
    left|right)
      CAN_PORTS=("can0")
      ;;
  esac
elif [[ "${HANDS_MODE}" == "both" ]]; then
  # 双臂模式：即使用户只传了一个端口，也尝试拉起 can0/can1
  for port in can0 can1; do
    found=0
    for existing in "${CAN_PORTS[@]}"; do
      if [[ "${existing}" == "${port}" ]]; then
        found=1
        break
      fi
    done
    if [[ "${found}" -eq 0 ]]; then
      CAN_PORTS+=("${port}")
    fi
  done
fi

# 自动激活 CAN 口（ip link down → 配置 bitrate → up）
if [[ "${DO_CAN_ACTIVATE}" -eq 1 && ${#CAN_PORTS[@]} -gt 0 ]]; then
  for port in "${CAN_PORTS[@]}"; do
    echo "[Launcher] 激活 CAN ${port} (bitrate=${CAN_BITRATE}) ..."
    sudo ip link set "${port}" down
    sudo ip link set "${port}" type can bitrate "${CAN_BITRATE}"
    sudo ip link set "${port}" up
  done
fi

cd "${ROOT_DIR}"
echo "[Launcher] 使用解释器: ${PYTHON_BIN}"
exec "${PYTHON_BIN}" "${ROOT_DIR}/scripts/teleop_piper_webxr.py" "${ARGS[@]}"
