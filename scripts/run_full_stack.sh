#!/bin/bash
# 一键启动：WebXR 服务 + 双臂双手遥操作 + ROS 发布节点
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${PROJECT_DIR}/.venv"
VENV_PYTHON="${VENV_DIR}/bin/python"
VENV_ACTIVATE="${VENV_DIR}/bin/activate"
WEBXR_DIR="${PROJECT_DIR}/webxr"
LOG_DIR="${PROJECT_DIR}/logs"
CAN_BITRATE="${CAN_BITRATE:-1000000}"

DO_CAN_ACTIVATE=1
DO_VR_SERVER=1
DO_TELEOP=1
DO_PUBLISHER=1
TELEOP_ARGS=()
BACKEND="${TELEOP_BACKEND:-piper}"

# ROS 环境：优先 ROS_SETUP，其次自动探测常见发行版
ROS_SETUP="${ROS_SETUP:-}"
if [[ -z "${ROS_SETUP}" ]]; then
  for candidate in /opt/ros/jazzy/setup.bash /opt/ros/iron/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -f "${candidate}" ]]; then
      ROS_SETUP="${candidate}"
      break
    fi
  done
fi

usage() {
  cat <<EOF
用法: $(basename "$0") [选项] [-- 遥操作附加参数]

一键启动:
  1) WebXR HTTPS/WSS 服务 (server.py)
  2) 遥操作 (backend 可选 piper/jaka/g1)
  3) ROS 发布节点 (teleop_realsense_publisher.py)

选项:
  --no-can-activate     跳过 can0/can1 自动激活
  --backend <name>      选择遥操作后端: piper | jaka | g1（默认 piper）
  --no-vr-server        不启动 WebXR 服务
  --no-teleop           不启动遥操作
  --no-publisher        不启动 ROS 发布节点
  -h, --help            显示帮助

环境变量:
  TELEOP_BACKEND        遥操作后端（默认 piper）
  ROS_SETUP             ROS setup.bash 路径（默认自动探测）
  CAN_BITRATE           CAN 波特率（默认 1000000）
  ROS_ARGS              传给 publisher 的 ros-args 字符串

示例:
  $(basename "$0")
  $(basename "$0") --backend jaka
  $(basename "$0") --backend g1 -- --motion --network-interface enp12s0
  $(basename "$0") -- --left-hand-port /dev/ttyUSB0 --right-hand-port /dev/ttyUSB1
  ROS_ARGS="-p camera_f_serial:=xxxx" $(basename "$0")
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-can-activate)
      DO_CAN_ACTIVATE=0
      shift
      ;;
    --no-vr-server)
      DO_VR_SERVER=0
      shift
      ;;
    --backend)
      BACKEND="$2"
      shift 2
      ;;
    --backend=*)
      BACKEND="${1#*=}"
      shift
      ;;
    --no-teleop)
      DO_TELEOP=0
      shift
      ;;
    --no-publisher)
      DO_PUBLISHER=0
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      TELEOP_ARGS+=("$@")
      break
      ;;
    *)
      TELEOP_ARGS+=("$1")
      shift
      ;;
  esac
done

if [[ "${BACKEND}" != "piper" && "${BACKEND}" != "jaka" && "${BACKEND}" != "g1" ]]; then
  echo "[错误] --backend 仅支持 piper / jaka / g1，当前: ${BACKEND}"
  exit 1
fi

if [[ ! -x "${VENV_PYTHON}" || ! -f "${VENV_ACTIVATE}" ]]; then
  echo "[错误] 未找到虚拟环境: ${VENV_DIR}"
  echo "请先执行: cd ${PROJECT_DIR} && ./scripts/setup_env.sh"
  exit 1
fi

activate_runtime_env() {
  # ROS setup.bash 在 set -u 下可能引用未定义变量，需临时关闭
  set +u
  if [[ -n "${ROS_SETUP}" && -f "${ROS_SETUP}" ]]; then
    # shellcheck disable=SC1090
    source "${ROS_SETUP}"
  fi
  set -u
  # shellcheck disable=SC1091
  source "${VENV_ACTIVATE}"
}

mkdir -p "${LOG_DIR}"
PIDS=()
NAMES=()

cleanup() {
  local idx
  echo ""
  echo "[Launcher] 正在停止所有子进程 ..."
  for idx in "${!PIDS[@]}"; do
    local pid="${PIDS[$idx]}"
    local name="${NAMES[$idx]}"
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      echo "[Launcher] 已发送停止信号: ${name} (pid=${pid})"
    fi
  done
  wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

wait_for_port() {
  local port="$1"
  local timeout="${2:-20}"
  local elapsed=0
  while (( elapsed < timeout )); do
    if (echo >"/dev/tcp/127.0.0.1/${port}") >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
    elapsed=$((elapsed + 1))
  done
  return 1
}

activate_one_can_port() {
  local port="$1"
  if ! ip link show "${port}" &>/dev/null; then
    echo "[Launcher] 警告: 未找到接口 ${port}，跳过"
    return 0
  fi
  local state
  state="$(ip -br link show "${port}" 2>/dev/null | awk '{print $2}' || true)"
  if [[ "${state}" == "UP" ]]; then
    echo "[Launcher] CAN ${port} 已激活，跳过"
    return 0
  fi

  echo "[Launcher] 激活 CAN ${port} (bitrate=${CAN_BITRATE}) ..."
  sudo ip link set "${port}" down 2>/dev/null || true

  if ! sudo ip link set "${port}" type can bitrate "${CAN_BITRATE}"; then
    echo "[Launcher] 警告: ${port} 配置失败，继续启动（请手动检查 CAN）"
    return 0
  fi

  if ! sudo ip link set "${port}" up; then
    echo "[Launcher] 警告: ${port} 拉起失败，继续启动（请手动检查 CAN）"
    return 0
  fi
  echo "[Launcher] CAN ${port} 已 UP"
}

activate_can_ports() {
  if [[ "${BACKEND}" != "piper" ]]; then
    return 0
  fi
  if [[ "${DO_CAN_ACTIVATE}" -ne 1 || "${DO_TELEOP}" -ne 1 ]]; then
    return 0
  fi
  for port in can0 can1; do
    activate_one_can_port "${port}"
  done
}

start_bg() {
  local name="$1"
  shift
  echo "[Launcher] 启动 ${name}: $*"
  (
    activate_runtime_env
    cd "${PROJECT_DIR}"
    exec "$@"
  ) >>"${LOG_DIR}/${name}.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("${name}")
  echo "[Launcher] ${name} pid=${PIDS[-1]}  日志: ${LOG_DIR}/${name}.log"
}

start_publisher() {
  echo "[Launcher] 启动 publisher: teleop_realsense_publisher.py (ROS + venv)"
  (
    activate_runtime_env
    cd "${PROJECT_DIR}"
    if [[ -n "${ROS_ARGS:-}" ]]; then
      # shellcheck disable=SC2086
      exec python publisher/teleop_realsense_publisher.py --ros-args ${ROS_ARGS}
    else
      exec python publisher/teleop_realsense_publisher.py
    fi
  ) >>"${LOG_DIR}/publisher.log" 2>&1 &
  PIDS+=("$!")
  NAMES+=("publisher")
  echo "[Launcher] publisher pid=${PIDS[-1]}  日志: ${LOG_DIR}/publisher.log"
  sleep 1
  if ! kill -0 "${PIDS[-1]}" 2>/dev/null; then
    echo "[错误] publisher 启动后立即退出，请查看 ${LOG_DIR}/publisher.log"
    tail -n 20 "${LOG_DIR}/publisher.log" || true
    exit 1
  fi
}

echo "============================================================"
echo " pico_vr_teleop 一键启动 "
echo "============================================================"
echo "[Launcher] 项目目录: ${PROJECT_DIR}"
echo "[Launcher] 虚拟环境: ${VENV_DIR}"
echo "[Launcher] 运行时: source ROS + source .venv/bin/activate"

activate_can_ports

# 冷启动后 USB-CAN 与机械臂控制器需要短暂稳定时间
if [[ "${BACKEND}" == "piper" && "${DO_CAN_ACTIVATE}" -eq 1 && "${DO_TELEOP}" -eq 1 ]]; then
  echo "[Launcher] CAN 激活后等待 1.5s，确保总线/机械臂就绪 ..."
  sleep 1.5
fi

if [[ "${DO_VR_SERVER}" -eq 1 ]]; then
  if [[ ! -f "${WEBXR_DIR}/cert.pem" || ! -f "${WEBXR_DIR}/key.pem" ]]; then
    echo "[警告] 未找到 ${WEBXR_DIR}/cert.pem 或 key.pem，WebXR 服务可能启动失败。"
  fi
  start_bg "vr_server" "${VENV_PYTHON}" "${WEBXR_DIR}/server.py"
  if wait_for_port 8000 25; then
    echo "[Launcher] WebXR HTTPS 端口 8000 已就绪"
  else
    echo "[警告] HTTPS 8000 未在预期时间内就绪，请查看 ${LOG_DIR}/vr_server.log"
  fi
  if wait_for_port 8081 25; then
    echo "[Launcher] WebXR WSS 端口 8081 已就绪"
  else
    echo "[警告] WSS 8081 未在预期时间内就绪，请查看 ${LOG_DIR}/vr_server.log"
  fi
fi

if [[ "${DO_PUBLISHER}" -eq 1 ]]; then
  if [[ -z "${ROS_SETUP}" || ! -f "${ROS_SETUP}" ]]; then
    echo "[警告] 跳过 ROS 发布节点：未找到 ROS 环境 (ROS_SETUP)"
    DO_PUBLISHER=0
  else
    echo "[Launcher] ROS 环境: ${ROS_SETUP}"
    start_publisher
  fi
fi

if [[ "${DO_TELEOP}" -eq 1 ]]; then
  if [[ "${BACKEND}" == "jaka" ]]; then
    TELEOP_ENTRY="${PROJECT_DIR}/entrypoints/jaka_dual_webxr.py"
  elif [[ "${BACKEND}" == "g1" ]]; then
    TELEOP_ENTRY="${PROJECT_DIR}/entrypoints/g1_dual_webxr.py"
  else
    TELEOP_ENTRY="${PROJECT_DIR}/entrypoints/piper_dual_webxr.py"
  fi
  echo "[Launcher] 遥操作后端: ${BACKEND}"
  echo "[Launcher] 启动遥操作（前台）: ${TELEOP_ENTRY} ${TELEOP_ARGS[*]:-}"
  echo "[Launcher] 状态默认上报 UDP 127.0.0.1:17981 -> publisher"
  echo "[Launcher] PICO 访问页面见 ${LOG_DIR}/vr_server.log 中的 HTTPS 地址"
  echo "------------------------------------------------------------"
  (
    activate_runtime_env
    cd "${PROJECT_DIR}"
    if [[ "${BACKEND}" == "jaka" ]]; then
      JAKA_SDK_DIR="${PROJECT_DIR}/backends/jaka/20260104145805A007/SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu"
      if [[ -d "${JAKA_SDK_DIR}" ]]; then
        export LD_LIBRARY_PATH="${JAKA_SDK_DIR}:${LD_LIBRARY_PATH:-}"
      else
        echo "[Launcher] 警告: 未找到 JAKA SDK 目录: ${JAKA_SDK_DIR}"
      fi
    elif [[ "${BACKEND}" == "g1" ]]; then
      if [[ -n "${UNITREE_SDK2_PYTHON:-}" && -d "${UNITREE_SDK2_PYTHON}" ]]; then
        export PYTHONPATH="${UNITREE_SDK2_PYTHON}:${PYTHONPATH:-}"
      fi
    fi
    exec python "${TELEOP_ENTRY}" "${TELEOP_ARGS[@]}"
  )
else
  echo "[Launcher] 后台服务已启动。按 Ctrl+C 退出并停止所有进程。"
  echo "[Launcher] 日志目录: ${LOG_DIR}"
  wait
fi
