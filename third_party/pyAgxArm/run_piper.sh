#!/bin/bash
# Piper 机械臂启动脚本：激活虚拟环境、配置 CAN、运行监控或控制程序
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CAN_PORT="${CAN_PORT:-can0}"
BITRATE="${BITRATE:-1000000}"

if [[ ! -d ".venv" ]]; then
    echo "错误: 未找到虚拟环境 .venv，请先执行:"
    echo "  python3 -m venv .venv && source .venv/bin/activate && pip install python-can && pip install -e ."
    exit 1
fi

# shellcheck disable=SC1091
source .venv/bin/activate

activate_can() {
    if ip link show "$CAN_PORT" &>/dev/null; then
        if ip -br link show "$CAN_PORT" | grep -q "UP"; then
            echo "CAN 接口 $CAN_PORT 已存在且已 UP"
            return 0
        fi
        echo "CAN 接口 $CAN_PORT 存在但未 UP，正在激活..."
        sudo ip link set "$CAN_PORT" up type can bitrate "$BITRATE"
        return 0
    fi

    echo "未检测到 $CAN_PORT，尝试使用官方脚本查找并激活 USB-CAN 模块..."
    sudo bash pyAgxArm/scripts/ubuntu/can_activate.sh "$CAN_PORT" "$BITRATE"
}

usage() {
    cat <<EOF
用法: $0 <monitor|control|teleop|demo> [额外参数...]

  monitor   实时监控 Piper 状态（默认）
  control   使能机械臂并执行安全回零动作
  teleop    键盘末端位姿增量遥操作（Placo QP IK）
  demo      运行官方 demo (pyAgxArm/demos/piper/test1.py)

环境变量:
  CAN_PORT  CAN 通道名，默认 can0
  BITRATE   波特率，默认 1000000

示例:
  $0 monitor
  $0 control
  CAN_PORT=can1 $0 monitor
EOF
}

MODE="${1:-monitor}"
shift || true

activate_can

case "$MODE" in
    monitor)
        exec python pyAgxArm/demos/detect_piper_series.py --can_port "$CAN_PORT" "$@"
        ;;
    control)
        exec python run_piper_control.py --can_port "$CAN_PORT" "$@"
        ;;
    teleop|keyboard)
        exec python run_piper_keyboard_teleop.py --can_port "$CAN_PORT" "$@"
        ;;
    demo)
        exec python pyAgxArm/demos/piper/test1.py "$@"
        ;;
    -h|--help|help)
        usage
        ;;
    *)
        echo "未知模式: $MODE"
        usage
        exit 1
        ;;
esac
