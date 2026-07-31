#!/usr/bin/env bash
# 在天轶机器人上启动 body_control + XARM（提供 set_arm_enable）
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "${PROJECT_DIR}/scripts/start_tianyee_robot_stack.py" "$@"
