#!/usr/bin/env bash
# 在天轶机器人上启动 UDP→ROS 末端桥（Humble + XARM）
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "${PROJECT_DIR}/backends/tianyee/scripts/start_tianyee_udp_bridge.py" "$@"
