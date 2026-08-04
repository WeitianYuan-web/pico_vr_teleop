#!/usr/bin/env bash
# 将 UDP 桥持久安装到天轶机器人并设为开机自启
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec python3 "${PROJECT_DIR}/backends/tianyee/scripts/install_tianyee_bridge_on_robot.py" "$@"
