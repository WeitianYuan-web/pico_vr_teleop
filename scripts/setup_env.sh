#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${PROJECT_DIR}/.venv"
PYAGX="${PROJECT_DIR}/third_party/pyAgxArm"

echo "[Setup] 项目目录: ${PROJECT_DIR}"
python3 -m venv "${VENV}"
"${VENV}/bin/pip" install --upgrade pip
"${VENV}/bin/pip" install -r "${PYAGX}/requirements-teleop.txt"
"${VENV}/bin/pip" install -r "${PROJECT_DIR}/publisher/requirements.txt"
"${VENV}/bin/pip" install -e "${PYAGX}"

echo "[Setup] 完成。推荐启动方式:"
echo "  source /opt/ros/jazzy/setup.bash   # 按你的 ROS 发行版调整"
echo "  source \"${VENV}/bin/activate\""
echo "  ./scripts/run_full_stack.sh"
