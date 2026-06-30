#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${PROJECT_DIR}/.venv"

echo "[Setup] 项目目录: ${PROJECT_DIR}"
python3 -m venv "${VENV}"
"${VENV}/bin/pip" install -r "${PROJECT_DIR}/pyAgxArm/requirements-teleop.txt"
"${VENV}/bin/pip" install -e "${PROJECT_DIR}/pyAgxArm"

echo "[Setup] 完成。使用方式:"
echo "  source \"${VENV}/bin/activate\""
