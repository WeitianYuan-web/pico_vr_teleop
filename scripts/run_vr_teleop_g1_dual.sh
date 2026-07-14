#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误: 未找到虚拟环境 ${PYTHON_BIN}"
  echo "请先执行: ${PROJECT_DIR}/scripts/setup_env.sh"
  exit 1
fi

# 可选：若已安装 unitree_sdk2_python 到自定义路径，加入 PYTHONPATH
if [[ -n "${UNITREE_SDK2_PYTHON:-}" && -d "${UNITREE_SDK2_PYTHON}" ]]; then
  export PYTHONPATH="${UNITREE_SDK2_PYTHON}:${PYTHONPATH:-}"
fi

echo "[Launcher] 使用解释器: ${PYTHON_BIN}"
echo "[Launcher] G1 双臂 VR 遥操作（Grip 接合，B 回零）"
exec "${PYTHON_BIN}" "${PROJECT_DIR}/entrypoints/g1_dual_webxr.py" "$@"
