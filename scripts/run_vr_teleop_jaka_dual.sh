#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PROJECT_DIR}/.venv/bin/python"
JAKA_SDK_DIR="${PROJECT_DIR}/jaka_control/20260104145805A007/SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu"

if [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "错误: 未找到虚拟环境 ${PYTHON_BIN}"
  echo "请先执行: ${PROJECT_DIR}/scripts/setup_env.sh"
  exit 1
fi

# 显式导出 SDK 动态库目录，避免 import jkrc 时找不到 libjakaAPI.so
if [[ -d "${JAKA_SDK_DIR}" ]]; then
  export LD_LIBRARY_PATH="${JAKA_SDK_DIR}:${LD_LIBRARY_PATH:-}"
else
  echo "[Launcher] 警告: 未找到 JAKA SDK 目录: ${JAKA_SDK_DIR}"
fi

echo "[Launcher] 使用解释器: ${PYTHON_BIN}"
exec "${PYTHON_BIN}" "${PROJECT_DIR}/vr_teleop/jaka_dual_webxr.py" "$@"
