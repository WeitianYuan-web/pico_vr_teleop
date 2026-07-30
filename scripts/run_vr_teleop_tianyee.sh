#!/usr/bin/env bash
# 本机：天轶 VR 遥操作（默认 UDP → 机器人 bridge）
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="${PROJECT_DIR}/.venv/bin/python"
ENTRY="${PROJECT_DIR}/entrypoints/tianyee_dual_webxr.py"

if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "[错误] 未找到 ${VENV_PYTHON}，请先 ./scripts/setup_env.sh"
  exit 1
fi

export PYTHONPATH="${PROJECT_DIR}:${PROJECT_DIR}/backends/tianyee:${PYTHONPATH:-}"
exec "${VENV_PYTHON}" "${ENTRY}" --transport udp "$@"
