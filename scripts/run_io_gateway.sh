#!/usr/bin/env bash
# 启动本仓 third_party/io_exotrans2hand 的 IO Gesture gateway
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IO_ROOT="${PROJECT_DIR}/third_party/io_exotrans2hand"
GATEWAY="${IO_ROOT}/scripts/run_gateway.sh"

if [[ ! -x "${GATEWAY}" && ! -f "${GATEWAY}" ]]; then
  echo "[错误] 未找到 IO gateway: ${GATEWAY}"
  echo "请先执行: ./scripts/sync_io_exotrans2hand.sh"
  exit 1
fi

if [[ ! -d "${IO_ROOT}/bundle" ]]; then
  echo "[错误] 缺少 ${IO_ROOT}/bundle/"
  echo "请先执行: ./scripts/sync_io_exotrans2hand.sh"
  exit 1
fi

export IO_EXOTRANS2HAND_ROOT="${IO_ROOT}"
cd "${IO_ROOT}"
exec bash "${GATEWAY}" "$@"
