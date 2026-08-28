#!/usr/bin/env bash
# 启动本仓 third_party/io_exotrans2hand 的 IO Gesture gateway
# 选手型：--model rh5dg2 或 export IO_HAND_MODEL=rh5dg2（写入 gateway.yaml hand_choose）
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck disable=SC1091
source "${PROJECT_DIR}/scripts/io_hand_env.sh"
IO_ROOT="${PROJECT_DIR}/third_party/io_exotrans2hand"
GATEWAY="${IO_ROOT}/scripts/run_gateway.sh"

cli_model=""
passthrough=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --model)
      cli_model="$2"
      shift 2
      ;;
    --model=*)
      cli_model="${1#*=}"
      shift
      ;;
    *)
      passthrough+=("$1")
      shift
      ;;
  esac
done
if [[ -n "${cli_model}" ]]; then
  export IO_HAND_MODEL="$(io_hand_normalize_model "${cli_model}")"
fi
set -- "${passthrough[@]+"${passthrough[@]}"}"

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

HAND_NAME="$(io_hand_io_name)"
explicit_hand=0
if [[ -n "${IO_HAND_MODEL:-}" || -n "${IO_HANDS:-}" || -n "${cli_model}" ]]; then
  explicit_hand=1
fi
GW_YAML="${IO_ROOT}/configs/config/gateway.yaml"
if [[ "${explicit_hand}" -eq 1 && -f "${GW_YAML}" ]]; then
  HAND_NAME="${HAND_NAME}" GW_YAML="${GW_YAML}" python3 - <<'PY'
from pathlib import Path
import os
import re
path = Path(os.environ["GW_YAML"])
hand = os.environ["HAND_NAME"]
text = path.read_text(encoding="utf-8")
updated, n = re.subn(
    r"(hand_choose:\n)(?:  - .+\n)+",
    rf"\g<1>  - {hand}\n",
    text,
    count=1,
)
if n:
    if updated != text:
        path.write_text(updated, encoding="utf-8")
        print(f"[io_gateway] hand_choose -> {hand}")
    else:
        print(f"[io_gateway] hand_choose 已是 {hand}")
else:
    print(f"[io_gateway] 警告: 未找到 hand_choose，请手动把 gateway.yaml 设为 {hand}")
PY
fi

export IO_EXOTRANS2HAND_ROOT="${IO_ROOT}"
cd "${IO_ROOT}"
exec bash "${GATEWAY}" "$@"
