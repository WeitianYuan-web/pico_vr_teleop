#!/usr/bin/env bash
# 使用 uv 虚拟环境 + JAKA SDK 库路径运行命令
# 用法: ./run.sh python keyboard_teleop.py --no-power
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_LIB="$DIR/../20260104145805A007/SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu"

export LD_LIBRARY_PATH="${SDK_LIB}:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="${SDK_LIB}:${DIR}:${PYTHONPATH:-}"

if [[ ! -d "$DIR/.venv" ]]; then
  echo "创建 uv 虚拟环境 ..."
  (cd "$DIR" && uv venv)
fi

PYTHON="$DIR/.venv/bin/python"
if [[ "${1:-}" == "python3" ]]; then
  shift
fi
cd "$DIR"
exec "$PYTHON" "$@"
