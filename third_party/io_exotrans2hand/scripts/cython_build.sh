#!/usr/bin/env bash
# =============================================================================
# Cython 编译 io_gateway 模块为 .so
#
# 用法：
#   ./scripts/cython_build.sh              # 编译，保留 .py
#   ./scripts/cython_build.sh --strip-py   # 编译后删除已编译的 .py
#
# 须在 Ubuntu 22.04 构建容器内执行，使用 bundle Python 3.10。
# =============================================================================
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IO_EXOTRANS2HAND_ROOT="${IO_EXOTRANS2HAND_ROOT:-$ROOT}"
STRIP_PY=0

for arg in "$@"; do
  case "$arg" in
    --strip-py) STRIP_PY=1 ;;
    -h|--help)
      echo "用法: $0 [--strip-py]"
      exit 0
      ;;
    *) echo "未知参数: $arg" >&2; exit 1 ;;
  esac
done

# shellcheck disable=SC1091
source "$ROOT/scripts/bundle-env.sh"

PYTHON="${IO_PYTHON:-$(command -v python3)}"

echo "[cython_build] PYTHON=$PYTHON"
"$PYTHON" --version

if ! dpkg -s python3.10-dev >/dev/null 2>&1; then
  echo "[cython_build] 安装 python3.10-dev …"
  export DEBIAN_FRONTEND=noninteractive
  apt-get update -qq
  apt-get install -y -qq python3.10-dev build-essential
fi

echo "[cython_build] 安装 Cython / setuptools …"
"$PYTHON" -m pip install -q "Cython>=3.0" setuptools wheel

cd "$ROOT"
echo "[cython_build] build_ext --inplace …"
"$PYTHON" setup_cython.py build_ext --inplace

echo "[cython_build] 清理中间文件 *.c …"
find "$ROOT/src/io_gateway" -name '*.c' -delete

if [[ "$STRIP_PY" -eq 1 ]]; then
  echo "[cython_build] 删除已编译的 .py …"
  while IFS= read -r rel; do
    [[ -z "$rel" ]] && continue
    rm -f "$ROOT/src/$rel"
  done <<'PYLIST'
io_gateway/backend/state.py
io_gateway/backend/runtime_mode.py
io_gateway/backend/network_udp.py
io_gateway/backend/config_loader.py
io_gateway/backend/glove_manager.py
io_gateway/backend/hand_upload.py
io_gateway/backend/main.py
io_gateway/backend/api/routes.py
io_gateway/backend/ws/ws_hub.py
io_gateway/backend/zenoh/bridge.py
io_gateway/backend/headless/report.py
io_gateway/backend/orchestrator/manager.py
io_gateway/backend/orchestrator/planner.py
io_gateway/backend/orchestrator/probe.py
io_gateway/backend/orchestrator/probe_udp.py
io_gateway/backend/orchestrator/probe_cycle.py
io_gateway/backend/orchestrator/supervisor.py
PYLIST
fi

echo "[cython_build] 验证 import …"
"$PYTHON" -c "
import io_gateway.backend.state as st
import io_gateway.backend.main as main
print('state ->', st.__file__)
print('main  ->', main.__file__)
"

echo "[cython_build] 完成"
