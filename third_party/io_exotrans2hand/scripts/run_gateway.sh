#!/usr/bin/env bash
# =============================================================================
# io_gateway 启动脚本
#
# 模式（默认 head）：
#   ./scripts/run_gateway.sh
#       → Web 控制台 + 就绪后自动打开浏览器
#   ./scripts/run_gateway.sh --headless
#       → 仅 REST / WebSocket / 编排，不挂载静态页、不打开浏览器
#
# 环境变量：
#   GATEWAY_PORT      覆盖监听端口（默认读 gateway.yaml 或 8080）
#   GATEWAY_NO_BROWSER=1  head 模式下也不打开浏览器
# =============================================================================
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IO_EXOTRANS2HAND_ROOT="${IO_EXOTRANS2HAND_ROOT:-$ROOT}"

# bundle 内 Python 3.12 / ROS / io-deps（不依赖系统 python3 版本与 .venv）
# shellcheck disable=SC1091
source "$ROOT/scripts/bundle-env.sh"

# bundle-env 已设置 IO_PYTHON 与 PATH；此处统一网关入口
PYTHON="${IO_PYTHON:-$(command -v python3)}"

mkdir -p "$ROOT/logs"

# --- 解析 head / headless ---
HEADLESS=0
PYTHON_ARGS=()
for arg in "$@"; do
  case "$arg" in
    --headless|-headless)
      HEADLESS=1
      ;;
    --no-browser)
      export GATEWAY_NO_BROWSER=1
      ;;
    -h|--help)
      echo "用法: $0 [--headless|-headless] [--no-browser] [其它参数传给 io_gateway.backend.main]"
      echo "  默认 head：启动 Web UI 并自动打开 http://127.0.0.1:<port>/"
      echo "  --headless：无 Web UI，适合 SSH / systemd / 仅用 API 与 WS"
      exit 0
      ;;
    *)
      PYTHON_ARGS+=("$arg")
      ;;
  esac
done

if [[ "$HEADLESS" -eq 1 ]]; then
  PYTHON_ARGS=(--headless "${PYTHON_ARGS[@]}")
fi

_gateway_port() {
  if [[ -n "${GATEWAY_PORT:-}" ]]; then
    echo "$GATEWAY_PORT"
    return
  fi
  "$PYTHON" - <<'PY' 2>/dev/null || echo 8080
from pathlib import Path
import yaml
root = Path(__import__("os").environ.get("IO_EXOTRANS2HAND_ROOT", ".")).resolve()
for p in (root / "configs/config/gateway.yaml", root / "configs/gateway.yaml"):
    if p.is_file():
        d = yaml.safe_load(p.read_text()) or {}
        print(int(d.get("listen_port", 8080)))
        break
else:
    print(8080)
PY
}

PORT="$(_gateway_port)"
BASE_URL="http://127.0.0.1:${PORT}"

wait_for_gateway() {
  local i
  for i in $(seq 1 60); do
    if curl -sf "${BASE_URL}/api/v1/status" >/dev/null 2>&1; then
      return 0
    fi
    if [[ "$HEADLESS" -eq 1 ]] && curl -sf "${BASE_URL}/" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done
  return 1
}

open_gateway_browser() {
  [[ -z "${GATEWAY_NO_BROWSER:-}" ]] || return 0
  local url="${BASE_URL}/"
  if command -v xdg-open >/dev/null 2>&1; then
    xdg-open "$url" >/dev/null 2>&1 &
  elif command -v sensible-browser >/dev/null 2>&1; then
    sensible-browser "$url" >/dev/null 2>&1 &
  elif command -v gnome-open >/dev/null 2>&1; then
    gnome-open "$url" >/dev/null 2>&1 &
  else
    echo "[run_gateway] 未找到 xdg-open，请手动打开: $url"
    return 1
  fi
}

cd "$ROOT"

# Cython 编译后 main 为 .so，无法用 python -m；改为 import 后调用 main()
_GATEWAY_PY=( -c 'import sys; from io_gateway.backend.main import main; main(sys.argv[1:] or None)' )

if [[ "$HEADLESS" -eq 1 ]]; then
  echo "[run_gateway] mode=headless  API=${BASE_URL}/api/v1  WS=ws://127.0.0.1:${PORT}/ws"
  exec "$PYTHON" "${_GATEWAY_PY[@]}" "${PYTHON_ARGS[@]}"
fi

echo "[run_gateway] mode=head  UI=${BASE_URL}/"
"$PYTHON" "${_GATEWAY_PY[@]}" "${PYTHON_ARGS[@]}" &
GWPID=$!
cleanup() {
  kill "$GWPID" 2>/dev/null || true
  wait "$GWPID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if wait_for_gateway; then
  open_gateway_browser || true
else
  echo "[run_gateway] 警告: 网关未在预期时间内就绪，请检查日志 logs/*/io_gateway.log" >&2
fi

wait "$GWPID"
