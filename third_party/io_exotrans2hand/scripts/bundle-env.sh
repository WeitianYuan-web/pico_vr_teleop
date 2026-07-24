#!/usr/bin/env bash
# 从 configs/config/gateway.yaml 的 bundle 段加载环境变量并 source ROS / workspace。
set -eo pipefail

_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IO_EXOTRANS2HAND_ROOT="${IO_EXOTRANS2HAND_ROOT:-$_ROOT}"

_read_bundle_yaml_path() {
  # 从 gateway.yaml bundle 段读取 python / python_site 等路径并展开 {root}
  local key="$1" default="${2:-}"
  local gw="$_ROOT/configs/config/gateway.yaml"
  local line path
  if [[ ! -f "$gw" ]]; then
    [[ -n "$default" ]] && printf '%s' "$default"
    return 0
  fi
  line="$(grep -E "^[[:space:]]+${key}:" "$gw" | head -1 || true)"
  if [[ -z "$line" ]]; then
    [[ -n "$default" ]] && printf '%s' "$default"
    return 0
  fi
  path="$(sed -n "s/^[[:space:]]*${key}:[[:space:]]*['\"]\\(.*\\)['\"].*/\\1/p" <<<"$line")"
  path="${path//\{root\}/$_ROOT}"
  printf '%s' "$path"
}

_bootstrap_pythonpath() {
  local site
  site="$(_read_bundle_yaml_path python_site "${_ROOT}/bundle/python/lib/python3.10/site-packages")"
  printf '%s:%s' "$_ROOT/src" "$site"
}

# 引导 import io_gateway；完整 PYTHONPATH / LD_LIBRARY_PATH / PATH 由 emit_bundle_env_shell 输出
_BOOTSTRAP_PYPATH="$(_bootstrap_pythonpath)"
export PYTHONPATH="${_BOOTSTRAP_PYPATH}${PYTHONPATH:+:$PYTHONPATH}"

_can_run_emit() {
  IO_EXOTRANS2HAND_ROOT="$_ROOT" PYTHONPATH="$_BOOTSTRAP_PYPATH" "$1" - <<'PY' >/dev/null 2>&1
import yaml  # noqa: F401
from io_gateway.backend.config_loader import emit_bundle_env_shell  # noqa: F401
PY
}

_pick_python() {
  local py sys
  py="$(_read_bundle_yaml_path python)"
  if [[ -n "$py" && -x "$py" ]] && _can_run_emit "$py"; then
    printf '%s' "$py"
    return 0
  fi
  sys="$(command -v python3 2>/dev/null || true)"
  if [[ -n "$sys" && -x "$sys" ]] && _can_run_emit "$sys"; then
    printf '%s' "$sys"
    return 0
  fi
  return 1
}

_IO_PY="$(_pick_python || true)"
if [[ -z "$_IO_PY" ]]; then
  echo "bundle-env: 未找到可用 Python（需 bundle python + PyYAML + io_gateway，见 gateway.yaml bundle.python）" >&2
  exit 1
fi

# ROS setup.bash 在 set -u 下会触发未定义变量，source 前临时关闭
set +u
eval "$(IO_EXOTRANS2HAND_ROOT="$IO_EXOTRANS2HAND_ROOT" PYTHONPATH="$_BOOTSTRAP_PYPATH" "$_IO_PY" - <<'PY'
from io_gateway.backend.config_loader import emit_bundle_env_shell
print(emit_bundle_env_shell())
PY
)"
set -u
