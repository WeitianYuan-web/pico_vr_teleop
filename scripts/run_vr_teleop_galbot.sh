#!/usr/bin/env bash
# 本机：Galbot G1 VR 遥操作（Embosa；SDK 1.8+ WBC / SDK 1.7 Motion EE）
# 与 Unitree G1（--backend g1）不是同一套 SDK。
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_PYTHON="${PROJECT_DIR}/.venv/bin/python"
ENTRY="${PROJECT_DIR}/entrypoints/galbot_dual_webxr.py"

if [[ ! -x "${VENV_PYTHON}" ]]; then
  echo "[错误] 未找到 ${VENV_PYTHON}，请先 ./scripts/setup_env.sh"
  exit 1
fi

resolve_galbot_setup() {
  local plat="${GALBOT_PLATFORM:-linux-x86_64-gcc940}"
  if [[ -n "${GALBOT_SETUP:-}" && -f "${GALBOT_SETUP}" ]]; then
    echo "${GALBOT_SETUP}"
    return
  fi
  # GBS 1.15.x 必须用 SDK 1.7；优先独立安装目录，避免和 1.9 混在 /opt/galbot。
  local candidates=()
  if [[ -n "${GALBOT_HOME:-}" ]]; then
    candidates+=("${GALBOT_HOME}")
  fi
  candidates+=(
    "/opt/galbot-1.7.3"
    "${HOME}/.local/galbot-1.7.3"
    "${PROJECT_DIR}/third_party/GalbotSDK-V1.7.3"
    "/opt/galbot"
    "${PROJECT_DIR}/third_party/GalbotSDK-main"
  )
  local home
  for home in "${candidates[@]}"; do
    if [[ -f "${home}/galbot_sdk/${plat}/setup.sh" ]]; then
      echo "${home}/galbot_sdk/${plat}/setup.sh"
      return
    fi
  done
  echo "[错误] 未找到 Galbot setup.sh。GBS 1.15 请装 SDK 1.7.3：" >&2
  echo "  cd third_party/GalbotSDK-V1.7.3 && sudo ./install.sh --platform linux-x86_64-gcc940 --install-dir /opt/galbot-1.7.3 -y" >&2
  exit 1
}

GALBOT_SETUP_SH="$(resolve_galbot_setup)"

# Embosa 自带 FastDDS，不能套本机 Domain 42 isolation。
unset ROS_DOMAIN_ID || true
unset FASTRTPS_DEFAULT_PROFILES_FILE || true
unset FASTDDS_DEFAULT_PROFILES_FILE || true
unset RMW_IMPLEMENTATION || true
unset CYCLONEDDS_URI || true

set +u
# shellcheck disable=SC1090
source "${GALBOT_SETUP_SH}"
set -u

export PYTHONPATH="${PROJECT_DIR}:${PROJECT_DIR}/backends/galbot:${PYTHONPATH:-}"

echo "[Galbot] setup=${GALBOT_SETUP_SH}"
echo "[Galbot] LD_LIBRARY_PATH 前缀: ${LD_LIBRARY_PATH%%:*}"
exec "${VENV_PYTHON}" "${ENTRY}" "$@"
