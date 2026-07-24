#!/usr/bin/env bash
# 将 IO Gesture (io_exotrans2hand) 整包同步到本仓库 third_party/io_exotrans2hand/
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${PROJECT_DIR}/third_party/io_exotrans2hand"
SRC="${IO_EXOTRANS2HAND_SRC:-/home/b0106/teleop/io_exotrans2hand_project_zenoh_22.04_x86_v2.0.2}"

usage() {
  cat <<EOF
用法: $(basename "$0") [--src DIR]

将 IO exotrans2hand 工程 rsync 到:
  ${DEST}

默认源: ${SRC}
可用环境变量 IO_EXOTRANS2HAND_SRC 覆盖。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --src) SRC="$2"; shift 2 ;;
    --src=*) SRC="${1#*=}"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数: $1"; usage; exit 1 ;;
  esac
done

if [[ ! -d "${SRC}" ]]; then
  echo "[错误] 源目录不存在: ${SRC}"
  echo "请设置 IO_EXOTRANS2HAND_SRC 或传入 --src"
  exit 1
fi

mkdir -p "${DEST}"
echo "[sync] ${SRC}/ -> ${DEST}/"
# 保留本仓为 Jazzy/Py3.12 加的纯 Python codec、zenoh2ros --hands 补丁，以及本地 .cache
rsync -a --delete \
  --exclude '.git/' \
  --exclude 'logs/' \
  --exclude '.cache/' \
  --exclude 'src/io_bus_proto/io_bus_codec.py' \
  --exclude 'tools/zenoh2ros_bridge.py' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  "${SRC}/" "${DEST}/"

echo "[sync] 完成。bundle 存在: $([[ -d "${DEST}/bundle" ]] && echo yes || echo NO)"
echo "[sync] 启动网关: ./scripts/run_io_gateway.sh"
