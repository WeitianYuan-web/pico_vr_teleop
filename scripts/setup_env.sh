#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="${PROJECT_DIR}/.venv"
PYAGX="${PROJECT_DIR}/third_party/pyAgxArm"

echo "[Setup] 项目目录: ${PROJECT_DIR}"
python3 -m venv "${VENV}"
"${VENV}/bin/pip" install --upgrade pip
"${VENV}/bin/pip" install -r "${PYAGX}/requirements-teleop.txt"
"${VENV}/bin/pip" install -r "${PROJECT_DIR}/publisher/requirements.txt"
"${VENV}/bin/pip" install -e "${PYAGX}"

# 可选依赖提示（不阻断 setup）
echo ""
echo "[Setup] 基础环境完成（Piper SDK + publisher）。"
if [[ ! -d "${PROJECT_DIR}/third_party/InspireHandSDK_Y/src" && ! -d "${PROJECT_DIR}/third_party/InspireHandSDK_Y/include" ]]; then
  echo "[Setup] 提示: 未检测到 InspireHandSDK_Y 源码 → Piper 双手需拷贝到 third_party/InspireHandSDK_Y/"
fi
JAKA_SO_GLOB="${PROJECT_DIR}/backends/jaka/20260104145805A007"
if [[ ! -d "${JAKA_SO_GLOB}" ]]; then
  echo "[Setup] 提示: 未检测到 JAKA 厂商包 → 请放到 backends/jaka/20260104145805A007/（见 backends/jaka/README.md）"
fi
if ! "${VENV}/bin/python" -c "import unitree_sdk2py" 2>/dev/null; then
  echo "[Setup] 提示: 未检测到 unitree_sdk2py → G1 需另装 unitree_sdk2_python（见 backends/g1/README.md）"
fi

echo ""
echo "[Setup] 完整依赖说明: ${PROJECT_DIR}/DEPENDENCIES.md"
echo "推荐启动:"
echo "  source /opt/ros/jazzy/setup.bash   # 按你的 ROS 发行版调整"
echo "  source \"${VENV}/bin/activate\""
echo "  ./scripts/run_full_stack.sh --backend piper|jaka|g1"
