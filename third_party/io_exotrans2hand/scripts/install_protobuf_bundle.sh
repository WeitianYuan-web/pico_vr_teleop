#!/usr/bin/env bash
# 编译 libprotobuf + protoc → bundle/opt/io-deps，并 pip 安装 Python protobuf
# 用法：在 Docker 构建容器内，已 export §3 环境变量后执行
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IO_EXOTRANS2HAND_ROOT="${IO_EXOTRANS2HAND_ROOT:-$ROOT}"

: "${PREFIX:?请先 export PREFIX（如 /ros_ws/bundle/opt/io-deps）}"
: "${SRC:?请先 export SRC（如 /ros_ws/tmp）}"
: "${PY_SITE:?请先 export PY_SITE}"
: "${PYTHON_EXECUTABLE:=/usr/bin/python3.10}"

export PROTOBUF_TAG="${PROTOBUF_TAG:-v28.3}"
export PROTOBUF_CPP_VERSION="${PROTOBUF_CPP_VERSION:-28.3}"
export PROTOBUF_PY_VERSION="${PROTOBUF_PY_VERSION:-5.28.3}"

if [[ ! -d "$SRC/protobuf" ]]; then
  git clone --depth 1 --branch "$PROTOBUF_TAG" --recurse-submodules \
    https://github.com/protocolbuffers/protobuf.git "$SRC/protobuf"
fi

cmake -S "$SRC/protobuf" -B "$SRC/protobuf/build" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$PREFIX" \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
  -Dprotobuf_BUILD_SHARED_LIBS=ON \
  -Dprotobuf_BUILD_TESTS=OFF \
  -Dprotobuf_BUILD_PROTOC_BINARIES=ON \
  -Dprotobuf_INSTALL=ON \
  -Dprotobuf_ABSL_PROVIDER=module

cmake --build "$SRC/protobuf/build" -j"$(nproc)"
cmake --install "$SRC/protobuf/build"

export PATH="$PREFIX/bin:$PATH"
protoc --version

"$PYTHON_EXECUTABLE" -m pip install --no-cache-dir --target="$PY_SITE" \
  "protobuf==${PROTOBUF_PY_VERSION}"

"$PYTHON_EXECUTABLE" -c "
import sys
sys.path.insert(0, '$PY_SITE')
import google.protobuf
print('protobuf py', google.protobuf.__version__)
"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$ROOT/scripts/gen_protobuf.sh"

echo "[install_protobuf] C++ libprotoc ${PROTOBUF_CPP_VERSION} + pip ${PROTOBUF_PY_VERSION} OK"
