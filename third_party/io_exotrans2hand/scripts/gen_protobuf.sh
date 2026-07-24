#!/usr/bin/env bash
# protoc 生成 io_msgs → src/io_bus_proto/generated/
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export IO_EXOTRANS2HAND_ROOT="${IO_EXOTRANS2HAND_ROOT:-$ROOT}"

PREFIX="${PREFIX:-$ROOT/bundle/opt/io-deps}"
PY_SITE="${PY_SITE:-$ROOT/bundle/python/lib/python3.10/site-packages}"
export PATH="$PREFIX/bin:$PATH"
export LD_LIBRARY_PATH="${PREFIX}/lib:${LD_LIBRARY_PATH:-}"

IO_BUS_GEN_CPP="$ROOT/src/io_bus_proto/generated/cpp"
IO_BUS_GEN_PY="$ROOT/src/io_bus_proto/generated/python"
PROTO_FILE="$ROOT/proto/io_msgs/messages.proto"

command -v protoc >/dev/null || { echo "未找到 protoc，请先 install_protobuf_bundle.sh"; exit 1; }
test -f "$PROTO_FILE" || { echo "缺少 $PROTO_FILE"; exit 1; }

mkdir -p "$IO_BUS_GEN_CPP/io_msgs" "$IO_BUS_GEN_PY/io_msgs"

protoc -I "$ROOT/proto" --cpp_out="$IO_BUS_GEN_CPP" "$PROTO_FILE"
protoc -I "$ROOT/proto" --python_out="$IO_BUS_GEN_PY" "$PROTO_FILE"

touch "$IO_BUS_GEN_PY/io_msgs/__init__.py"

test -f "$IO_BUS_GEN_CPP/io_msgs/messages.pb.h"
test -f "$IO_BUS_GEN_CPP/io_msgs/messages.pb.cc"
test -f "$IO_BUS_GEN_PY/io_msgs/messages_pb2.py"

echo "protoc 生成 C++/Python 完成"

# Python 验证需要 pip protobuf（在 PY_SITE，非系统 python）
PYTHON="${IO_PYTHON:-${PYTHON_EXECUTABLE:-python3}}"
if ! "$PYTHON" -c "import sys; sys.path.insert(0, '$PY_SITE'); import google.protobuf" 2>/dev/null; then
  echo "警告: PY_SITE 未安装 google.protobuf，跳过 round-trip 验证" >&2
  echo "  请先: pip install --target=\"$PY_SITE\" 'protobuf==5.28.3'" >&2
  echo "  或:   ./scripts/install_protobuf_bundle.sh" >&2
else
  "$PYTHON" - <<PY
import sys
sys.path.insert(0, "${PY_SITE}")
sys.path.insert(0, "${IO_BUS_GEN_PY}")
from io_msgs import messages_pb2

msg = messages_pb2.JointState(stamp_ns=1, names=["j1"], position=[0.1])
raw = msg.SerializeToString()
out = messages_pb2.JointState()
out.ParseFromString(raw)
assert out.stamp_ns == 1 and list(out.names) == ["j1"]
print("messages_pb2 round-trip OK")
PY
fi

echo "[gen_protobuf] OK"
echo "  C++:    $IO_BUS_GEN_CPP/io_msgs/"
echo "  Python: $IO_BUS_GEN_PY/io_msgs/"
echo "  验证:   ./scripts/verify_protobuf.sh"
