# third_party/

厂商 / 第三方 SDK（与业务代码分离）。

| 目录 | 说明 |
|------|------|
| `pyAgxArm/` | Piper CAN SDK + Placo IK 辅助脚本 |
| `InspireHandSDK_Y/` | Inspire RH56H1 灵巧手 SDK |

安装：

```bash
./scripts/setup_env.sh
# 或手动：
pip install -e third_party/pyAgxArm

cd third_party/InspireHandSDK_Y
cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON
cmake --build build --target inspire_hand_py
```
