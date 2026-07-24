# third_party/

厂商 / 第三方 SDK。是否随 git 拉取见下表。

| 目录 | 是否随仓库 | 说明 |
|------|------------|------|
| `pyAgxArm/` | **是** | Piper CAN SDK；`setup_env.sh` 会 `pip install -e` |
| `InspireHandSDK_Y/` | **否**（体积大） | 灵巧手；需拷贝源码后本地编译绑定 |

## Piper（已随仓）

```bash
./scripts/setup_env.sh
# 等价于：
# pip install -r third_party/pyAgxArm/requirements-teleop.txt
# pip install -e third_party/pyAgxArm
```

## Inspire 灵巧手（可选，Piper 双手）

1. 将 SDK 放到 `third_party/InspireHandSDK_Y/`（可从已配好机器 `rsync`）
2. 编译 Python 绑定：

```bash
cd third_party/InspireHandSDK_Y
cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON
cmake --build build --target inspire_hand_py
```

`build/` 已在 `.gitignore` 中，勿提交。

## 其他后端（不在 third_party）

| 后端 | 依赖 | 是否随仓 |
|------|------|----------|
| JAKA | `backends/jaka/20260104145805A007/` 厂商包 | 否，见 [backends/jaka/README.md](../backends/jaka/README.md) |
| G1 | `unitree_sdk2_python` + CycloneDDS 0.10.x | 否，见 [backends/g1/README.md](../backends/g1/README.md) |
| IO Gesture | `third_party/io_exotrans2hand/`（含 bundle） | 否，见下 |

## IO Gesture（外骨骼 → 手指重定向）

整包放到 `third_party/io_exotrans2hand/`（与 JAKA 厂商包类似，本机同步、大二进制不入库）：

```bash
./scripts/sync_io_exotrans2hand.sh
# 或：IO_EXOTRANS2HAND_SRC=/path/to/io_exotrans2hand_project_* ./scripts/sync_io_exotrans2hand.sh
```

`bundle/` 已在 `.gitignore`。联调入口见 [controllers/io_hand/README.md](../controllers/io_hand/README.md)。

