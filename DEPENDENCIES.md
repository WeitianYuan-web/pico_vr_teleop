# 跨设备依赖说明

拉代码后按后端补齐依赖。业务源码在仓库内；厂商闭源包 / 大体量 SDK 需本机放置。

## 一键（Piper 基础）

```bash
./scripts/setup_env.sh
source .venv/bin/activate
```

这会安装：`third_party/pyAgxArm`、`publisher` 依赖。

## 按后端

### Piper

| 组件 | 随仓？ | 操作 |
|------|--------|------|
| `third_party/pyAgxArm` | 是 | `setup_env.sh` |
| Inspire 手 `third_party/InspireHandSDK_Y` | 否 | 拷贝后编译绑定，见 [third_party/README.md](third_party/README.md) |
| `webxr/cert.pem` `key.pem` | 视提交情况 | 缺失则自签或从旧机复制 |

### JAKA

| 组件 | 随仓？ | 操作 |
|------|--------|------|
| `backends/jaka/sdk` | 是 | 拉代码即可 |
| `backends/jaka/20260104145805A007` | 否 | 从已配好机器拷贝整目录，见 [backends/jaka/README.md](backends/jaka/README.md) |

### G1

| 组件 | 随仓？ | 操作 |
|------|--------|------|
| `backends/g1` | 是 | 拉代码即可 |
| `unitree_sdk2_python` | 否 | clone + `pip install -e`，并装 CycloneDDS，见 [backends/g1/README.md](backends/g1/README.md) |

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
# 按官方 README 准备 CycloneDDS 0.10.x → CYCLONEDDS_HOME
pip install -e ./unitree_sdk2_python
# 或：export UNITREE_SDK2_PYTHON=/path/to/unitree_sdk2_python
```

### Galbot（Galaxy G1，`--backend galbot`）

与上面的 Unitree G1 **不是**同一套。SDK 体积大，不入库。

| 组件 | 随仓？ | 操作 |
|------|--------|------|
| `third_party/GalbotSDK-V1.7.3` | 否 | GBS **1.15.x** 用 SDK **1.7.3** |
| `third_party/GalbotSDK-main` | 否 | GBS 1.17 才用 SDK 1.9.1 |
| `/opt/galbot-1.7.3` native 依赖 | 否 | `sudo ./install.sh --install-dir /opt/galbot-1.7.3 -y` |
| `/data/config/embosa_ip_config.json` | 否 | `sudo ./configure_embosa_ip.sh`（`cp -n` 不覆盖已有） |

本机机器人当前是 GBS 1.15.15，**不要用 1.9**（init 能过、关节/EE 会空）。WBC 流式末端从 SDK 1.8 才有。

```bash
# 默认 PC 192.168.1.99 ↔ XCU 192.168.1.66 / HPU 192.168.1.88
./scripts/run_full_stack.sh --backend galbot
```

详见 [backends/galbot/README.md](backends/galbot/README.md)。

### IO Gesture（外骨骼控手，与 VR 臂共存）

| 组件 | 随仓？ | 操作 |
|------|--------|------|
| `third_party/io_exotrans2hand/` | 否（含 ~740MB bundle） | `./scripts/sync_io_exotrans2hand.sh` |
| `third_party/InspireHandSDK_Y` | 否 | 编译 Python 绑定；手桥默认 **RH56F2** |
| `protobuf>=5.28`（Jazzy / Py3.12） | — | `.venv/bin/pip install 'protobuf>=5.28,<6'`（zenoh2ros 解码） |

联调步骤与排错：[controllers/io_hand/README.md](controllers/io_hand/README.md)、[third_party/README.md](third_party/README.md)。

统一手控（占串口 + `/puppet/hand_*`）：

```bash
./scripts/run_hand_controller.sh
ros2 topic hz /puppet/hand_right
```

自检（gateway + zenoh2ros 已起）：

```bash
ros2 topic hz /io_teleop/Inspire_RH56F2/joint_cmd_finger_right
ls third_party/InspireHandSDK_Y/build/python/inspire_hand_py*.so
```

## 快速自检

```bash
# Piper SDK
python -c "from pyAgxArm import AgxArmFactory; print('piper ok')"

# JAKA 厂商库路径（存在即路径对）
ls "backends/jaka/20260104145805A007/SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu/libjakaAPI.so"

# G1（Unitree）
python -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('g1 sdk ok')"

# Galbot（需先 source /opt/galbot/.../setup.sh）
python -c "from galbot_sdk.g1 import GalbotRobot; print('galbot sdk ok')"
```
