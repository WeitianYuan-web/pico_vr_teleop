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
| `backends/jaka/sdk` 等源码 | 是 | 拉代码即可 |
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

## 快速自检

```bash
# Piper SDK
python -c "from pyAgxArm import AgxArmFactory; print('piper ok')"

# JAKA 厂商库路径（存在即路径对）
ls "backends/jaka/20260104145805A007/SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu/libjakaAPI.so"

# G1
python -c "from unitree_sdk2py.core.channel import ChannelFactoryInitialize; print('g1 sdk ok')"
```
