# G1 双臂 WebXR 遥操作

将 Unitree G1 SDK（`unitree_sdk2_python`）接入本仓库，复用现有 PICO WebXR → WSS 管线。

## 架构

```
PICO WebXR → webxr_test/server.py (WSS)
           → vr_teleop/g1_dual_webxr.py
           → g1_control/vr_teleop_dual.py
                ├─ Grip clutch 增量位姿映射
                ├─ Pinocchio CLIK（g1_dual_arm.urdf，14 DoF）
                └─ DDS 关节 PD（rt/arm_sdk 或 rt/lowcmd）
```

## 依赖安装

```bash
# 1. 本仓库 venv（已有 pinocchio / websockets）
cd pico_vr_teleop
source .venv/bin/activate

# 2. Unitree Python SDK（需 CycloneDDS 0.10.x）
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

若 `pip install` 报找不到 CycloneDDS，按官方 README 编译安装 `cyclonedds` 并设置 `CYCLONEDDS_HOME`。

也可将 SDK 路径导出后启动：

```bash
export UNITREE_SDK2_PYTHON=/path/to/unitree_sdk2_python
```

## 启动

终端 1：WebXR 服务

```bash
cd webxr_test && ../.venv/bin/python server.py
```

终端 2：G1 遥操作

```bash
# 空跑（不连真机，验证 WebXR + IK）
./scripts/run_vr_teleop_g1_dual.sh --dry-run --print-status

# 真机：运控 Regular 模式 + arm_sdk（推荐）
./scripts/run_vr_teleop_g1_dual.sh --motion --network-interface enp12s0 --print-status

# 仿真（DDS domain=1，配合 unitree_sim_isaaclab）
./scripts/run_vr_teleop_g1_dual.sh --sim --motion --print-status
```

一键全栈：

```bash
./scripts/run_full_stack.sh --backend g1 -- --motion --network-interface enp12s0
```

## 操作

| 操作 | 说明 |
|------|------|
| 按住 Grip | 接合该侧臂，手柄增量映射到末端 |
| 松开 Grip | 断开，臂保持当前关节 |
| B 键 | 双臂回零位 |
| A 键 | 仅在 `--rotation-mode hold-a` 时启用姿态跟踪 |

## 真机注意

1. 机器人进入 **Regular 运控**（常见：`L2+B` → `L2+UP` → `R1+X`，以官方遥控说明为准）
2. 本机网卡与机器人同网段（常见 `192.168.123.x`）
3. `--motion` 使用 `rt/arm_sdk`，并通过关节 29 的 weight 与运控混合；退出时自动 weight→0
4. 无 `--motion` 时走 `rt/lowcmd`（Debug），会锁非臂关节，需确认已关闭冲突的高层服务

## 文件

| 文件 | 说明 |
|------|------|
| `g1_arm_controller.py` | DDS 双臂 PD / Mock |
| `g1_arm_ik.py` | Pinocchio 双臂 CLIK |
| `vr_teleop_dual.py` | WebXR clutch 主循环 |
| `assets/g1_dual_arm.urdf` | 14 DoF 双臂模型 |

参考实现来源：[unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)。
