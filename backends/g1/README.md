# backends/g1

Unitree G1 双臂 WebXR 遥操作（DDS + Placo QP IK）。

## 布局

```
backends/g1/
├── README.md
├── requirements.txt
├── config.py                # WS / URDF / 关节与坐标系默认
├── vr_teleop_dual.py        # WebXR clutch 主循环
├── g1_arm_controller.py     # DDS 双臂 PD / Mock
├── g1_arm_ik.py             # Placo QP IK
├── g1_joints.py             # 关节索引
└── assets/
    └── g1_dual_arm.urdf     # IK 模型；Placo 临时 URDF 写 /tmp
```

## 架构

```
PICO WebXR → webxr/server.py (WSS)
           → entrypoints/g1_dual_webxr.py
           → backends/g1/vr_teleop_dual.py
                ├─ Grip clutch 增量位姿映射
                ├─ Placo QP IK（末端位姿 + 肘部外展正则 + 关节限位）
                └─ DDS 关节 PD（rt/arm_sdk 或 rt/lowcmd）
```

| 项 | 内容 |
|----|------|
| 求解器 | `placo.KinematicsSolver`（与 Piper 同类） |
| 末端任务 | 左右 `rubber_hand` 位姿 / 位置软约束 |
| 肘部 | joints 正则偏向外展 + 肘 link ±Y 外侧软约束 |
| 限位 | `enable_joint_limits` + `enable_velocity_limits` + 边界内缩 |
| 单臂 | 松开侧 `mask_dof` + 关闭软任务；滤波钉死 `hold_q` |

偏好姿态见 `g1_arm_ik.PREFERRED_JOINTS`。

## 依赖

```bash
cd pico_vr_teleop
source .venv/bin/activate

git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

需 CycloneDDS 0.10.x（见官方 README / `CYCLONEDDS_HOME`）。也可：

```bash
export UNITREE_SDK2_PYTHON=/path/to/unitree_sdk2_python
```

## 启动

```bash
./scripts/run_vr_teleop_g1_dual.sh --dry-run --print-status
./scripts/run_vr_teleop_g1_dual.sh --motion --network-interface enp12s0 --print-status
./scripts/run_full_stack.sh --backend g1 -- --motion --network-interface enp12s0
```

入口：`entrypoints/g1_dual_webxr.py` → `vr_teleop_dual.py`。

## 操作

| 操作 | 说明 |
|------|------|
| 按住 Grip | 接合该侧臂 |
| 松开 Grip | 断开，关节冻结保持 |
| B 键 | 双臂回偏好姿态 |
| A 键 | 仅 `--rotation-mode hold-a` 时启用姿态 |

## 真机注意

1. 机器人进入 **Regular 运控**
2. 本机网卡与机器人同网段（常见 `192.168.123.x`）
3. `--motion` 使用 `rt/arm_sdk`；退出时 weight→0
4. 无 `--motion` 时走 `rt/lowcmd`（Debug）

参考：[unitreerobotics/xr_teleoperate](https://github.com/unitreerobotics/xr_teleoperate)。
