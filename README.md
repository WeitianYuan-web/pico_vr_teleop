# pico_vr_teleop

PICO WebXR 双臂遥操作项目：通过头显手柄控制 Piper 机械臂与 Inspire 灵巧手，支持 Placo QP 逆运动学、透视 AR 叠加、ROS2 状态发布。

## 功能概览

| 模块 | 说明 |
|------|------|
| WebXR 客户端 | PICO 浏览器访问，透视 AR + 坐标系叠加，手柄 6DoF 数据上传（额定 90 Hz） |
| WSS 服务 | `webxr_test/server.py`，HTTPS/WSS 转发 VR 数据 |
| 机械臂遥操作 | Placo QP IK，`move_j` 关节控制（额定 200 Hz） |
| 灵巧手 | Inspire RH56H1，扳机线性控制张合 |
| ROS 发布 | RealSense 图像 + `/puppet/*` 臂/手状态（UDP 桥接） |
| 键盘示例 | 单机 Piper 调试，含 MeshCat 可视化 |

## 目录结构

```
pico_vr_teleop/
├── control/                    # 双臂双手遥操作入口
│   └── dual_arm_dual_hand_webxr.py
├── webxr_test/                 # WebXR 网页、WSS 服务、VR 遥操作脚本
│   ├── index.html              # PICO 端页面（透视 AR）
│   ├── server.py
│   ├── scripts/teleop_piper_webxr.py
│   └── WEBXR_PIPER_TELEOP.md   # VR 遥操作详细说明
├── publisher/                  # ROS2 发布节点
│   ├── teleop_realsense_publisher.py
│   └── teleop_state_bridge.py  # UDP 状态桥接
├── pyAgxArm/                   # Piper SDK + Placo QP IK
├── InspireHandSDK_Y/           # Inspire 灵巧手 SDK（需编译 Python 绑定）
├── jaka_control/sdk/           # JAKA SDK 依赖与控制实现
│   ├── keyboard_teleop_dual.py
│   └── vr_teleop_dual.py
├── g1_control/                 # Unitree G1 SDK（DDS）双臂遥操作
│   ├── g1_arm_controller.py
│   ├── g1_arm_ik.py
│   └── vr_teleop_dual.py
├── vr_teleop/                  # 统一 VR 遥操作入口（按机型分发）
│   ├── piper_dual_webxr.py
│   ├── jaka_dual_webxr.py
│   └── g1_dual_webxr.py
├── scripts/                    # 启动脚本
│   ├── setup_env.sh
│   ├── run_full_stack.sh       # 一键启动（推荐）
│   ├── run_dual_arm_dual_hand.sh
│   ├── run_vr_teleop.sh
│   ├── run_vr_teleop_jaka_dual.sh
│   ├── run_vr_teleop_g1_dual.sh
│   └── run_keyboard_teleop.sh
└── logs/                       # 后台服务日志
```

## 一次性初始化

```bash
cd pico_vr_teleop
bash scripts/setup_env.sh
```

可选：编译 InspireHand Python 绑定（灵巧手控制需要）：

```bash
cd InspireHandSDK_Y
cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON
cmake --build build --target inspire_hand_py
```

WebXR 服务需要自签名证书（`webxr_test/cert.pem`、`key.pem`）。若缺失，可从旧项目目录复制或自行生成。

## 推荐：一键启动全栈

同时启动 WebXR 服务、遥操作、ROS 发布节点（可选 Piper/JAKA/G1）：

```bash
cd pico_vr_teleop
source /opt/ros/jazzy/setup.bash   # 按本机 ROS 发行版调整
source .venv/bin/activate
./scripts/run_full_stack.sh
```

常用选项：

```bash
# 选择后端：Piper（默认）/ JAKA / G1
./scripts/run_full_stack.sh --backend piper
./scripts/run_full_stack.sh --backend jaka
./scripts/run_full_stack.sh --backend g1 -- --motion --network-interface enp12s0

# 跳过 CAN 自动激活（已手动配置 can0/can1 时）
./scripts/run_full_stack.sh --no-can-activate

# 仅启动 VR + 遥操作，不启 ROS
./scripts/run_full_stack.sh --no-publisher

# 传递灵巧手串口等参数（-- 之后传给遥操作脚本）
./scripts/run_full_stack.sh -- --left-hand-port /dev/ttyUSB0 --right-hand-port /dev/ttyUSB1
```

说明：

- `--backend piper`：发布双臂 + 双手状态到 `/puppet/*`
- `--backend jaka`：发布双臂状态到 `/puppet/*`（手关节按无效/零值占位）
- `--backend g1`：Unitree G1 双臂 DDS 控制，状态同样上报 `/puppet/*`（手关节占位）

启动后：

1. 查看 `logs/vr_server.log` 中的 HTTPS 地址
2. PICO 浏览器访问该地址，点击「进入透视 AR」
3. 终端每 3 秒打印 `[频率监测]`，可确认 WebXR 上传与机械臂控制实际频率

`Ctrl+C` 退出时机械臂默认**不失能**（保持使能状态）。

## 分步启动

### 双臂 + 双手 VR 遥操作

```bash
# 终端 1：WebXR 服务
cd webxr_test && ../.venv/bin/python server.py

# 终端 2：遥操作
./scripts/run_dual_arm_dual_hand.sh
```

默认硬件映射（双臂模式）：

| 设备 | 左 | 右 |
|------|----|----|
| 机械臂 CAN | can0 | can1 |
| 灵巧手串口 | /dev/ttyUSB0 | /dev/ttyUSB1 |

### 单臂 VR 遥操作

```bash
./scripts/run_vr_teleop.sh --hands right --right-can-port can0
```

### JAKA 双臂 VR 遥操作（servo_p）

```bash
# 终端 1：WebXR 服务
cd webxr_test && ../.venv/bin/python server.py

# 终端 2：JAKA 双臂 VR（servo_p）
./scripts/run_vr_teleop_jaka_dual.sh --hands both
```

说明：实际入口统一放在 `vr_teleop/jaka_dual_webxr.py`，JAKA SDK 依赖与实现留在 `jaka_control/sdk/`，便于将“入口聚合”和“机型依赖”解耦。

常用参数：

```bash
./scripts/run_vr_teleop_jaka_dual.sh \
  --left-ip 192.168.10.21 \
  --right-ip 192.168.10.11 \
  --rotation-mode always \
  --speed-mm-s 160 \
  --speed-deg-s 40 \
  --no-shutdown
```

### Unitree G1 双臂 VR 遥操作（DDS + Pinocchio IK）

```bash
# 终端 1：WebXR 服务
cd webxr_test && ../.venv/bin/python server.py

# 终端 2：空跑验证（不连真机）
./scripts/run_vr_teleop_g1_dual.sh --dry-run --print-status

# 终端 2：真机（推荐 motion 模式）
./scripts/run_vr_teleop_g1_dual.sh --motion --network-interface enp12s0 --print-status
```

说明：入口为 `vr_teleop/g1_dual_webxr.py`，实现位于 `g1_control/`。需先安装 `unitree_sdk2_python`，详见 [g1_control/README.md](g1_control/README.md)。

### 键盘控制（调试）

```bash
./scripts/run_keyboard_teleop.sh --can_port can0
```

常用按键：方向键平移、`u/o/i/j/k/l` 旋转、`b` 回初始位姿、`q` 退出。

## VR 操作说明

| 输入 | 功能 |
|------|------|
| **Grip（握把）** | 按住接合：平移增量 + 姿态绝对跟随；松开断开 |
| **Trigger（扳机）** | 灵巧手张合（线性映射，见下表） |
| **B 键** | 对应臂回到初始关节/末端位姿 |

控制频率（额定值，实际以终端 `[频率监测]` 为准）：

- WebXR 数据上传：90 Hz
- 机械臂控制循环：200 Hz

初始关节角、TCP 偏移、姿态保护范围等可在 `webxr_test/scripts/teleop_piper_webxr.py` 顶部常量中修改，详见 [webxr_test/WEBXR_PIPER_TELEOP.md](webxr_test/WEBXR_PIPER_TELEOP.md)。

## 灵巧手扳机映射

`alpha` 语义：`0` = 全张，`1` = 全握（插值端点分别为 `default_open_pose` 与 `full_close_pose`）。

| 扳机 | 默认 alpha | 效果 |
|------|-----------|------|
| 松开 | 0.05 | 接近全张 |
| 按满 | 0.6 | 约六成闭合 |

可通过参数调整：

```bash
./scripts/run_dual_arm_dual_hand.sh \
  --hand-min-position 0.05 \
  --hand-max-position 0.6
```

灵巧手连接失败时默认跳过该侧并继续机械臂控制；加 `--strict-hand-connect` 可在连接失败时退出。

## ROS 话题

遥操作脚本通过 UDP `127.0.0.1:17981` 上报臂/手状态，由 `publisher/teleop_realsense_publisher.py` 发布：

- `/puppet/joint_left`、`/puppet/joint_right`
- `/puppet/end_pose_left`、`/puppet/end_pose_right`
- `/puppet/hand_left`、`/puppet/hand_right`
- `/camera_f/l/r/color/image_raw`（RealSense，可选）

详见 [publisher/README.md](publisher/README.md)。

## 常见问题

### CAN 口 `Device is DOWN`

启动脚本会自动激活 `can0`/`can1`（需 `sudo`）。手动激活：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

跳过自动激活：各启动脚本均支持 `--no-can-activate`。

### 双臂时右手无反应

确认右臂使用 `can1`（不要与左臂共用 `can0`）。双臂模式默认 `left=can0, right=can1`。

### 灵巧手默认姿态不对

- `hand_min_position` 控制扳机**松开**端（应接近 0，否则默认会偏握）
- `hand_max_position` 控制扳机**按满**端（当前默认 0.6，非全握）

### 运动抖动

查看终端 `[频率监测]` 实际频率。若远低于额定值，说明双臂 IK + CAN 耗时是瓶颈，可适当降低 `CMD_RATE_HZ` 或增大 `POS_SMOOTH_TAU_S` / `JOINT_SMOOTH_TAU_S`。

## 相关文档

- [webxr_test/WEBXR_PIPER_TELEOP.md](webxr_test/WEBXR_PIPER_TELEOP.md) — VR 遥操作架构、坐标系、参数表
- [publisher/README.md](publisher/README.md) — ROS 发布节点与 UDP 桥接
- [pyAgxArm/docs/piper_placo_teleop.md](pyAgxArm/docs/piper_placo_teleop.md) — Placo IK 与键盘遥操作
