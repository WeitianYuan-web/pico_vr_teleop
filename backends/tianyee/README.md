# backends/tianyee

天轶 2.0 Pro 双臂 WebXR 遥操作（末端位姿 → XARM `/endposetarget_*`）。

## 架构（推荐）

本机 Jazzy 与机器人 Humble DDS 不稳定，默认走 **UDP 桥**：

```text
PICO → webxr/server.py (WSS)
     → entrypoints/tianyee_dual_webxr.py  (本机 clutch 映射)
     → UDP :19011
     → backends/tianyee/udp_ros_bridge.py (机器人 Humble)
     → /endposetarget_L|R → endpose QP 控制器
```

| 操作 | 说明 |
|------|------|
| Grip | 接合该侧，增量跟随手柄（滞回 + 死区，原地反复按不应乱飘） |
| 松开 Grip | 冻结目标约 0.45s 后停发，末端应较快稳住 |
| A（按住） | 默认 `hold-a`：仅此时跟随旋转；Grip 只平移 |
| B | 回到启动时的抬手位 |

稳定性相关默认在 `config.py`（`--pos-deadzone-m`、`--max-cmd-step-m`、`--release-freeze-s`、`--rotation-mode` 等可覆盖）。再次 Grip 时用本地 hold 作基准，不再每次读滞后 TF。

### TCP 坐标系

整机：`+X` 前、`+Y` 左、`+Z` 上。控制帧：`waist_yaw_link` → `left/right_tcp_link`。

URDF 中 `*_tcp_link` 固连于 `wrist_roll_*`，沿腕部 **-Z** 偏移约 8.5cm → **手指伸出方向 = TCP 的 -Z**。

### 初始位姿

默认 **`hold_box`**：先走**关节空间肘朝下就绪**，再**切回 endpose QP**，然后小幅笛卡尔微调（默认前伸 6cm、抬高 5cm）。  
（若不切回末端控制器，握 Grip 时本机有目标但手臂不跟。）

```bash
./scripts/run_tianyee_udp_bridge.sh --prepare   # 需重启桥以加载 go_home_joints
./scripts/run_vr_teleop_tianyee.sh

# 再低一点
./scripts/run_vr_teleop_tianyee.sh --home-offset-xyz 0.06 0 0.0

# 只抬手、不改关节
./scripts/run_vr_teleop_tianyee.sh --home-pose keep
```

关节角可在 `config.py` 的 `HOME_Q_LEFT/RIGHT` 微调。

## 启动

### 1) 机器人：XARM + body + 桥

```bash
# 整机重启后先拉控制栈，再开桥：
./scripts/run_tianyee_robot_stack.sh
./scripts/run_tianyee_udp_bridge.sh --prepare
```

本机消息包见 `third_party/tianyee_ros_ws/`（Jazzy `install/`；可选 `humble_install/` / `config/` DDS）。

### 2) 本机：WebXR + 遥操作

```bash
./scripts/run_full_stack.sh --backend tianyee --no-can-activate --no-publisher
# 或：
./scripts/run_vr_teleop_tianyee.sh
```

可选 `--transport ros`（需本机与机器人 ROS 互通良好，一般不推荐）。

## 文件

| 文件 | 作用 |
|------|------|
| `vr_teleop_dual.py` | WebXR clutch 主循环 |
| `udp_ros_bridge.py` | 机器人 UDP→ROS |
| `ros_endpose.py` | TF / 发布 / 使能 |
| `udp_protocol.py` | UDP JSON |
| `config.py` | 默认帧与端口 |
