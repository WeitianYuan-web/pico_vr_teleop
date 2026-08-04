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

`run_full_stack.sh --backend tianyee` 还会把本机 ROS 默认隔离到 Domain 42，Fast DDS
仅使用 loopback/SHM；机器人保持 Domain 0。本机 publisher 从 `127.0.0.1:17981`
接收遥操作状态，不需要进入机器人 ROS 图。需要临时关闭隔离时显式设置
`TIANYEE_ROS_ISOLATION=0`。

| 操作 | 说明 |
|------|------|
| Grip | 接合该侧，增量跟随手柄平移与姿态（滞回 + 死区） |
| 松开 Grip | 机器人 bridge 立即读取实际 TCP 并锁住，不再追松开前的旧目标 |
| A/X（按住） | 仅在手动选择 `--rotation-mode hold-a` 时用于姿态离合 |
| B | 左右臂直接回到配置的固定默认 TCP（与启动目标相同） |

稳定性相关默认在 `config.py`（`--pos-deadzone-m`、`--max-cmd-step-m`、`--release-freeze-s`、`--rotation-mode` 等可覆盖）。再次 Grip 时会从 bridge 重新读取实际 TCP 作基准，避免释放锁住后使用旧目标导致跳变。

bridge 还有默认 0.15s 的 active UDP watchdog：遥操进程或网络意外中断时，机器人侧会自动锁住当前 TCP。可在 bridge 启动时用 `--watchdog-timeout-s` 调整。

### TCP 坐标系

整机：`+X` 前、`+Y` 左、`+Z` 上。控制帧：`waist_yaw_link` → `left/right_tcp_link`。

URDF 中 `*_tcp_link` 固连于 `wrist_roll_*`，沿腕部 **-Z** 偏移约 8.5cm → **手指伸出方向 = TCP 的 -Z**。

### 初始位姿

默认 **`hold_box`**：笛卡尔到固定默认 TCP `(0.35, ±0.35, 0.08)`，姿态为 **TCP -Z 朝前**（左 RPY `(90,0,-90)`，右 `(-90,0,90)`，掌心相对）。启动与 **B** 共用。
位置与朝向都已接近时才跳过运动。

```bash
./scripts/run_tianyee_udp_bridge.sh --prepare
./scripts/run_vr_teleop_tianyee.sh

# 改固定默认位（腰系米）
./scripts/run_vr_teleop_tianyee.sh \
  --home-xyz-left 0.35 0.35 0.08 \
  --home-xyz-right 0.35 -0.35 0.08

# 肘姿态异常时才加：先关节就绪再笛卡尔
./scripts/run_vr_teleop_tianyee.sh --home-joints-first
```

关节角见 `HOME_Q_LEFT/RIGHT`；默认 TCP 改 `DEFAULT_HOME_XYZ_*`。

## 启动

### 1) 机器人：持久安装 UDP 桥（推荐，开机自启）

把桥接代码拷到机器人 `/home/ubuntu/pico_vr_teleop_tianyee`，安装 systemd 服务 `tianyee_udp_bridge.service`：

```bash
./scripts/run_tianyee_bridge_install.sh
```

安装两个开机服务（会等官方 `proc_manager` 自检到 Running，避免抢跑导致反复 Initing）：
- `tianyee_xarm.service`：等 body 就绪后拉起 `tianyi2_bringup`
- `tianyee_udp_bridge.service`：等 XARM 就绪 → 检查掉线 → 监听 UDP `:19011` 并写 `status.json`（默认**不开** `--prepare`，以免干扰官方自检）

```bash
# 本机查状态（UDP）
python3 scripts/query_tianyee_bridge_status.py
# 或 SSH 读文件
python3 scripts/query_tianyee_bridge_status.py --ssh

# 机器人上
ssh ubuntu@192.168.41.1 'systemctl status tianyee_udp_bridge; cat ~/pico_vr_teleop_tianyee/status.json'
```

更新代码后重新执行 `./scripts/run_tianyee_bridge_install.sh` 即可覆盖并重启服务。
卸载：`./scripts/run_tianyee_bridge_install.sh --uninstall`

> 仍可用一次性脚本：`./scripts/run_tianyee_robot_stack.sh` + `./scripts/run_tianyee_udp_bridge.sh --prepare`
> **不要**用本机 Jazzy `--transport ros` 狂扫话题（易触发 DDS 反序列化异常 → OOM → 电机 33072 掉线）。

本机采集 publisher 只使用标准 ROS2 消息，不需要天轶自定义消息包。
天轶运维脚本在 `backends/tianyee/scripts/`（`scripts/run_tianyee_*.sh` 为薄入口）。

机器人 bridge 的只读 `get_state` 会把 `/joint_states` 中左右臂各 7 个真实关节角和
TCP 位姿传回本机；遥操作使用独立 UDP 状态线程转发到 `127.0.0.1:17981`，publisher
再发布 `/puppet/joint_left|right` 与 `/puppet/end_pose_left|right`。状态轮询不暂停
控制 watchdog，也不进入机器人 DDS 网络。

### 2) 本机：WebXR + 遥操作

```bash
./scripts/run_full_stack.sh --backend tianyee  # 启用本机隔离 ROS publisher
# 不需要本机采集话题时才加 --no-publisher
# 或：
./scripts/run_vr_teleop_tianyee.sh
```

## 文件

| 文件 | 作用 |
|------|------|
| `vr_teleop_dual.py` | WebXR clutch 主循环 |
| `udp_ros_bridge.py` | 机器人 UDP→ROS |
| `robot_status.py` | 臂健康监控 → `status.json` / UDP `get_status` |
| `robot_service/` | 机器人 systemd 安装文件 |
| `scripts/` | 安装/启动 bridge、查状态（根目录 `scripts/run_tianyee_*.sh` 为薄入口） |
| `ros_endpose.py` | TF / 发布 / 使能（仅机器人 bridge / 可选 ROS 直连） |
| `udp_protocol.py` | UDP JSON |
| `config.py` | 默认帧与端口 |
