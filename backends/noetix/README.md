# backends/noetix

Noetix M1 双臂 WebXR 遥操作（本机 ROS2 / CycloneDDS → `/Cartesian_Cmd_Topic`）。

## 架构

```text
PICO → webxr/server.py (WSS)
     → entrypoints/noetix_dual_webxr.py
     → backends/noetix/vr_teleop_dual.py
     → CycloneDDS (本机 192.168.127.40 ↔ 机器人 192.168.127.20)
     → /Control_Mode_Topic /Motor_Cmd_Topic /Cartesian_Cmd_Topic
状态：遥操作 → UDP 127.0.0.1:17981 → publisher（本机 Domain 42）
```

依赖工作空间：`third_party/cartesian_min_ws`（需已 `colcon build`，含 `noetix_m1` 消息）。

| 操作 | 说明 |
|------|------|
| Grip | 接合该侧，增量跟随手柄平移与姿态 |
| 松开 Grip | 锁定松开时的 EE 目标 |
| B | 保持 mode2，笛卡尔 EE 插值回启动 home（不切 mode1） |
| Trigger | 首版不控夹爪（夹爪保持打开） |

## 坐标系

默认 **`x_forward`**（与 G1/tianyee 相同）：

| 臂 | axis_sign | 说明 |
|----|-----------|------|
| 右 | `1 1 1` | 实机确认正确 |
| 左 | `-1 -1 1` | 前后、左右均反，上下正常 |

```bash
./scripts/run_full_stack.sh --backend noetix
# 覆盖左臂符号示例：
./scripts/run_full_stack.sh --backend noetix -- --axis-sign-left -1 -1 1
```

## 网络 / RMW

- 本机有线网卡需有 `192.168.127.40`（`cyclonedds.xml` 绑定该地址）
- 对端机器人：`192.168.127.20`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI=file://.../cartesian_min_ws/.../cyclonedds.xml`

本机常见配置（网线接到 `enp12s0` 后）：

```bash
sudo ip link set enp12s0 up
sudo ip addr add 192.168.127.40/24 dev enp12s0
```

`cartesian_min_ws` 必须在本机 `colcon build`（勿直接拷贝他机 `install/`，其中 symlink 会断）。

`run_full_stack.sh --backend noetix` 会对**遥操作进程**单独注入上述 env，**不会**套 Domain 42 FastDDS（那只给本机 publisher）。

## 启动

```bash
# 确认 cartesian_min_ws 已编译
source /opt/ros/jazzy/setup.bash
source third_party/cartesian_min_ws/install/setup.bash

./scripts/run_full_stack.sh --backend noetix
# 或
./scripts/run_vr_teleop_noetix.sh
```

跳过抱箱、直接切笛卡尔：

```bash
./scripts/run_full_stack.sh --backend noetix -- --skip-box-pose
```

B 回位默认保持 mode2，只改 EE 目标；可调插值时长：

```bash
./scripts/run_full_stack.sh --backend noetix -- --cart-home-interp-s 6
```

## 文件

| 文件 | 作用 |
|------|------|
| `config.py` | 默认关节/步长/网络路径 |
| `ros_cartesian.py` | Status 订阅与 mode/MIT/cartesian 发布 |
| `vr_teleop_dual.py` | WebXR clutch + 启动/回位序列 |
