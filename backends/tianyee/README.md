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
| Grip | 接合该侧，增量跟随手柄 |
| 松开 Grip | 保持当前末端 |
| B | 用当前 TCP 刷新保持位姿 |

坐标系：`waist_yaw_link` → `left_tcp_link` / `right_tcp_link`（米制）。

## 启动

### 1) 机器人：XARM + body + 桥

```bash
# 机器人上已有 body_control、tianyi2.launch.py hardware:=real
# 本机一键拉起桥（会 enable/mode3/auto_switch）：
./scripts/run_tianyee_udp_bridge.sh --prepare
```

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
