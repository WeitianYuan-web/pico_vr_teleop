# backends/galbot

Galbot G1 双臂 WebXR 遥操作。

**不是** Unitree G1。Unitree 后端仍是 `--backend g1` / `backends/g1/`。本后端是 Galaxy Robotics **Galbot G1**，入口 `--backend galbot`。

## 版本（先看这个）

官方对应关系：

| 机上 GBS | PC 侧 SDK | 末端控制 |
|----------|-----------|----------|
| **1.15.x**（本机机器人当前是 **1.15.15**） | **1.7.3** | 本地 pinocchio IK + `set_joint_commands(t=0)` |
| 1.16 | 1.8 | WBC `set_end_effector_command` |
| 1.17 | 1.9 | WBC `set_end_effector_command` |

WBC 流式末端是 **SDK 1.8 才加的**。GBS 1.15 不能装 1.8/1.9：`init()` 可能成功，但关节/EE 会一直空。

本仓对 1.15 的默认安装目录是 **`/opt/galbot-1.7.3`**，不覆盖已有的 `/opt/galbot`（1.9.1）。启动脚本优先找 1.7.3。

1.7 没有 WBC 流式末端；笛卡尔规划器执行会 FAULT。遥操作热路径是 **本机 pinocchio IK（标定 URDF）+ 50 Hz `set_joint_commands(t=0)`**。HPU `inverse_kinematics` 只用于启动回位。要真正笛卡尔伺服，最终应升 GBS 到 1.17。

## 架构

```text
PICO → webxr/server.py (WSS)
     → entrypoints/galbot_dual_webxr.py
     → backends/galbot/vr_teleop_dual.py
     → Galbot SDK (Embosa / FastDDS)
     → SDK 1.8+ : set_end_effector_command(lee + ree)
        SDK 1.7 : 本地 pinocchio IK → set_joint_commands(t=0)
状态：遥操作 → UDP 127.0.0.1:17981 → publisher（本机 Domain 42）
```

SDK 源码树在 `third_party/GalbotSDK-V1.7.3/` 或 `GalbotSDK-main/`（不入库）。仅有源码树不够跑真机：`install.sh` 会把 boost / embosa / opencv 等装进安装目录。

| 操作 | 说明 |
|------|------|
| Grip | 接合该侧，增量跟随手柄平移与姿态 |
| 松开 Grip | 锁定松开时的 EE 目标，继续发送 |
| B | 关节空间回到启动初始位（位置+姿态） |
| Trigger | 首版不控手；灵巧手走统一 `run_hand_controller.sh` |

## 坐标系

默认 **`x_forward`**（与 Unitree G1 / tianyee 相同）。左右臂 `axis_sign` 先按 `1 1 1`；**尚未实机确认**。若某侧前后/左右反了：

```bash
./scripts/run_full_stack.sh --backend galbot -- --axis-sign-left -1 -1 1
```

Galbot 位姿是 `[x,y,z,qx,qy,qz,qw]`（xyzw）；本仓内部仍用 wxyz，进出 SDK 时转换。

EE 坐标系：

- 读：`left_arm_end_effector_mount_link` / `right_arm_end_effector_mount_link`
- 1.7 写：链名 `left_arm` / `right_arm`（官方示例如此）
- 1.8+ WBC key：`lee_pose` / `ree_pose`

## 网络 / Embosa

Galbot 不用 ROS RMW。PC ↔ 机器人走 Embosa（自带 FastDDS）。默认：

| 端 | IP |
|----|-----|
| PC | `192.168.1.99` |
| XCU | `192.168.1.66` |
| HPU | `192.168.1.88` |

配置写在 **`/data/config/embosa_ip_config.json`**（`system.cfg` 里 `device_type: pc`）。SDK 目录一键脚本：

```bash
cd third_party/GalbotSDK-V1.7.3
sudo ./configure_embosa_ip.sh
```

`run_full_stack.sh --backend galbot` 会对**遥操作进程** unset `ROS_DOMAIN_ID` / `FASTRTPS_*`，避免本机 Domain 42 isolation 把 Embosa 锁在 loopback。publisher 仍走本机隔离。

## 依赖

本机机器人 GBS **1.15.15** 时：

```bash
# 1) SDK 源码树（已 clone 到 third_party/GalbotSDK-V1.7.3）
# 2) 安装 native 依赖到独立目录，不要覆盖 /opt/galbot 的 1.9
cd third_party/GalbotSDK-V1.7.3
sudo ./install.sh --platform linux-x86_64-gcc940 --install-dir /opt/galbot-1.7.3 -y
# 3) Embosa IP（cp -n，不会覆盖已有 json）
sudo ./configure_embosa_ip.sh
```

若机上已升到 GBS 1.17，再装 1.9 到 `/opt/galbot`，并 `export GALBOT_HOME=/opt/galbot`。

本机 Python 3.12 可直接用 SDK 自带的 `galbot_sdk.cpython-312-*.so`。启动脚本会 `source …/setup.sh`。

覆盖安装目录：

```bash
export GALBOT_HOME=/opt/galbot-1.7.3
```

## 启动

默认把双臂送到配置里的 **7 关节初始位**。同一套末端 xyz 可以对应拧着的肘/腕；只对齐笛卡尔会“坐标对、看起来乱”。若只要抬末端、不管关节：`-- --home-use-cartesian`。

```bash
# 一键（WebXR + 遥操作 + publisher）
./scripts/run_full_stack.sh --backend galbot

# 仅遥操作
./scripts/run_vr_teleop_galbot.sh

# 无机器人，只测 clutch / WSS
./scripts/run_vr_teleop_galbot.sh --dry-run --no-publish-state
```

单臂：

```bash
./scripts/run_full_stack.sh --backend galbot -- --hands right
```

跟手关节限速 / 回位速度：

```bash
./scripts/run_full_stack.sh --backend galbot -- --teleop-max-rad-s 0.45 --home-speed-rad-s 0.75
```

## 注意

1. **不要**和 `--backend g1`（Unitree）同时跑，DDS/网段会打架。
2. 未跑过 `install.sh` 时 `import galbot_sdk` 会缺 `libembosa` / `libboost_thread`。
3. 旧版 `install.sh` 可能把 `source /opt/galbot/.../setup.sh` 写进 `~/.bashrc`。Galbot 的 FastCDR 和 ROS Jazzy 不兼容，同一终端里直接跑 `ros2` / publisher 会 `symbol lookup error: serializeEj`。`run_full_stack.sh` 会在 publisher / WebXR 进程里剥掉 Galbot 库路径；遥操作子进程再单独 source SDK。
4. 手控与臂控解耦：Galbot 机载灵巧手 API（`set_dexhand_command`）首版不用；外接 Inspire 仍走本仓 RS485 手控。
5. SDK 1.7 的笛卡尔走规划器，**不是** WBC 流。规划中途再发新目标可能打断上一段；手感差是版本限制，不是 clutch 写错。
