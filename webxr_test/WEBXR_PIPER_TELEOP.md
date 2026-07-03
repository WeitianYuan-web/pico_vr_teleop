# WebXR -> Piper 遥操作说明（Placo QP 版）

本文档对应脚本：`pico_vr_teleop/webxr_test/scripts/teleop_piper_webxr.py`  
目标：用 PICO WebXR 手柄遥操作 Piper 机械臂（`move_j`），支持单臂/双臂，并保证稳定性与安全性。

---

## 1. 控制架构

当前链路：

- 前端采集：`webxr_test/index.html`（WebXR 上传 **45 Hz**）
- 数据服务：`webxr_test/server.py`（WSS 8081）
- 机械臂控制：`webxr_test/scripts/teleop_piper_webxr.py`（控制环 **100 Hz**）
- IK 后端：`pyAgxArm/piper_placo_qp_ik.py`（Placo 多任务加权 QP）

控制输出均为 **关节空间命令**：`robot.move_j(...)`。

### 1.1 单臂 / 双臂模式

| 模式 | 参数 | 说明 |
|------|------|------|
| 双臂（默认） | `--hands both` | 左手柄 → 左臂，右手柄 → 右臂 |
| 单右臂 | `--hands right` | 仅连接并控制右臂，右手柄操作 |
| 单左臂 | `--hands left` | 仅连接并控制左臂，左手柄操作 |

单臂模式下不会尝试连接另一台机械臂，适合只有一个 CAN 口的场景。

### 1.2 控制点与坐标系

| 项目 | 内容 |
|------|------|
| 控制点 | `link6` 法兰中心（ee 帧） |
| 位姿定义 | `[x,y,z]` 米 + 四元数/旋转矩阵；与法兰系差一个 Z 轴 90° 修正（`TCP_OFFSET_POSE` 默认 `[0,0,0,0,0,π/2]`） |
| 关节限位 | IK 内按 URDF 6 轴上下限硬约束剪裁 |
| 姿态精度 | 位置权重 **1.0** > 姿态权重 **0.1**（姿态可被牺牲） |
| 速度限制 | 上层插值 `max_pos_speed=0.8 m/s`；关节步进 `MAX_JOINT_STEP_RAD` |

### 1.3 平移 / 姿态控制方式

| 维度 | 方式 | 说明 |
|------|------|------|
| **平移** | 增量 | Grip 接合时记录 `ref_ee_xyz`，手柄位移增量叠加 |
| **姿态** | 绝对 | Grip 接合时记录控制器参考姿态，后续按相对参考的角轴增量映射到 ee |

姿态绝对控制流程：

1. Grip 接合瞬间：记录控制器参考姿态 `ref_controller_quat`
2. 每周期：`ctrl_delta = angle_axis(ref_controller_quat -> current_ctrl_quat)`
3. 轴向符号修正：`ctrl_delta = ctrl_delta * ROT_AXIS_SIGN`
4. 绝对姿态目标：`target_quat = apply_delta(ref_ee_quat, ctrl_delta)`
5. **朝前保护**：目标姿态限定在启动时 ee 初始朝向的 **60°** 以内（`MAX_ROT_RANGE_RAD`）

---

## 2. 启动顺序

### 2.1 启动 WebXR 数据服务

```bash
cd /home/b0106/pico_test/pico_vr_teleop/webxr_test
/home/b0106/pico_test/pico_vr_teleop/.venv/bin/python server.py
```

### 2.2 PICO 打开采集页并进入 VR

在 PICO 浏览器访问：

```text
https://<你的电脑IP>:8000/index.html
```

进入 VR 后，确保页面显示已开始发送数据。

### 2.3 启动机械臂控制脚本

推荐使用项目内启动脚本（会固定使用 `pico_vr_teleop/.venv`）：

```bash
cd /home/b0106/pico_test/pico_vr_teleop
./scripts/run_vr_teleop.sh
```

#### 常用启动示例

```bash
# 单右臂（一个 CAN 口）
./scripts/run_vr_teleop.sh --hands right --right-can-port can0

# 单左臂
./scripts/run_vr_teleop.sh --hands left --left-can-port can0

# 双臂（两个 CAN 口）
./scripts/run_vr_teleop.sh --hands both --left-can-port can0 --right-can-port can1

# 带 TCP 偏移（ee 帧 Z 轴 90° 为默认，可覆盖）
./scripts/run_vr_teleop.sh --hands right --tcp-offset 0,0,0,0,0,1.5708
```

手工运行（需先激活虚拟环境）：

```bash
cd /home/b0106/pico_test/pico_vr_teleop
source .venv/bin/activate
pip install -r pyAgxArm/requirements-teleop.txt
pip install -e pyAgxArm
cd webxr_test
python scripts/teleop_piper_webxr.py --hands right --right-can-port can0
```

---

## 3. 命令行参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--hands` | `both` | `both` / `left` / `right` |
| `--left-can-port` | 自动检测 | 左臂 CAN 端口（如 `can0`） |
| `--right-can-port` | 自动检测 | 右臂 CAN 端口（如 `can1`） |
| `--robot-model` | `piper_h` | 机械臂型号 |
| `--rotation-mode` | `always` | `always` / `hold-a` / `off` |
| `--rotation-scale` | `1.0` | 旋转灵敏度（绝对模式下影响较小） |
| `--tcp-offset` | `0,0,0,0,0,0` | 法兰→TCP 偏移 `x,y,z,roll,pitch,yaw`（m/rad） |
| `--disable-on-exit` | 关闭 | 退出时执行 `disable()` |

---

## 4. 两阶段初始化

脚本启动后（每台已启用的机械臂）按以下顺序执行：

### 阶段1：先到抬起关节姿态

```python
robot.move_j(INIT_JOINTS)
wait_motion_done(...)
```

默认：`INIT_JOINTS = [0.0, 0.35, -0.35, 0.0, 0.0, 0.0]`

### 阶段2：再到绝对末端位姿（可选）

若 `INIT_ABS_X/Y/Z` 非 `None`，则用 Placo QP 求解并 `move_j` 到目标 XYZ。

---

## 5. 手柄控制语义

脚本接受字段：

```text
controllers[].{grip,trigger,x,y,z,qx,qy,qz,qw,buttons}
```

| 输入 | 功能 |
|------|------|
| `grip` | 离合键（>0.5 进入控制，松开停止） |
| `trigger` | 夹爪开合 |
| `A` 键 | `--rotation-mode hold-a` 时，需 Grip+A 才启用姿态 |
| `B` 键 | 回到启动时的初始位姿（边沿触发，2 秒冷却） |
| `x,y,z,qx,qy,qz,qw` | 手柄位姿 |

### 离合控制流程

1. 按下 `grip`：记录机械臂末端参考位姿 `ref_ee` 与控制器参考姿态 `ref_controller_quat`
2. 按住期间：
   - 平移：`target_xyz = ref_ee_xyz + delta_xyz`（增量）
   - 姿态：`target_quat = apply_delta(ref_ee_quat, ctrl_delta)`（绝对）
3. 松开 `grip`：停止更新，机械臂保持

### 旋转模式

```bash
# 默认：Grip 时平移 + 绝对姿态
./scripts/run_vr_teleop.sh --rotation-mode always

# 仅 Grip 平移，按住 A 才跟手姿态
./scripts/run_vr_teleop.sh --rotation-mode hold-a

# 仅平移，姿态锁定为接合时 ee 朝向
./scripts/run_vr_teleop.sh --rotation-mode off
```

### 坐标映射（与 XRoboToolkit 一致）

```python
R_HEADSET_TO_WORLD = [
    [0, 0, -1],
    [-1, 0, 0],
    [0, 1, 0],
]
```

含义（WebXR Y-up → 机器人 Z-up）：

- 手柄前向（-Z）→ 机器人 +X
- 手柄右向（+X）→ 机器人 -Y
- 手柄上向（+Y）→ 机器人 +Z

---

## 6. 稳定性策略

1. **Placo 多任务 QP**
   - 位置任务（权重 1.0）
   - 全姿态任务（权重 0.1）
   - manipulability 任务
   - joints regularization

2. **双环频率**
   - WebXR 数据接收：45 Hz（`index.html`）
   - 机械臂控制环：100 Hz（`control_loop`）
   - 两帧 WebXR 之间由控制环补点

3. **EMA 平滑**
   - 平移增量、绝对姿态分别滤波

4. **关节插值**
   - `JOINT_INTERP_ALPHA` 一阶插值 + 单步限幅

5. **速度限幅**
   - 笛卡尔位置：`MAX_POS_SPEED = 0.8 m/s`
   - 关节步进：`MAX_JOINT_STEP_RAD = 0.03 rad`（100 Hz 下约 3 rad/s）

6. **朝前保护**
   - 姿态目标限定在初始 ee 朝向 ±60° 内

---

## 7. 可调参数（脚本顶部常量）

| 常量 | 默认值 | 说明 |
|------|--------|------|
| `SCALE_FACTOR` | `1.0` | 手柄位移映射比例 |
| `CMD_RATE_HZ` | `100` | 控制频率 |
| `MAX_JOINT_STEP_RAD` | `0.03` | 关节步长限幅 |
| `MAX_POS_SPEED` | `0.8` | 位置速度上限 (m/s) |
| `JOINT_INTERP_ALPHA` | `0.75` | 关节插值系数（越大越跟手） |
| `MAX_ROT_RANGE_RAD` | `60°` | 姿态朝前保护范围 |
| `TCP_OFFSET_POSE` | `[0,0,0,0,0,π/2]` | ee 帧相对法兰 Z 转 90° |
| `INIT_JOINTS` | `[0, 0.35, -0.35, 0, 0, 0]` | 初始关节角 |
| `INIT_ABS_X/Y/Z` | `None` | 绝对末端初始 XYZ |

---

## 8. 退出行为

默认退出时 **不会失能**，仅断开连接。

```bash
./scripts/run_vr_teleop.sh --disable-on-exit   # 退出时 disable
```

---

## 9. 常见问题

### 9.1 CAN 口 `Device is DOWN`

启动脚本 `scripts/run_vr_teleop.sh` 会**自动激活**命令行里出现的 CAN 口
（`--left-can-port` / `--right-can-port`），无需手动操作：

```bash
# 启动时自动激活 can0（DOWN 时拉起，默认 bitrate=1000000）
./scripts/run_vr_teleop.sh --hands right --right-can-port can0

# 自定义波特率
CAN_BITRATE=500000 ./scripts/run_vr_teleop.sh --hands right --right-can-port can0

# 跳过自动激活（已手动配置时）
./scripts/run_vr_teleop.sh --hands right --right-can-port can0 --no-can-activate
```

自动激活会调用 `pico_vr_teleop/pyAgxArm/pyAgxArm/scripts/ubuntu/can_activate.sh`（找不到会回退 linux 版本），需要 `sudo` 权限
（首次可能提示输入密码）。

手动激活（等效操作）：

```bash
sudo bash /home/b0106/pico_test/pico_vr_teleop/pyAgxArm/pyAgxArm/scripts/ubuntu/can_activate.sh can0 1000000
# 或
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000 restart-ms 100
sudo ip link set can1 up
```

单 CAN 口时请用 `--hands right` 或 `--hands left`，不要配双臂。

### 9.2 `package://... mesh could not be found`

`piper_placo_qp_ik.py` 已做 URDF `package://` 自动重写。

### 9.3 连接到错误服务

脚本固定连接 `wss://localhost:8081`，请启动 `server.py`。

### 9.4 手柄无控制

确认：PICO 已进入 VR、`grip` 已按下、数据字段为新版协议、对应手的机械臂已连接。

### 9.5 运动偏慢 / 偏快

可调：`SCALE_FACTOR`、`MAX_POS_SPEED`、`JOINT_INTERP_ALPHA`、脚本内 `set_speed_percent(60)`。
