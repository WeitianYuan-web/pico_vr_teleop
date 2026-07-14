# Piper 遥操作说明（Placo QP 版）

本文档说明当前仓库中 Piper 遥操作链路的实现与使用方式。  
当前已统一为 **Placo 多任务加权 QP 微分逆运动学**，不再使用旧的 Pinocchio 逆解脚本。

---

## 1. 当前架构

### 1.1 逆解后端

- 核心模块：`piper_placo_qp_ik.py`
- 求解器：`placo.KinematicsSolver`
- 任务组合：
  - `frame_task`：末端位姿任务
  - `manipulability_task`：可操作度任务（抑制奇异位形）
  - `joints_regularization_task`：关节正则项（抑制突变）
- 输出控制：`move_j`（关节空间命令）

### 1.2 入口脚本

- 键盘遥操作：`run_piper_keyboard_teleop.py`
- WebXR 遥操作：`../backends/piper/teleop/teleop_piper_webxr.py`
- 启动脚本：`run_piper.sh`

---

## 2. 依赖与安装

### 2.1 额外依赖

`requirements-teleop.txt` 当前内容：

- `python-can>=3.3.4`
- `numpy>=1.24`
- `placo`

### 2.2 安装建议

```bash
cd pyAgxArm
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements-teleop.txt
pip install -e .
```

---

## 3. CAN 与基础启动

推荐使用已有脚本自动处理 CAN 激活：

```bash
cd pyAgxArm
./run_piper.sh monitor
```

也可以手动激活（Linux 示例）：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

---

## 4. 键盘遥操作（Placo QP）

### 4.1 快速运行

```bash
cd pyAgxArm
./run_piper.sh teleop
```

等价命令：

```bash
python run_piper_keyboard_teleop.py --can_port can0
```

### 4.2 键位

- 平移：`W/S`（X±），`A/D`（Y±），`Q/E`（Z±）
- 姿态：**锁定不变**（仅控制末端位置）
- 步长：`[` `]`（平移）
- 复位目标：`z`（并刷新姿态锁）
- 帮助：`h`
- 退出：`c`

### 4.3 关键参数

- `--pos-step`：平移增量（米）
- `--qp-dt`：QP 迭代时间步
- `--max-joint-step`：单步关节限幅（弧度，防突跳）
- `--wait-motion`：每步等待到位（更稳但更慢）
- `--init-joints`：启动先执行的绝对关节角（move_j，6个 rad）
- `--init-joint-timeout`：初始关节姿态等待到位超时
- `--init-x --init-y --init-z`：启动时末端全局绝对位置（m）
- `--init-wait`：启动初始姿态等待时间（秒）
- `--disable-on-exit`：退出时执行失能；默认仅断开连接，不失能

---

## 5. WebXR 遥操作（Placo QP）

### 5.1 运行顺序

1) 启动 WebXR 数据服务（`webxr_test`）  
2) PICO 打开采集页并进入 VR  
3) 启动机械臂控制脚本

### 5.2 命令

```bash
# 终端1
cd webxr_test
python server.py

# 终端2
cd webxr_test
python scripts/teleop_piper_webxr.py
```

### 5.3 控制逻辑

- 仅使用新版协议字段：
  - `controllers[].{grip,trigger,x,y,z,qx,qy,qz,qw}`
- `grip`：离合（按住才控制机械臂）
- `trigger`：夹爪开合
- 默认 **6DOF**：Grip 离合时平移 + 旋转同步控制
- `A` 键：可选 `--rotation-mode hold-a` 时启用旋转
- `B` 键（右手柄）：回到启动初始位姿（两阶段回零）
- 增量位姿映射 + EMA 平滑 + QP 求解 + 关节/姿态限幅
- 上电初始化后会先执行“轻微抬起”姿态，避免贴地奇异
- 退出时默认 **不失能**；需要时可加 `--disable-on-exit`

---

## 6. 已移除/不再使用

以下旧脚本已删除，不再作为遥操作路径：

- `piper_pinocchio_kinematics.py`

说明：当前链路统一为 Placo QP；若需要旧逻辑，请使用历史提交恢复。

---

## 7. 常见问题

### 7.1 报错 `ImportError: placo`

请确认当前 Python 环境已安装 `placo`，并使用了正确的虚拟环境。

### 7.2 机械臂动作抖动

优先调整：

- `--max-joint-step`（减小）
- `--qp-dt`（适当减小）
- `--pos-step` / `--rot-step-deg`（减小）

### 7.3 WebXR 数据断流或卡顿

先参考 `webxr_test/TROUBLESHOOTING.md`（尤其是网卡省电设置与链路诊断）。

---

## 8. 版本建议

若后续需要进一步增强稳定性，可增加：

- 速度/加速度软约束任务
- 末端速度限制（基于 `dt` 的自适应限幅）
- 网络抖动时的命令冻结与平滑恢复策略

