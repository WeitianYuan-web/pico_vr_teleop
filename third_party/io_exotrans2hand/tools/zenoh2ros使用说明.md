# Zenoh → ROS2 桥接使用说明

`tools/zenoh2ros_bridge.py` 是**可选调试工具**，把 io_gateway 经 Zenoh 发布的数据转成 ROS2 topic，供 RViz、ros2 bag 等使用。**不是**网关主程序，主程序仍用 `./scripts/run_gateway.sh` 启动。

---

## 1. 使用前准备

| 项目 | 说明 |
|------|------|
| io_gateway | 必须先启动并保持运行 |
| ROS2 | 客户本机需已安装 ROS2（常见：Humble / Jazzy） |
| 工作目录 | 所有命令均在**项目根目录**执行（含 `configs/`、`tools/` 的目录） |

启动网关（终端 A）：

```bash
cd <项目根目录>
./scripts/run_gateway.sh
```

等待外骨骼/手型链路正常、网页或日志中有数据后再开桥接。

---

## 2. 确认本机 ROS 版本

```bash
echo $ROS_DISTRO
# 或
ls /opt/ros/
```

| 常见发行版 | 对应 Python | 推荐用法 |
|-----------|-------------|----------|
| **humble**、**iron** | 3.10 | 见下文 **方案 A** |
| **jazzy**、**rolling** | 3.12 | 见下文 **方案 B** |
| 不确定 | — | 先执行 `python3 --version`，3.10 用方案 A，3.12 用方案 B |

---

## 3. 方案 A：ROS Humble / Iron（Python 3.10）

在**新终端**（终端 B）执行：

```bash
cd <项目根目录>

set +u
source /opt/ros/humble/setup.bash    # iron 则改为 source /opt/ros/iron/setup.bash
set -u

export IO_EXOTRANS2HAND_ROOT="$PWD"
export PYTHONPATH="$PWD/bundle/python/lib/python3.10/site-packages:$PWD/src:$PWD/src/io_bus_proto/generated/python:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="$PWD/bundle/opt/zenoh/lib:${LD_LIBRARY_PATH:-}"

./bundle/opt/python/bin/python3 tools/zenoh2ros_bridge.py
```

> 必须使用项目自带的 **bundle Python 3.10**，不要用系统 `python3`（若系统已是 3.12）。

---

## 4. 方案 B：ROS Jazzy / Rolling（Python 3.12）

Jazzy 与 bundle（3.10）混用会报错，需分开处理：**ROS 用系统 Python，zenoh 单独挂载**。

### 4.1 一次性准备（每台机器只做一次）

```bash
# protobuf（解码 Zenoh 消息需要）
/usr/bin/python3 -m pip install --user --break-system-packages protobuf

# 仅暴露 zenoh 包，避免加载 bundle 里给 3.10 编译的 numpy
ROOT=<项目根目录>
mkdir -p /tmp/zenoh_py312_path
ln -sfn "$ROOT/bundle/python/lib/python3.10/site-packages/zenoh" /tmp/zenoh_py312_path/zenoh
```

将 `<项目根目录>` 换成实际路径；`/tmp/zenoh_py312_path` 可改成固定目录（如 `~/zenoh_py312_path`），重启后需重新执行 `ln -sfn`。

### 4.2 每次启动桥接（终端 B）

```bash
cd <项目根目录>

unset PYTHONPATH

set +u
source /opt/ros/jazzy/setup.bash    # rolling 则改为对应路径
set -u

export IO_EXOTRANS2HAND_ROOT="$PWD"
export PYTHONPATH="$PWD/src:$PWD/src/io_bus_proto/generated/python:${PYTHONPATH}:/tmp/zenoh_py312_path"
export LD_LIBRARY_PATH="$PWD/bundle/opt/zenoh/lib:${LD_LIBRARY_PATH:-}"

/usr/bin/python3 tools/zenoh2ros_bridge.py
```

---

## 5. 运行是否正常

**正常：** 终端打印 `hands: [...]` 及多行 `xxx -> /io_...`，进程**持续运行**不退出。

**验证 ROS topic（终端 C）：**

```bash
set +u && source /opt/ros/<你的发行版>/setup.bash && set -u
ros2 topic list | grep io_
ros2 topic echo /io_esk/joint_data --once
```

**退出桥接：** 在终端 B 按 `Ctrl+C`。

---

## 6. 转发的数据（摘要）

| Zenoh key 示例 | ROS topic |
|----------------|-----------|
| `io_fusion/tf_exoskeleton` | `/io_fusion/tf_exoskeleton` |
| `io_esk/joint_data` | `/io_esk/joint_data` |
| `io_esk/joystick_data` | `/io_esk/joystick_data` |
| `io_esk/imu_data_right` / `left` | `/io_esk/imu_data_right` / `left` |
| `io_align/<手型>/tf_hand` | `/io_align/<手型>/tf_hand` |
| `io_teleop/<手型>/joint_cmd_finger_*` | 对应 `/io_teleop/...` |
| `/io_esk/vibration_feedback`（ROS→Zenoh） | 发布到 `io_esk/vibration_feedback` |

手型列表由程序启动时扫描 Zenoh 上约 1 秒内的 `tf_hand` key 自动发现；需 gateway 已在发布手型数据。

---

## 7. 常见问题

| 现象 | 原因 | 处理 |
|------|------|------|
| `No module named 'rclpy'` | 未加载 ROS 环境，或 PYTHONPATH 覆盖了 ROS 路径 | 先 `source /opt/ros/.../setup.bash`；Jazzy 方案 B 中 PYTHONPATH 末尾保留 `${PYTHONPATH}` |
| `rclpy._rclpy_pybind11` / Python 版本不符 | 3.10 与 3.12 混用 | Humble 用 bundle python；Jazzy 用 `/usr/bin/python3` |
| numpy 导入失败 | 把整个 bundle `site-packages` 给了 3.12 | 改用方案 B，不要整包加入 PYTHONPATH |
| `AMENT_TRACE_SETUP_FILES: 未绑定的变量` | shell 开了 `set -u` | `source` ROS 前执行 `set +u`，之后可 `set -u` |
| `hands: []` 只有全局 topic | gateway 未跑或尚无手型数据 | 先启动 gateway，应用手型后重试 |
| 找不到 zenoh 配置 | 不在项目根目录 | 先 `cd` 到项目根再运行 |

---

## 8. 说明

- 本工具依赖客户**自行安装**的 ROS2，不属于 gateway 默认交付范围。
- 与 gateway 共用同一 Zenoh 配置：`configs/config/zenoh.json5`。
- 文件头注释中的 `--hands` 参数当前版本**未实现**，手型由自动扫描决定。
