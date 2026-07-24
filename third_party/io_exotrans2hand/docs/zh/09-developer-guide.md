# 09 · 二次开发指南

> 适用读者：二次开发者

本章覆盖运维必需的轻量开发：新增一款手型、编写/改编硬件桥接脚本，以及构建脚本速览。更深入的架构改造超出本手册范围。

## 9.1 新增一款手型

### 需要的文件
在 `configs/end_tools/<NewHandName>/` 下准备：

1. `tf_transform_v2.yml` — 外骨骼 link 与新手 URDF link 的 `tf_list` 映射
2. `controller_v2_3_left.yml` / `controller_v2_3_right.yml` — `free_joints`、`task.pose/vector` 与 URDF 一致
3. `urdf/<model>.urdf` — 含 task 引用的 link（各指 `*_tip`、`*_hand_ee_link` 等）
4. `meshes/*.STL` — Web 上传校验与可视化所需

手型名规则：`^[A-Za-z0-9_]{1,128}$`，且应与桥接脚本的 Zenoh/ROS 命名空间一致。

### 步骤
```mermaid
flowchart LR
  P1["准备 3 yml + urdf/ + meshes/"] --> P2["Web 上传 或 复制到 end_tools/"]
  P2 --> P3["GET /hands/configs 校验列出"]
  P3 --> P4["应用手型 (Web 或 hands/select)"]
  P4 --> P5["接入外骨骼 -> 编排自动起 transform/controller"]
  P5 --> P6["(可选) 编写硬件桥接"]
  P6 --> P7["联调: 网关 -> zenoh2ros -> 桥接 -> 实手"]
```

1. **打包上传**（Web 控制台）或手动复制到 `configs/end_tools/<NewHandName>/`。
2. **校验**：`GET /api/v1/hands/configs` 应列出新手型。
3. **应用**：Web「应用」或 `POST /api/v1/hands/select`，body `{"hands":["NewHandName"]}`；同时写入 `gateway.yaml` 的 `hand_choose`。
4. **接入外骨骼**：编排器检测到拓扑后自动启动 `transform@NewHandName` 与对应 controller。
5. **（可选）硬件桥接**：见 9.2。
6. **联调**：查看 `logs/<date>/transform_*.log`、`controller_*.log`。

### 从现有手型复制时的修改要点
| 文件 | 必改项 |
|------|--------|
| `tf_transform_v2.yml` | `tf_list` 中 exo/hand link 名与 RPY 标定 |
| `controller_v2_3_*.yml` | `model.urdf`、`free_joints`、`task` 中 link 名与 scale |
| `urdf/` | 关节命名与 `free_joints`、桥接 `joint_suffix` 对齐 |
| 桥接脚本 | 映射表、寄存器协议、默认 topic 中的 `<HandName>` |

## 9.2 编写/改编硬件桥接脚本

参考现有 `scripts/Inspire_Hardware_Bridge/inspire_rh56f2_teleop_bridge.py`（6 DOF）或 `inspire_rh5dg2_teleop_bridge.py`（13 DOF）。核心是三部分：

### 1) 关节映射表
按「关节角求和 → 归一化 [0,1] → 线性映射到寄存器范围」定义每路手指：
```python
FINGER_MAPPINGS = (
    FingerMapping(('pinky_1_joint',), 1740, 900, 0.0, 1.47),   # (关节, reg@下限, reg@上限, rad下限, rad上限)
    ...
)
```

### 2) 寄存器协议（RS485）
帧头 `0xEB 0x90`，写命令 `0x12`。各型号寄存器地址不同：
| 型号 | angleSet/Setpos | speed | force |
|------|-----------------|-------|-------|
| RH56F2 | 1040 | 1052 | 1046 |
| RH5DG2 | 1080 | 0x0454 | 1093 |
| RH56E2 | 0x05C2（Setpos） | — | — |

### 3) 默认订阅话题
必须与网关发布一致：
```python
self.declare_parameter('right_input_topic', '/io_teleop/<NewHandName>/joint_cmd_finger_right')
self.declare_parameter('left_input_topic',  '/io_teleop/<NewHandName>/joint_cmd_finger_left')
```

> **已知坑**：`Inspire_RH5DG2_control_node.py`（旧版单进程）订阅 `/io_teleop/RH5DG2/...`，与网关实际 `/io_teleop/Inspire_RH5DG2/...` 不符。新写桥接请以 `*_teleop_bridge.py` 为模板，话题用完整手型名。

### 语法自检
```bash
python3 -m py_compile scripts/Inspire_Hardware_Bridge/inspire_<model>_teleop_bridge.py
```

## 9.3 构建脚本（仅构建流水线）

发行包已预编译，运维无需执行。以下仅供从源码构建时参考：

| 脚本 | 用途 | 环境 |
|------|------|------|
| `scripts/cython_build.sh` | 将 `io_gateway` 后端编译为 `.so`（`--strip-py` 删除源 .py） | Ubuntu 22.04 构建容器 + `python3.10-dev` + `Cython>=3.0` |
| `scripts/gen_protobuf.sh` | 用 `protoc` 从 `proto/io_msgs/messages.proto` 生成 C++/Python | `bundle/opt/io-deps/bin/protoc` 可用 |
| `scripts/install_protobuf_bundle.sh` | 从源码编译 libprotobuf + protoc 安装到 bundle | Docker 构建容器，需 `PREFIX/SRC/PY_SITE` |

> 注意：`io_gateway.backend` 多数模块已编译为 `.so`（如 `main`、`config_loader`、`orchestrator/*`、`glove_manager`、`zenoh/bridge`），源码不在发行包内。可读源仅 `main.py`、`api/routes.py` 等少数入口。

## 9.4 可选调试工具

| 工具 | 用途 |
|------|------|
| `tools/zenoh2ros_bridge.py` | 将 Zenoh 数据桥接为 ROS2 话题（硬件桥接前置），详见 `tools/zenoh2ros使用说明.md` |
| `tools/ws2ros_bridge.py` | 将 WebSocket 数据桥接为 ROS2 话题 |

---

返回：[文档首页](../README.md)
