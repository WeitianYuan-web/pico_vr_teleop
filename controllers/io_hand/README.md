# controllers/io_hand

IO Gesture 手指重定向 → Inspire RH56F2（本仓 `InspireHandSDK_Y`）。

与 VR 共存：**VR 控臂，IO 控手**。手串口只由本桥占用。

## 依赖

1. 同步 IO 整包（含 bundle）：

```bash
./scripts/sync_io_exotrans2hand.sh
```

2. 编译 Inspire Python 绑定：

```bash
cd third_party/InspireHandSDK_Y
cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON
cmake --build build --target inspire_hand_py
```

3. 本机 ROS2（Jazzy/Humble）+ 项目 `.venv`。

## 数据链

```text
外骨骼手套
  → ./scripts/run_io_gateway.sh          # third_party/io_exotrans2hand
  → Apply Inspire_RH56F2
  → ./scripts/run_io_zenoh2ros.sh        # Zenoh → /io_teleop/.../joint_cmd_finger_*
  → ./scripts/run_io_hand_bridge.sh      # JointState → inspire_hand_py(rh56f2)
  → RH56F2 RS485

VR 臂（并行）:
  ./scripts/run_full_stack.sh --backend piper -- --disable-hands
```

## 话题 / 映射

| 侧 | ROS 话题 | 关节前缀 |
|----|----------|----------|
| 左 | `/io_teleop/Inspire_RH56F2/joint_cmd_finger_left` | `left_` |
| 右 | `/io_teleop/Inspire_RH56F2/joint_cmd_finger_right` | `right_` |

电缸顺序：粉红/无名/中/食/拇指弯/拇指转。rad→寄存器与原 `inspire_rh56f2_teleop_bridge` 一致（见 `mapping.py`）。

## 手桥参数示例

单手（仅右手 `/dev/ttyUSB0`）：

```bash
./scripts/run_io_hand_bridge.sh \
  -p enable_left_hand:=false \
  -p right_serial_port:=/dev/ttyUSB0
```

双手：

```bash
./scripts/run_io_hand_bridge.sh \
  -p left_serial_port:=/dev/ttyUSB1 \
  -p right_serial_port:=/dev/ttyUSB0 \
  -p hand_model:=rh56f2 \
  -p hand_force:=6000 \
  -p hand_speed:=4000 \
  -p log_mapped_positions:=true
```

（脚本会自动补上 `--ros-args`；裸写 `-p` 而不带 `--ros-args` 时，ROS 会把参数当成 remap，左手默认仍会去连 `/dev/ttyUSB1`。）

gateway 的 `probe_exclude_ports` 请排除手串口，避免外骨骼探测占用。

## 模块

| 文件 | 作用 |
|------|------|
| `mapping.py` | JointState → 6 路寄存器 |
| `inspire_sdk_driver.py` | `inspire_hand_py` 封装 |
| `io_rh56f2_bridge.py` | ROS2 订阅节点 |

原 IO 工程自带的裸 RS485 bridge 仍在 `third_party/io_exotrans2hand/scripts/Inspire_Hardware_Bridge/`，日常请用本目录桥。
