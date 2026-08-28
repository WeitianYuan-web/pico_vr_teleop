# controllers/io_hand

统一 **Inspire 手控执行节点**（默认 RH56F2；可选 RH5DG2）：只占串口、执行指令、发布真实手状态。

指令源（VR / IO）**只发控制数据，不碰串口**。

## 架构

```text
IO 手套 ──zenoh2ros──► /io_teleop/.../joint_cmd_finger_* ──┐
                                                             ├──► rh56f2_controller ──► RS485
VR Trigger ──Piper──► /hand_cmd/{left,right} ───────────────┘              │
                                                                           ▼
                                                              /puppet/hand_left|right
```

同侧以**最新消息**为准。

## 依赖

```bash
./scripts/sync_io_exotrans2hand.sh   # 仅 IO 手套需要
cd third_party/InspireHandSDK_Y
cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON
cmake --build build --target inspire_hand_py
.venv/bin/pip install 'protobuf>=5.28,<6'   # Jazzy zenoh2ros
```

## 启动

### 仅 VR 扳机控手 + Piper 臂

```bash
# 终端 A：统一手控（占串口 + 发 /puppet/hand_*）
./scripts/run_hand_controller.sh \
  -p right_serial_port:=/dev/ttyUSB0 \
  -p left_serial_port:=/dev/ttyUSB1

# 终端 B：VR 臂；Trigger → /hand_cmd（默认，不占串口）
./scripts/run_full_stack.sh --backend piper
```

### IO 手套控手 + VR 臂（Piper 或 JAKA）

```bash
./scripts/run_io_gateway.sh
./scripts/run_io_zenoh2ros.sh   # 须含 joint_cmd_finger_*

./scripts/run_hand_controller.sh \
  -p right_serial_port:=/dev/ttyUSB0 \
  -p left_serial_port:=/dev/ttyUSB1 \
  -p log_mapped_positions:=true

# Piper：关掉 VR 手指令，避免与手套抢最新消息
./scripts/run_full_stack.sh --backend piper -- --disable-hands

# JAKA（无相机示例）
./scripts/run_full_stack.sh --backend jaka --no-publisher --no-can-activate
```

### IO 手套 + RH5DG2（G2）

三个进程必须同一手型。只接手、不启臂：

```bash
export IO_HAND_MODEL=rh5dg2   # 或每个脚本加 --model rh5dg2

./scripts/run_io_gateway.sh
./scripts/run_io_zenoh2ros.sh
./scripts/run_hand_controller.sh \
  -p enable_left_hand:=false \
  -p right_serial_port:=/dev/ttyUSB0
```

手控用 SDK `rh5dg2`、默认波特率 **921600**（F2 仍是 115200）。需要覆盖时加 `-p baud_rate:=...`。

不要同时跑 `inspire_rh5dg2_teleop_bridge.py` 与 `run_hand_controller.sh`。

### gateway 串口排除

```yaml
# third_party/io_exotrans2hand/configs/config/gateway.yaml
probe_exclude_ports:
  - /dev/ttyUSB0
  - /dev/ttyUSB1
hand_choose:
  - Inspire_RH56F2   # G2 改为 Inspire_RH5DG2
```

## 话题

| 方向 | 话题 | 说明 |
|------|------|------|
| 入 | `/hand_cmd/left` `/hand_cmd/right` | VR 等统一指令 |
| 入 | `/io_teleop/Inspire_RH56F2/joint_cmd_finger_*` | F2 |
| 入 | `/io_teleop/Inspire_RH5DG2/joint_cmd_finger_*` | G2 |
| 出 | `/puppet/hand_left` `/puppet/hand_right` | F2: `finger_1..6`；G2: 13 路关节名 |

Publisher 默认 **不再**从 UDP 发手话题（`publish_hands_from_udp:=false`），避免覆盖本节点。

## 模块

| 文件 | 作用 |
|------|------|
| `rh56f2_controller.py` | 统一执行节点（`hand_model` 切 F2/G2） |
| `mapping.py` | JointState ↔ F2 6 路 / G2 13 路 |
| `inspire_sdk_driver.py` | `inspire_hand_py` 封装 |
| `vr_trigger_cmd.py` | Trigger alpha → JointState |

## 常见问题

| 现象 | 处理 |
|------|------|
| 手不动且无 `/io_teleop/...` | 重启 zenoh2ros，确认 `hands` 与 `hand_choose` 同为 F2 或 G2 |
| VR 与 IO 同时动手乱跳 | Piper 加 `--disable-hands` |
| `/puppet/hand_*` 被清空 | 确认 publisher 未设 `publish_hands_from_udp:=true` |
| 抢串口 | 不要用 `--legacy-direct-hand` 与手控同时开 |
