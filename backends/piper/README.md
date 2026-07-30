# backends/piper

Piper 双臂 WebXR 遥操作。灵巧手默认**不占串口**：Trigger → `/hand_cmd/*` → 统一手控。

## 布局

```
backends/piper/
├── README.md
└── teleop/
    ├── teleop_piper_webxr.py          # 单/双臂 WebXR + Placo IK
    └── dual_arm_dual_hand_webxr.py    # 双臂 + 手指令发布
```

## 依赖

| 路径 | 随仓？ | 作用 |
|------|--------|------|
| `third_party/pyAgxArm/` | 是 | Piper CAN + Placo IK |
| `third_party/InspireHandSDK_Y/` | 否 | 统一手控 / 姿态端点 |
| ROS2 `rclpy` | 系统 | 发布 `/hand_cmd` |

## 启动

### 推荐：统一手控

```bash
./scripts/run_hand_controller.sh
./scripts/run_full_stack.sh --backend piper
```

### IO 手套控手（关掉 VR 手指令）

```bash
./scripts/run_io_gateway.sh
./scripts/run_io_zenoh2ros.sh
./scripts/run_hand_controller.sh
./scripts/run_full_stack.sh --backend piper -- --disable-hands
```

### LEGACY（本进程直连串口，调试用）

```bash
ROS_ARGS="-p publish_hands_from_udp:=true" \
  ./scripts/run_full_stack.sh --backend piper -- --legacy-direct-hand
```

与 `run_hand_controller.sh` **不要同时开**。

入口：`entrypoints/piper_dual_webxr.py` → `teleop/dual_arm_dual_hand_webxr.py`。

详见 [controllers/io_hand/README.md](../../controllers/io_hand/README.md)。
